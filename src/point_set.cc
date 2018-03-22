/*
 * Copyright 2018 Justin Manley and Joseph Bolling.
 */
#include "point_set.h"
#include "sba_utils.h"

#include <iostream>
#include <memory>

#include <opencv2/features2d.hpp>

namespace ggck {

// Converts from an array of dense points and matches to a PointMatches
// structure
PointMatches DensePointsAndMatchesToPointMatches(
    DensePointsAndMatches dpMatches) {
  PointMatches pm;
  pm.reserve(dpMatches.matches.size());

  for (auto& dpMatch : dpMatches.matches) {
    std::pair<cv::Point2d, cv::Point2d> pair;
    pair.first = dpMatches.kp1[dpMatch.queryIdx].pt;
    pair.second = dpMatches.kp2[dpMatch.trainIdx].pt;
    pm.push_back(pair);
  }

  return pm;
}

PointSet::PointSet(const std::vector<ImageMetadata>& metadataList)
    : metadataList(metadataList) {
  numMatches = 0;
}

void PointSet::Add(PointMatches matches, ImagePair metadata) {
  std::pair<int, int> indices;

  // determine which index to assign to metadata
  for (int i = 0; i < metadataList.size(); i++) {
    if (metadata.first.Filename() == metadataList[i].Filename()) {
      indices.first = i;
    } else if (metadata.second.Filename() == metadataList[i].Filename()) {
      indices.second = i;
    }
  }

  // store matches and calculated indices
  points.push_back(matches);
  imageIndices.push_back(indices);
  numMatches += static_cast<int>(matches.size());
}

void PointSet::Add(DensePointsAndMatches dpMatches, ImagePair metadata) {
  this->Add(DensePointsAndMatchesToPointMatches(dpMatches), metadata);
}

/*
 * From the SBA library:
 * measurements vector: (x_11^T, .. x_1m^T, ..., x_n1^T, .. x_nm^T)^T where
 * x_ij is the projection of the i-th point on the j-th image.
 * NOTE: some of the x_ij might be missing, if point i is not visible in image
 * j;
 * see vmask[i, j], max. size n*m*mnp
 */
SbaMeasurementInfo PointSet::GetSbaMeasurementInfo() {
  int numFrames = metadataList.size();

  // initialize mask with zeros
  std::vector<char> mask(numMatches * metadataList.size());
  std::fill(mask.begin(), mask.end(), 0);

  // initialize measurement vector - each 3D point is observed exactly twice
  // and has two parameters
  std::vector<double> measurements(numMatches * 2 * 2);

  // iterate over all points
  int matchIndex = 0;
  for (int i = 0; i < imageIndices.size(); i++) {
    if (imageIndices[i].first < imageIndices[i].second) {
      for (auto const& match : points[i]) {
        // Store 2d point data in measurements array, ordered by index
        measurements[matchIndex * 4] = match.first.x;
        measurements[matchIndex * 4 + 1] = match.first.y;
        measurements[matchIndex * 4 + 2] = match.second.x;
        measurements[matchIndex * 4 + 3] = match.second.y;

        // mark that point is visible in these images in mask array
        mask[matchIndex * numFrames + imageIndices[i].first] = 1;
        mask[matchIndex * numFrames + imageIndices[i].second] = 1;

        matchIndex++;
      }
    } else if (imageIndices[i].second < imageIndices[i].first) {
      for (auto const& match : points[i]) {
        // Store 2d point data in measurements array, ordered by index
        measurements[matchIndex * 4] = match.second.x;
        measurements[matchIndex * 4 + 1] = match.second.y;
        measurements[matchIndex * 4 + 2] = match.first.x;
        measurements[matchIndex * 4 + 3] = match.first.y;

        // mark that point is visible in these images in mask array
        mask[matchIndex * numFrames + imageIndices[i].first] = 1;
        mask[matchIndex * numFrames + imageIndices[i].second] = 1;

        matchIndex++;
      }
    }
  }
  return SbaMeasurementInfo{measurements, mask};
}

// Returns an initial parameter estimate vector for use with SBA
// if cnp = 11: constructs vector assuming a 5-DoF Camera projection matrix
//    (f, aspect ratio, cx, cy, skewness)
std::vector<double> PointSet::GetSbaInitialParams(int cnp)
{
  double altitude = 2000;  //assume aircraft is flying at 2000m
  int m = metadataList.size();
  int n = numMatches;
  int pnp = 3;  //points are Euclidian in 3D space
  // allocate initial vector with space for cnp parameters per camera and 
  // pnp parameters per point
  std::vector<double> p(m*cnp + n*pnp);
  
  int pIndex = 0;
  if (cnp == 11)
  {
    // iterate over cameras to set parameters
    for (auto & image : metadataList)
    {
      CameraParams params;
      int h = image.ImageSize().height;
      int w = image.ImageSize().width;

      // camera instrinsics -
      // f, u0, v0, ar, s
      // To calculate f, get the length of the image horizontal in the world frame
      cv::Point2f wEnd = image.ImageToGeo(cv::Point2f(0, w));
      cv::Point2f wOrigin = image.ImageToGeo(cv::Point2f(0, 0));
      double wWidth = cv::norm(wOrigin - wEnd);
      params.f = w / wWidth * altitude;
      params.u0 = image.ImageSize().width / 2.0;
      params.v0 = image.ImageSize().height / 2.0;
      params.ar = 1.0;
      params.s = 0;

      // Calculate Rotation
      // Since the GeoTIFF images have already been homographically stitched, the image
      // axes are aligned. For simplicity, we set the world coordinate system
      // to have the same alignment. The initial rotation estimate for each camera is then
      // the identity quaternion 1, 0, 0, 0
      params.q0 = 1;
      params.q1 = 0;
      params.q2 = 0;
      params.q3 = 0;

      // Calculate camera translation
      // The Geo coordinate frame is positive in North and East, while the image frame is 
      // positive in South and East. To maintain the identity rotation above, we must invert
      // the North/South and Up/Down axes of the Geo frame to produce the world frame.
      cv::Point2f imCenter = cv::Point2f(params.u0, params.v0);
      cv::Point2f camCenter = image.ImageToGeo(imCenter);
      params.x = - camCenter.x;
      params.y = camCenter.y;
      params.z = altitude;

      // Reduce Quaternion representation to 3D & copy to p
      quat2vec((double*)&params, cnp + 1, &p[pIndex], cnp);

      pIndex += cnp;
    }
  }
  
  if (pIndex != m * cnp)
  {
    std::cerr << "index error in GetSbaInitialParams. Expected: " 
      << m * cnp << " Got: " << pIndex << std::endl;
  }

  // Iterate over points to set initial estimates
  for (int i = 0; i < imageIndices.size(); i++)
  {
    for (auto const & match : points[i])
    {
      // Take the avgerage estimaged Geo location for each point
      cv::Point2f firstLoc = metadataList[imageIndices[i].first].ImageToGeo(match.first);
      cv::Point2f secondLoc = metadataList[imageIndices[i].second].ImageToGeo(match.second);
      cv::Point2f avgLoc = (firstLoc + secondLoc) / 2;

      // Store point in initial estimates array
      // As above, we invert north/south to match the camera frame of reference
      // rotationally. altitude is zero, so no inversion is necessary
      p[pIndex] = avgLoc.x;
      p[pIndex + 1] = -avgLoc.y;
      p[pIndex + 2] = 0;

      pIndex += pnp;
    }
  }

  if (pIndex != m * cnp + n * pnp)
  {
    std::cerr << "Final index error in GetSbaInitialParams. Expected: "
      << m * cnp + n * pnp << " Got: " << pIndex << std::endl;
  }

  return p;
}

int PointSet::Num3DPoints() {
  return numMatches;
}

int PointSet::NumFrames() {
  return metadataList.size();
}

}  // namespace ggck
