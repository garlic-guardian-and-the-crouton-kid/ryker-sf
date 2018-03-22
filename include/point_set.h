/*
 * Copyright 2018 Justin Manley and Joseph Bolling.
 */
#ifndef INCLUDE_POINT_SET_H_
#define INCLUDE_POINT_SET_H_

#include "point_match.h"

#include <utility>
#include <vector>

#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

#include "overlapping_image_set.h"

namespace ggck {

// A vector of matched points
typedef std::vector<std::pair<cv::Point2d, cv::Point2d>> PointMatches;

struct SbaMeasurementInfo {
  // Double vector with proper ordering for use with SBA
  std::vector<double> measurements;

  // a char* vector with the indicator mask of which points appear in which
  // images for use with SBA
  // visibility mask: vmask[i, j]=1 if point i visible in image j, 0 otherwise.
  // nxm
  std::vector<char> vmask;
};

// Converts from an array of dense points adn matches to a PointMatches
// structure
PointMatches DensePointsAndMatchesToPointMatches(
    DensePointsAndMatches dpMatches);

class PointSet {
 public:
  // Constructor accepts a list of all metadata
  explicit PointSet(const std::vector<ImageMetadata>& metadataList);

  // Stores matches and metadata
  void Add(PointMatches matches, ImagePair metadata);
  void Add(DensePointsAndMatches dpMatches, ImagePair metadata);

  // Calculates and returns the measurement vector and mask for use with SBA
  SbaMeasurementInfo GetSbaMeasurementInfo();

  // Returns an initial parameter estimate vector for use with SBA
  std::vector<double> GetSbaInitialParams(int cnp);

  // returns a 3D point cloud vector
  std::vector<cv::Point3d> GetPointCloud();

  // Returns the total number of 3D points, or the total number of matches
  int Num3DPoints();

  // Returns the total number of images
  int NumFrames();

 private:
  // Arrays of point correspondences. ptsA[i][j] refers to the ith match from
  // image correspondence j
  std::vector<PointMatches> points;

  // Number of unique 3D points in pointss
  int numMatches;

  // Ordered vector of image metadata. Used to assign numerical indices to
  // images
  const std::vector<ImageMetadata> metadataList;

  // Array of image index pairs. Can be used to index into metadata list to
  // retrieve metadata
  // for a particular pair in points
  std::vector<std::pair<int, int>> imageIndices;
};

}  // namespace ggck

#endif  // INCLUDE_POINT_SET_H_
