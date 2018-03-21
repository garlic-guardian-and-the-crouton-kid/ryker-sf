/*
 * Copyright 2018 Justin Manley and Joseph Bolling.
 */
#include "point_set.h"

#include <iostream>
#include <memory>

#include <opencv2/features2d.hpp>

namespace ggck {

PointSet::PointSet(const std::vector<ImageMetadata>& metadataList):
  metadataList(metadataList)
{
  numMatches = 0;
}

void PointSet::Add(PointMatches matches, ImagePair metadata)
{
  std::pair<int, int> indices;

  //determine which index to assign to metadata
  for (int i = 0; i < metadataList.size(); i++)
  {
    if (metadata.first.Filename() == metadataList[i].Filename())
    {
      indices.first = i;
    }
    else if (metadata.second.Filename() == metadataList[i].Filename())
    {
      indices.second = i;
    }
  }

  // store matches and calculated indices
  points.push_back(matches);
  imageIndices.push_back(indices);
  numMatches += static_cast<int>(matches.size());
}

/* 
 * From the SBA library:
 * measurements vector: (x_11^T, .. x_1m^T, ..., x_n1^T, .. x_nm^T)^T where
 * x_ij is the projection of the i-th point on the j-th image.
 * NOTE: some of the x_ij might be missing, if point i is not visible in image j;
 * see vmask[i, j], max. size n*m*mnp
 */
SbaMeasurementInfo PointSet::GetSbaMeasurementInfo() 
{
  int numFrames = metadataList.size();

  // initialize mask with zeros
  std::vector<char> mask(numMatches * metadataList.size()); 
  std::fill(mask.begin(), mask.end(), 0);

  // initialize measurement vector - each 3D point is observed exactly twice
  // and has two parameters
  std::vector<double> measurements(numMatches * 2 * 2);

  //iterate over all points
  int matchIndex = 0;
  for (int i = 0; i < imageIndices.size(); i++)
  {
    if (imageIndices[i].first < imageIndices[i].second)
    {
      for (auto const & match : points[i])
      {
        // Store 2d point data in measurements array, ordered by index
        measurements[matchIndex * 4] = match.first.x;
        measurements[matchIndex * 4 + 1] = match.first.y;
        measurements[matchIndex * 4 + 2] = match.second.x;
        measurements[matchIndex * 4 + 3] = match.second.y;

        // mark that point is visible in these images in mask array
        mask[matchIndex*numFrames + imageIndices[i].first] = 1;
        mask[matchIndex*numFrames + imageIndices[i].second] = 1;

        matchIndex++;
      }
    }
    else if (imageIndices[i].second < imageIndices[i].first)
    {
      for (auto const & match : points[i])
      {
        // Store 2d point data in measurements array, ordered by index
        measurements[matchIndex * 4] = match.second.x;
        measurements[matchIndex * 4 + 1] = match.second.y;
        measurements[matchIndex * 4 + 2] = match.first.x;
        measurements[matchIndex * 4 + 3] = match.first.y;

        // mark that point is visible in these images in mask array
        mask[matchIndex*numFrames + imageIndices[i].first] = 1;
        mask[matchIndex*numFrames + imageIndices[i].second] = 1;

        matchIndex++;
      }
    }
  }
  return SbaMeasurementInfo{ measurements, mask };
}

}  // namespace ggck