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
  numPoints = 0;
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
  numPoints += static_cast<int>(matches.first.size());
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
  // initialize mask with zeros
  std::vector<char> mask(numPoints * metadataList.size()); 
  std::fill(mask.begin(), mask.end(), 0);

  std::vector<double> measurements(numPoints);

  //iterate over all points
  for (int i = 0; i < imageIndices.size(); i++)
  {
    if (imageIndices[i].first < imageIndices[i].second)
    {

    }
  }
  return SbaMeasurementInfo{ measurements, mask };
}

}  // namespace ggck