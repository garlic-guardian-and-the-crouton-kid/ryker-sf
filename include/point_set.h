/*
 * Copyright 2018 Justin Manley and Joseph Bolling.
 */
#ifndef INCLUDE_POINT_SET_H_
#define INCLUDE_POINT_SET_H_

#include "overlapping_image_set.h"

#include <vector>

#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

namespace ggck {

// A pair of point arrays, where ptsA[x] is matched with ptsB[x]
typedef std::pair<std::vector<cv::Point2d>, std::vector<cv::Point2d>> PointMatches;

struct SbaMeasurementInfo {
  // Double vector with proper ordering for use with SBA 
  std::vector<double> measurements;
  // a char* vector with the indicator mask of which points appear in which images for use with SBA
  // visibility mask: vmask[i, j]=1 if point i visible in image j, 0 otherwise. nxm
  std::vector<char> vmask;
};

class PointSet {
public:
  // Constructor accepts a list of all metadata 
  PointSet(const std::vector<ImageMetadata>& metadataList);

  // Stores matches and metadata
  void Add(const PointMatches& matches, const ImagePair& metadata);

  // Calculates and returns the measurement vector and mask for use with SBA
  SbaMeasurementInfo GetSbaMeasurementInfo();

private:
  // Arrays of point correspondences. ptsA[i][j] refers to the ith point from image j
  std::vector<PointMatches> points;

  // Number of unique points in points
  int numPoints;

  // Ordered vector of image metadata. Used to assign numerical indices to images
  std::vector<ImageMetadata> metadataList;

  // Array of image index pairs. Can be used to index into metadata list to retrieve metadata
  // for a particular pair in points
  std::vector<std::pair<int, int>> imageIndices;
};

}  // namespace ggck

#endif  // INCLUDE_POINT_SET_H_