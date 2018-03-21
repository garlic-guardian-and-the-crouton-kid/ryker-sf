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

class PointSet {
public:
  // Stores matches and metadata
  void Add(const PointMatches& matches, const ImagePair& metadata);

  // Returns a double vector with proper ordering for use with SBA 
  std::vector<double> GetSbaMeasurements();

private:
  // Arrays of point correspondences. ptsA[i][j] refers to the ith point from image j
  std::vector<PointMatches> points;

  // Metadata corresponding to the 
  std::vector<ImagePair> imagePairs;
};

}  // namespace ggck

#endif  // INCLUDE_POINT_SET_H_