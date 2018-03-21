/*
 * Copyright 2018 Justin Manley and Joseph Bolling.
 */
#ifndef INCLUDE_POINT_MATCH_H_
#define INCLUDE_POINT_MATCH_H_

#include <vector>

#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

#include "overlapping_image_set.h"

namespace ggck {

// Returns a matrix of points corresponded between im1 and im2
cv::Mat PointMatches(const MaskedImage& im1, const MaskedImage& im2);

}  // namespace ggck

#endif  // INCLUDE_POINT_MATCH_H_