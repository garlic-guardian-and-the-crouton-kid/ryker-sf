/*
* Copyright 2018 Justin Manley and Joseph Bolling.
*/
#ifndef INCLUDE_POINT_MATCH_H_
#define INCLUDE_POINT_MATCH_H_

#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "overlapping_image_set.h"

namespace ggck {
namespace point_match {

typedef cv::Mat Mat;

/*
* Returns a matrix of points corresponded between im1 and im2
*/
Mat PointMatches(const overlapping_image_set::MaskedImage& im1, const overlapping_image_set::MaskedImage& im2);

} //namespace point_match
} //namespace ggck

#endif //INCLUDE_POINT_MATCH_H_
