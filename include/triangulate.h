/*
* Copyright 2018 Justin Manley and Joseph Bolling.
*/
#ifndef INCLUDE_TRIANGULATE_H_
#define INCLUDE_TRIANGULATE_H_

#include <vector>

#include <opencv2/opencv.hpp>

#include "point_set.h"

namespace ggck {

// Wrapper for SBA library's sba_motstr_levmar function. Runs Bundle adjustment
// using the measurements and initial estimates calculated by PointSet
std::vector<cv::Point3d> RunSba(PointSet* pointSet);

}  // namespace ggck

#endif  // INCLUDE_TRIANGULATE_H_
