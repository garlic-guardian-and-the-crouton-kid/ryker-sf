/*
 * Copyright 2018 Justin Manley and Joseph Bolling.
 */
#include "point_set.h"

#include <iostream>
#include <memory>

#include <opencv2/features2d.hpp>

namespace ggck {

void PointSet::Add(const PointMatches& matches, const ImagePair& metadata)
{
  points.push_back(matches);
  imagePairs.push_back(metadata);
}


}  // namespace ggck