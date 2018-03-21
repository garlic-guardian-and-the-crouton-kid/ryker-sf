/*
 * Copyright 2018 Justin Manley and Joseph Bolling.
 */
#ifndef INCLUDE_PARTITION_H_
#define INCLUDE_PARTITION_H_

#include <vector>

#include "opencv2/core.hpp"

#include "image_metadata.h"
#include "overlapping_image_set.h"

namespace ggck {

std::vector<OverlappingImageSet> ComputeOverlaps(
    const std::vector<ImageMetadata>& images);

}  // namespace ggck

#endif  // INCLUDE_PARTITION_H_
