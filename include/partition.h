/*
 * Copyright 2018 Justin Manley and Joseph Bolling.
 */
#ifndef INCLUDE_PARTITION_H_
#define INCLUDE_PARTITION_H_

#include "opencv2/core.hpp"

#include "image_metadata.h"
#include "overlapping_image_set.h"

namespace ggck {
namespace partition {

std::vector<overlapping_image_set::OverlappingImageSet> ComputeOverlaps(
		const std::vector<image_metadata::ImageMetadata>& images);

}  // namespace partition
}  // namespace ggck

#endif  // INCLUDE_PARTITION_H_
