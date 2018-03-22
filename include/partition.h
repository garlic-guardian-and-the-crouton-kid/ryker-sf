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

// Overlay the images in a shared coordinate system and return a list of
// OverlappingImageSets, each representing a face of the map. NOTE: Because of
// the way the map is partitioned, it is possible for there to be more than one
// OverlappingImageSet representing the same set of images. In such cases, each
// OverlappingImageSet will represent a different region of the same set of
// images.
// TODO(justinmanley): Merge multiple OverlappingImageSets for the same sets of
// images together into a single OverlappingImageSet using CGAL boolean set
// operations on polygons and polygon sets. (Alternatively, we can merge
// the generated masks, rather than the underlying polygons).
std::vector<OverlappingImageSet> ComputeOverlaps(
    const std::vector<ImageMetadata>& images);

}  // namespace ggck

#endif  // INCLUDE_PARTITION_H_
