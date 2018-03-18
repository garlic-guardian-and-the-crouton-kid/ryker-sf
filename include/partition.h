/*
 * Copyright 2018 Justin Manley and Joseph Bolling.
 */
#ifndef INCLUDE_PARTITION_H_
#define INCLUDE_PARTITION_H_

#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>

#include "opencv2/core.hpp"

#include "image_metadata.h"

namespace ggck {
namespace partition {

typedef ::CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef ::CGAL::Polygon_2<Kernel> Polygon_2;
typedef ::CGAL::Arr_segment_traits_2<Kernel> Traits_2;
typedef ::CGAL::Arrangement_2<Traits_2> Arrangement_2;

struct OverlapInfo {
	std::vector<std::vector<bool>> images_per_face;
	Arrangement_2 arrangement;
};

OverlapInfo ComputeOverlaps(const std::vector<image_metadata::ImageMetadata>& images);

}  // namespace partition
}  // namespace ggck

#endif  // INCLUDE_PARTITION_H_
