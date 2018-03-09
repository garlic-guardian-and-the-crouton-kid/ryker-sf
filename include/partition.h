/*
 * Copyright 2018 Justin Manley and Joseph Bolling.
 */
#ifndef INCLUDE_PARTITION_H_
#define INCLUDE_PARTITION_H_

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <vector>

namespace ggck {
namespace partition {

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef CGAL::Polygon_2<Kernel> Polygon_2;

template<typename T>
using Matrix = std::vector<std::vector<T>>;

struct OverlapInfo {
  Matrix<bool> images_per_face;
};

OverlapInfo ComputeOverlaps(std::vector<Polygon_2> images);

}  // namespace partition
}  // namespace ggck

#endif  // INCLUDE_PARTITION_H_
