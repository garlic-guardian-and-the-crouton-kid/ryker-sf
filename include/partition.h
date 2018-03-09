#ifndef GGCK_PARTITION_H
#define GGCK_PARTITION_H

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>

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

#endif /* GGCK_PARTITION_H */
