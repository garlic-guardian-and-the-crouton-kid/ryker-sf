/*
 * Copyright 2018 Justin Manley and Joseph Bolling.
 */
#include "partition.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Boolean_set_operations_2.h>

namespace ggck {
namespace partition {

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef CGAL::Arr_segment_traits_2<Kernel> Traits_2;
typedef Traits_2::Point_2 Point_2;
typedef Traits_2::X_monotone_curve_2 Segment_2;
typedef CGAL::Arrangement_2<Traits_2> Arrangement_2;
typedef CGAL::Polygon_2<Kernel> Polygon_2;
typedef Arrangement_2::Face Face_2;

// Note: The face must be bounded and should not be fictitious.
// Its outer boundary circulator should be nonempty.
Polygon_2 FaceBoundaryToPolygon(const Face_2& face) {
  std::vector<Point_2> points;

  auto iterator_current_edge = face.outer_ccb();
  do {
      points.push_back(iterator_current_edge->source()->point());
      iterator_current_edge++;
  } while (iterator_current_edge != face.outer_ccb());

  return Polygon_2(points.begin(), points.end());
}

OverlapInfo ComputeOverlaps(std::vector<Polygon_2> images) {
  Arrangement_2 arrangement;
  for (auto image = images.begin(); image != images.end(); image++) {
    CGAL::insert(arrangement, image->edges_begin(), image->edges_end());
  }

  // The bool at the (i, j)-th position in this two-dimensional vector indicates
  // whether the ith face overlaps with the jth image. 
  std::vector<std::vector<bool>> images_per_face;
  std::transform(
          arrangement.faces_begin(),
          arrangement.faces_end(),
          std::back_inserter(images_per_face),
    [images, images_per_face](const auto& face) -> std::vector<bool> {
      if (face.is_fictitious() || face.is_unbounded()) {
        return std::vector<bool>(images.size(), false);
      }
      std::vector<bool> face_intersects_image;
      const Polygon_2 face_polygon = FaceBoundaryToPolygon(face);
      std::transform(
              images.begin(),
              images.end(),
              std::back_inserter(face_intersects_image),
        [face_polygon](const Polygon_2& image) -> bool {
          return CGAL::do_intersect(image, face_polygon);
        });
      return face_intersects_image;
    });

  return OverlapInfo{
    images_per_face = images_per_face
  };
}

}  // namespace partition
}  // namespace ggck
