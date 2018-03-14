/*
 * Copyright 2018 Justin Manley and Joseph Bolling.
 */
#include "partition.h"

#include <math.h>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Boolean_set_operations_2.h>

namespace ggck {
namespace partition {

using cv::Point2f;
using image_metadata::ImageMetadata;

typedef ::CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef ::CGAL::Arr_segment_traits_2<Kernel> Traits_2;
typedef Traits_2::Point_2 Point_2;
typedef Traits_2::X_monotone_curve_2 Segment_2;
typedef ::CGAL::Arrangement_2<Traits_2> Arrangement_2;
typedef ::CGAL::Polygon_2<Kernel> Polygon_2;
typedef Arrangement_2::Face Face_2;

Point_2 RoundToPoint(const Point2f& point) {
	return Point_2((int) rint(point.x), (int) rint(point.y));
}

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

Polygon_2 GetGeoPolygon(const ImageMetadata& image_metadata) {
	std::vector<Point_2> vertices;
	std::transform(image_metadata.GetPixelCornersBegin(), image_metadata.GetPixelCornersEnd(), std::back_inserter(vertices),
			[image_metadata](const cv::Point2f& corner) {
				return RoundToPoint(image_metadata.PixelToGeo(corner));
			});
  return Polygon_2(vertices.begin(), vertices.end());
}

OverlapInfo ComputeOverlaps(const std::vector<image_metadata::ImageMetadata>& image_metadata) {
	std::vector<Polygon_2> images;
	std::transform(image_metadata.begin(), image_metadata.end(), std::back_inserter(images),
			[](const ImageMetadata& image_metadata) {
				return GetGeoPolygon(image_metadata);
			});

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

  return OverlapInfo {
		images_per_face: images_per_face,
  };
}

}  // namespace partition
}  // namespace ggck
