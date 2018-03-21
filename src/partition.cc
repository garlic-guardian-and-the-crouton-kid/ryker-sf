/*
 * Copyright 2018 Justin Manley and Joseph Bolling.
 */
#include "partition.h"

#include <CGAL/Boolean_set_operations_2.h>

#include <math.h>
#include <iostream>

#include "geometry.h"

namespace ggck {

using cv::Point2f;

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

Point_2 RoundToPoint(const Point2f& point) {
  return Point_2(static_cast<int>(rint(point.x)),
                 static_cast<int>(rint(point.y)));
}

Polygon_2 GeoPolygon(const ImageMetadata& image_metadata) {
  std::vector<Point_2> vertices;
  std::transform(image_metadata.WarpedImageCornersBegin(),
                 image_metadata.WarpedImageCornersEnd(),
                 std::back_inserter(vertices),
                 [image_metadata](const cv::Point2f& corner) {
                   return RoundToPoint(image_metadata.ImageToGeo(corner));
                 });
  return Polygon_2(vertices.begin(), vertices.end());
}

std::vector<OverlappingImageSet> ComputeOverlaps(
    const std::vector<ImageMetadata>& images) {
  // WARNING! When this arrangement goes out of scope, the base objects
  // associated with it (e.g. the Point_2 objects representing the vertices)
  // will be destroyed, and any other objects (e.g. instances of Face_2) which
  // have been copied elsewhere will be left with dangling pointers to these
  // destroyed objects. This is because CGAL keeps a single copy of all Point_2
  // objects, which are accessed via a Handle, so when a Face_2 is copied, the
  // underlying points are not copied. An alternative way to handle this issue
  // would be to use one of the simple geometry kernels which copies points
  // naively.
  Arrangement_2 arrangement;
  for (auto image = images.begin(); image != images.end(); image++) {
    Polygon_2 image_polygon = GeoPolygon(*image);
    CGAL::insert(arrangement, image_polygon.edges_begin(),
                 image_polygon.edges_end());
  }

  std::vector<OverlappingImageSet> overlaps;
  for (auto face = arrangement.faces_begin(); face != arrangement.faces_end();
       face++) {
    if (face->is_fictitious() || face->is_unbounded()) {
      continue;
    }

    // TODO(justinmanley): Represent the face as a general polygon rather than
    // the simple polygon that is returned from FaceBoundaryToPolygon.
    const Polygon_2 face_polygon = FaceBoundaryToPolygon(*face);

    std::vector<ImageMetadata> overlapping_images;
    for (auto image = images.begin(); image != images.end(); image++) {
      // TODO(justinmanley): Avoid calling GeoPolygon more than once.
      const Polygon_2 image_polygon = GeoPolygon(*image);
      if (CGAL::do_intersect(image_polygon, face_polygon)) {
        overlapping_images.push_back(*image);
      }
    }

    // We only care about faces which have more than one image, since we need at
    // least two images to find point correspondences.
    if (overlapping_images.size() > 1) {
      OverlappingImageSet image_set =
          OverlappingImageSet(overlapping_images, face_polygon);
      overlaps.push_back(image_set);
    }
  }

  return overlaps;
}

}  // namespace ggck
