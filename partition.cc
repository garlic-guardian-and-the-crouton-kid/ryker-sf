#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Boolean_set_operations_2.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef CGAL::Arr_segment_traits_2<Kernel> Traits_2;
typedef Traits_2::Point_2 Point_2;
typedef Traits_2::X_monotone_curve_2 Segment_2;
typedef CGAL::Arrangement_2<Traits_2> Arrangement_2;
typedef CGAL::Polygon_2<Kernel> Polygon_2;
typedef Arrangement_2::Face Face_2;

Polygon_2 Polygon(const std::vector<Point_2>& vertices) {
  return Polygon_2(vertices.begin(), vertices.end());
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

const std::vector<Polygon_2> images = {
    Polygon({}),
    Polygon({}),
    Polygon({}),
    Polygon({}),
    Polygon({}),
    Polygon({}),
    Polygon({}),
    Polygon({
        Point_2(-13624919, 4540337),
        Point_2(-13624919, 4538763),
        Point_2(-13623134, 4540337),
        Point_2(-13623134, 4538763),
    }),
    Polygon({
        Point_2(-13624949, 4542609),
        Point_2(-13624949, 4541184),
        Point_2(-13623263, 4542609),
        Point_2(-13623263, 4541184),
    }),
    Polygon({
        Point_2(-13624949, 4542609),
        Point_2(-13624949, 4541184),
        Point_2(-13623263, 4542609),
        Point_2(-13623263, 4541184),
    }),
};

int main() {
  Arrangement_2 arrangement;
  for (auto image = images.begin(); image != images.end(); image++) {
    CGAL::insert(arrangement, image->edges_begin(), image->edges_end());
  }

  // The bool at the (i, j)-th position in this two-dimensional vector indicates
  // whether the ith face overlaps with the jth image.
  std::vector<std::vector<bool>> images_per_face;
  std::transform(arrangement.faces_begin(), arrangement.faces_end(), std::back_inserter(images_per_face),
    [images_per_face](const auto& face) -> std::vector<bool> {
      if (face.is_fictitious() || face.is_unbounded()) {
        return std::vector<bool>(images.size(), false);
      }
      std::vector<bool> face_intersects_image;
			const Polygon_2 face_polygon = FaceBoundaryToPolygon(face);
			print_polygon(face_polygon);
      std::transform(images.begin(), images.end(), std::back_inserter(face_intersects_image),
        [face_polygon](const Polygon_2& image) -> bool {
          return CGAL::do_intersect(image, face_polygon);
        });
      return face_intersects_image;
    });

	for (int i = 0; i < images_per_face.size(); i++) {
		std::cout << "Images contained in face " << i << ": ";
		std::vector<bool> faces_bitset = images_per_face[i];
		std::copy(faces_bitset.begin(), faces_bitset.end(), std::ostream_iterator<bool>(std::cout, ", "));
		std::cout << std::endl;
	}

  return 0;
}
