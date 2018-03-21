#ifndef INCLUDE_GEOMETRY_H_
#define INCLUDE_GEOMETRY_H_

#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>

namespace ggck {

// Geometric types used throughout the project.

typedef ::CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef ::CGAL::Polygon_2<Kernel> Polygon_2;
typedef ::CGAL::Arr_segment_traits_2<Kernel> Traits_2;
typedef Traits_2::X_monotone_curve_2 Segment_2;
typedef Traits_2::Point_2 Point_2;
typedef ::CGAL::Arrangement_2<Traits_2> Arrangement_2;
typedef Arrangement_2::Face Face_2;

}  // ggck

#endif  // INCLUDE_GEOMETRY_H_

