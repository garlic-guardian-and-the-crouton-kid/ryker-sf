/*
 * Copyright 2018 Justin Manley and Joseph Bolling.
 */
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Arr_segment_traits_2.h>

#include "gdal.h"
#include "gdal_priv.h"

#include "partition.h"
#include "image_metadata.h"

using ggck::partition::OverlapInfo;
using ggck::partition::ComputeOverlaps;
using ggck::image_metadata::ImageMetadata;

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef CGAL::Arr_segment_traits_2<Kernel> Traits_2;
typedef CGAL::Polygon_2<Kernel> Polygon_2;
typedef Traits_2::Point_2 Point_2;

Polygon_2 Polygon(const std::vector<Point_2>& vertices) {
  return Polygon_2(vertices.begin(), vertices.end());
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

const std::vector<std::string> image_paths = {
	"/home/justin/Downloads/test.tif",
};

int main() {
  GDALAllRegister();

	std::vector<ImageMetadata> image_metadata;
	std::transform(image_paths.begin(), image_paths.end(), std::back_inserter(image_metadata),
			[](const std::string& image_path) {
				ImageMetadata metadata(image_path);
				return metadata;
			});

  OverlapInfo overlap = ComputeOverlaps(image_metadata);
	std::vector<std::vector<bool>> images_per_face = overlap.images_per_face;

  for (int i = 0; i < images_per_face.size(); i++) {
    std::cout << "Images contained in face " << i << ": ";
    std::vector<bool> faces_bitset = images_per_face[i];
    std::copy(
        faces_bitset.begin(),
        faces_bitset.end(),
        std::ostream_iterator<bool>(std::cout, ", "));
    std::cout << std::endl;
  }

//    std::map<int, std::vector<Mask>> masks = ComputeMasks(overlap); 

//   std::cout << "Computed masks for " << masks.size() << " patches of map." << std::endl;

  return 0;
}
