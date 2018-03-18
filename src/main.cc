/*
 * Copyright 2018 Justin Manley and Joseph Bolling.
 */
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Arr_segment_traits_2.h>

#include "gdal.h"
#include "gdal_priv.h"

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include "partition.h"
#include "image_metadata.h"

using ggck::partition::OverlapInfo;
using ggck::partition::ComputeOverlaps;
using ggck::image_metadata::ImageMetadata;

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef CGAL::Arr_segment_traits_2<Kernel> Traits_2;
typedef CGAL::Polygon_2<Kernel> Polygon_2;
typedef Traits_2::Point_2 Point_2;
typedef CGAL::Arrangement_2<Traits_2> Arrangement_2;
typedef Arrangement_2::Face Face_2;

const std::vector<std::string> image_paths = {
	"/home/justin/p/rykers-sf/images/512w/001.tif",
	"/home/justin/p/rykers-sf/images/512w/002.tif",
	"/home/justin/p/rykers-sf/images/512w/003.tif",
	"/home/justin/p/rykers-sf/images/512w/004.tif",
	"/home/justin/p/rykers-sf/images/512w/005.tif",
	"/home/justin/p/rykers-sf/images/512w/006.tif",
};

struct CvPolygon {
	std::vector<cv::Point> points;
	int* npoints;
	int ncontours;
};

CvPolygon CgalToCvPolygon(const ImageMetadata& image_metadata, const Face_2& face) {
  std::vector<cv::Point> points;

  auto iterator_current_edge = face.outer_ccb();
  do {
			Point_2 point = iterator_current_edge->source()->point();
			// NOTE that we flip the x and y-coordinates here to accommodate the
			// change in nomenclature from width/height (CGAL) to rows/cols (OpenCV).
			// The x-dimension (width) corresponds to columns, and the y-dimension
			// (height) to rows.
			cv::Point2f p = cv::Point2f(CGAL::to_double(point.y()), CGAL::to_double(point.x()));

			std::cout << image_metadata.GeoToPixel(p) << std::endl;

      points.push_back(image_metadata.GeoToPixel(p));
      iterator_current_edge++;
  } while (iterator_current_edge != face.outer_ccb());

	int npoints[1] = { (int) points.size() };

	return CvPolygon {
    points: points,
		npoints: npoints,
		ncontours: 1,
	};
}

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

	std::vector<std::vector<cv::Mat>> masks_per_face;
	for (int i = 0; i < images_per_face.size(); i++) {
		std::vector<cv::Mat> masks;
		for (int j = 0; j < images_per_face[i].size(); j++) {
			bool image_in_face = images_per_face[i][j];	
			if (image_in_face) {
				cv::Size image_size = image_metadata[j].GetSize();
				cv::Mat mask = cv::Mat::zeros(image_size.height, image_size.width, CV_8U);
				
				auto faces = std::vector<Face_2>(
						overlap.arrangement.faces_begin(),
						overlap.arrangement.faces_end());

				CvPolygon polygon = CgalToCvPolygon(image_metadata[j], faces[i]);

				// TODO: Why is dynamic allocation necessary here?
				const cv::Point** points = new const cv::Point*[polygon.points.size()];
				for (int k = 0; k < polygon.points.size(); k++) {
					points[k] = &polygon.points[k];
				}

				cv::Scalar color = cv::Scalar(255);
				cv::fillPoly(mask, points, polygon.npoints, polygon.ncontours, color);

				cv::imshow("Mask demo", mask);
				cv::waitKey(0);

				delete[] points;

				masks.push_back(mask);
			}
		}
		masks_per_face.push_back(masks);
	}
	
  return 0;
}
