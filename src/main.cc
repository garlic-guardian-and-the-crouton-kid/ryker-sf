/*
 * Copyright 2018 Justin Manley and Joseph Bolling.
 */
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Arr_segment_traits_2.h>

#include "gdal.h"
#include "gdal_priv.h"

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"

#include "partition.h"
#include "image_metadata.h"
#include "masked_image.h"
#include "point_match.h"

using ggck::partition::OverlapInfo;
using ggck::partition::ComputeOverlaps;
using ggck::image_metadata::ImageMetadata;
using ggck::masked_image::MaskedImage;
using ggck::point_match::GetPoints;

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef CGAL::Arr_segment_traits_2<Kernel> Traits_2;
typedef CGAL::Polygon_2<Kernel> Polygon_2;
typedef Traits_2::Point_2 Point_2;
typedef CGAL::Arrangement_2<Traits_2> Arrangement_2;
typedef Arrangement_2::Face Face_2;

const std::vector<std::string> image_paths = {
	"/home/justin/p/rykers-sf/images/512w/001.tif",
	"/home/justin/p/rykers-sf/images/512w/002.tif",
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
			cv::Point2f p = cv::Point2f(CGAL::to_double(point.x()), CGAL::to_double(point.y()));
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

	auto faces = std::vector<Face_2>(
			overlap.arrangement.faces_begin(),
			overlap.arrangement.faces_end());

  for (int i = 0; i < images_per_face.size(); i++) {
    std::cout << "Images contained in face " << i << ": ";
    std::vector<bool> faces_bitset = images_per_face[i];
    std::copy(
        faces_bitset.begin(),
        faces_bitset.end(),
        std::ostream_iterator<bool>(std::cout, ", "));
    std::cout << std::endl;
  }

	std::vector<std::vector<MaskedImage>> masks_per_face;
	for (int i = 0; i < images_per_face.size(); i++) {
		std::vector<MaskedImage> masked_images;

		int overlaps = std::count(images_per_face[i].begin(), images_per_face[i].end(), true);

		if (overlaps > 1) {
			for (int j = 0; j < images_per_face[i].size(); j++) {
				bool image_in_face = images_per_face[i][j];	
				if (image_in_face) {
					cv::Size image_size = image_metadata[j].GetSize();
					// TODO: We may want to swap the height and width of this matrix, depending
					// on how OpenCV loads images for matching, etc. If we swap the height and width
					// of the matrix, then we also need to swap the coordinates of the points in the
					// mask polygons.
					cv::Mat mask = cv::Mat::zeros(image_size.width, image_size.height, CV_8U);

					CvPolygon polygon = CgalToCvPolygon(image_metadata[j], faces[i]);

					// TODO: Why is dynamic allocation necessary here?
					const cv::Point** points = new const cv::Point*[polygon.points.size()];
					for (int k = 0; k < polygon.points.size(); k++) {
						points[k] = &polygon.points[k];
					}

					cv::Scalar color = cv::Scalar(255);
					cv::fillPoly(mask, points, polygon.npoints, polygon.ncontours, color);

					cv::imshow("Mask demo for face " + std::to_string(i), mask);
					cv::waitKey(0);

					cv::imshow("Image", cv::imread(image_metadata[j].GetFilename(), cv::IMREAD_LOAD_GDAL));
					cv::waitKey(0);

					delete[] points;

					MaskedImage masked_image = MaskedImage {
						image_metadata[j],
						mask,
						// TODO: Replace with real image.
						cv::Mat(),
					};

					masked_images.push_back(masked_image);
				}
			}
		}

		masks_per_face.push_back(masked_images);
	}

	for (auto images_in_face = masks_per_face.begin(); images_in_face != masks_per_face.end(); images_in_face++) {
		for (auto im1 = images_in_face->begin(); im1 != images_in_face->end(); im1++) {
			for (auto im2 = images_in_face->begin(); im2 != images_in_face->end(); im2++) {
				if (im1->metadata.GetFilename() != im2->metadata.GetFilename()) {
					im1->image = cv::imread(im1->metadata.GetFilename(), CV_LOAD_IMAGE_GRAYSCALE);
					im2->image = cv::imread(im2->metadata.GetFilename(), CV_LOAD_IMAGE_GRAYSCALE);

					auto matches = GetPoints(*im1, *im2);
					cv::imshow("Matches", matches);
					cv::waitKey(0);
				}
			}
		}
	}

	for (auto images_in_face = masks_per_face.begin(); images_in_face != masks_per_face.end(); images_in_face++) {
		for (auto im1 = images_in_face->begin(); im1 != images_in_face->end(); im1++) {
			for (auto im2 = images_in_face->begin(); im2 != images_in_face->end(); im2++) {
				if (im1->metadata.GetFilename() != im2->metadata.GetFilename()) {
					im1->image = cv::imread(im1->metadata.GetFilename(), CV_LOAD_IMAGE_GRAYSCALE);
					im2->image = cv::imread(im2->metadata.GetFilename(), CV_LOAD_IMAGE_GRAYSCALE);

					cv::imshow("Masked image", im1->image & im1->mask);
					cv::waitKey(0);

					cv::imshow("Masked image", im2->image & im2->mask);
					cv::waitKey(0);
				}
			}
		}
	}
	
  return 0;
}
