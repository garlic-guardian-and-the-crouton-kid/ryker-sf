#include "image_metadata.h"

#include <iostream>

#include "gdal.h"
#include "gdal_priv.h"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

namespace ggck {
namespace image_metadata {

using cv::Point2f;
using cv::Point2i;

namespace {

// Transform a GDAL-style affine map into an OpenCV-style affine map.
// affine_map should be an array of 6 doubles.
cv::Mat AffineMapMatrix(double* affine_map) {
	cv::Mat_<double> T = cv::Mat(2, 3, CV_64F);
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			T(i, j) = affine_map[3 * i + j + 1];
		}
	}
	T(0, 2) = affine_map[0];
	T(1, 2) = affine_map[3];
	return T;
}

std::vector<Point2i> GetCorners(int width, int height) {
	// This list of coordinates MUST be a continuous chain of coordinates winding around the edge of the bounding box.
	// CGAL connects points in order to form a polygon, and out-of-order coordinates will produce a bowtie shape rather than a rectangle.
	return std::vector<Point2i>({
		Point2i(0, 0),
		Point2i(0, width),
		Point2i(height, width),
		Point2i(height, 0),
	});
}

cv::Mat Homogenize(const Point2f& point) {
	cv::Mat_<double> homogeneous_point(3, 1, CV_64F);
	homogeneous_point(0, 0) = point.x;
	homogeneous_point(1, 0) = point.y;
	homogeneous_point(2, 0) = 1;
	return homogeneous_point;
}

cv::Point2f MatToPoint(const cv::Mat& M) {
	return cv::Point2f(M.at<double>(0, 0), M.at<double>(1, 0));
}

}  // namespace

ImageMetadata::ImageMetadata(const std::string& image_filename)
	: image_filename(image_filename) {
  GDALDataset* dataset = (GDALDataset *) GDALOpen(image_filename.c_str(), GA_ReadOnly);
	if (dataset == nullptr) {
		throw std::runtime_error("Could not open " + image_filename);
	}
	double geoTransform[6];
	if (dataset->GetGeoTransform(geoTransform) != CE_None) {
		throw std::runtime_error("Expected " + image_filename + " to have an attached GeoTransform.");
	}

	size = cv::Size(dataset->GetRasterXSize(), dataset->GetRasterYSize());

	pixel_corners = GetCorners(size.width, size.height);
	pixels_to_geo_affine_map = AffineMapMatrix(geoTransform);

	std::cout << pixel_corners << std::endl;
}

ConstCornerIterator ImageMetadata::GetPixelCornersBegin() const {
	return pixel_corners.begin();
}

ConstCornerIterator ImageMetadata::GetPixelCornersEnd() const {
	return pixel_corners.end();
}

cv::Point2f ImageMetadata::GeoToPixel(const Point2f& geo_coords) const {
	cv::Mat geo_to_pixels_affine_map = cv::Mat(2, 3, CV_64F);
	cv::invertAffineTransform(pixels_to_geo_affine_map, geo_to_pixels_affine_map);
	return MatToPoint(geo_to_pixels_affine_map *  Homogenize(geo_coords));
}

cv::Point2f ImageMetadata::PixelToGeo(const Point2f& pixel_coords) const {
	return MatToPoint(pixels_to_geo_affine_map * Homogenize(pixel_coords));
}

cv::Size ImageMetadata::GetSize() const {
	return size;
}

std::string ImageMetadata::GetFilename() const {
    return image_filename;
}

}  // namespace image_metadata
}  // namespace ggck
