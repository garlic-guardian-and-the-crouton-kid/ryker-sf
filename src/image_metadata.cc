#include "image_metadata.h"

#include <iostream>

#include "gdal.h"
#include "gdal_priv.h"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/video.hpp"

namespace cv {

// Define < so that Point can be used as a key in std::map.
bool operator<(const cv::Point& a, const cv::Point& b) {
  return (a.x < b.x) || (a.x == b.x && a.y < b.y);
} 

}

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

// Returns a list of coordinates representing the corners of the bounding box of the entire GeoTIFF.
std::vector<Point2i> ImageCorners(cv::Size size) {
	// This list of coordinates MUST be a continuous chain of coordinates winding around the edge of the bounding box.
	// CGAL connects points in order to form a polygon, and out-of-order coordinates will produce a bowtie shape rather than a rectangle.
	return std::vector<Point2i>({
		Point2i(0, 0),
		Point2i(0, size.width),
		Point2i(size.height, size.width),
		Point2i(size.height, 0),
	});
}

cv::Mat Homogenize(const Point2f& point) {
	cv::Mat_<double> homogeneous_point(3, 1, CV_64F);
	homogeneous_point(0, 0) = point.x;
	homogeneous_point(1, 0) = point.y;
	homogeneous_point(2, 0) = 1;
	return homogeneous_point;
}

Point2f MatToPoint(const cv::Mat& M) {
	return Point2f(M.at<double>(0, 0), M.at<double>(1, 0));
}

// Temporary data structure used to track the smallest observed distance
// between two points during corner matching.
struct MatchCandidate {
	Point2i point;
	double distance;
};

// Given two vectors of points representing the corners of polygons (in no
// particular order), rearrange the vector so that corresponding corners
// align across both vectors. That is, given input vectors containing A,B,C,D
// and a,b,c,d, respectively (in no particular order), return the output vectors
// {A,B,C,D} and {a,b,c,d} (A aligns with a; B aligns with b, etc).
//
//   A ------------------ B 
//     |        ~\ b    |
//     | a   ~    \     |
//     |  ~        \    |
//     |  \         \   |
//     |   \        ~\  |
//     |    \    ~  c   |
//     |   d \~         |
//   D ------------------ C
std::pair<std::vector<Point2i>, std::vector<Point2i>> MatchClosestPoints(
		std::vector<Point2i> points1, std::vector<Point2i> points2) {
	std::map<Point2i, MatchCandidate> matching_points;

	for (auto p1 = points1.begin(); p1 != points1.end(); p1++) {
		for (auto p2 = points2.begin(); p2 != points2.end(); p2++) {
			double distance = cv::norm(*p1 - *p2);
			auto match = matching_points.find(*p1);
			if (match == matching_points.end() || distance < match->second.distance) {
				// p1 does not have a match, or p1 has a match, but it is farther away from p2.
				matching_points[*p1] = MatchCandidate{*p2, distance};
			}
		}
	}

	// TODO: Add a check to confirm that each point in points1 is matched with a unique point in points2.

	// These vectors will contain the matched points in the same order; that is,
	// matched_points1[i] will contain the point matched_points2[i].
	std::vector<Point2i> matched_points1;
	std::vector<Point2i> matched_points2;
	for (auto p = matching_points.begin(); p != matching_points.end(); p++) {
		matched_points1.push_back(p->first);
		matched_points2.push_back(p->second.point);
	}

	return std::make_pair(matched_points1, matched_points2);
};

// Returns a list of coordinates representing the corners of the warped image inside the GeoTIFF.
std::vector<Point2i> WarpedCorners(const std::string& filename) {
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;

	// The GeoTIFF files contain a warped (orthorectified) image set on a transparent background.
	// Here we identify the pixels that form the edge of the warped image in the GeoTIFF.
	cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
	cv::findContours(image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	if (contours.empty()) {
		throw std::runtime_error("No bounding rectangle detected in image " + filename);
	}

	// The findContours function returns a list containing each pixel belonging to the edge of
	// the warped image, in sequence (a chain of edge points). The approxPolyDp function trims
	// this list of points to contain only the four corners.
	// 
	// Epsilon is the amount that the approximation of the edge contour is allowed to deviate
	// from the "actual" contour calculated by findContours. The edges are typically sharp and
	// the contours high quality, so 1 or 2 work well to extract the 4 corners. Anything less
	// than 1, and the approximate contour starts to include points on the sides. In principle,
	// we should be able to increase epsilon dramatically, as long as it remains less than the
	// length of the shortest side of the polygon bounding the warped image.
	std::vector<cv::Point> warped_corners;
	cv::approxPolyDP(contours[0], warped_corners, /* epsilon= */ 2, /* closed= */ true);

	return warped_corners;
}

// Returns the affine transformation which maps the corners of the GeoTIFF onto the corners of the warped image inside the GeoTIFF.
cv::Mat ImageWarpAffineMap(
		const std::vector<Point2i> image_corners,
		const std::vector<Point2i> warped_corners) {
	std::pair<std::vector<Point2i>, std::vector<Point2i>> matched_corners =
		MatchClosestPoints(image_corners, warped_corners);

	// The corners bounding the GeoTIFF (i.e. (0,0), (0, height), etc).
	std::vector<Point2f> matched_image_corners;
	cv::Mat(matched_corners.first).copyTo(matched_image_corners);

	// The corners bounding the warped image inside the GeoTIFF.
	std::vector<Point2f> matched_warped_corners;
	cv::Mat(matched_corners.second).copyTo(matched_warped_corners);

	return estimateRigidTransform(
		matched_image_corners, matched_warped_corners, /* fullAffine= */ true);
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
	warped_image_corners = WarpedCorners(image_filename);
	pixels_to_geo_affine_map = AffineMapMatrix(geoTransform);
	image_warp_affine_map = ImageWarpAffineMap(ImageCorners(size), warped_image_corners);
}

ConstCornerIterator ImageMetadata::WarpedImageCornersBegin() const {
	return warped_image_corners.begin();
}

ConstCornerIterator ImageMetadata::WarpedImageCornersEnd() const {
	return warped_image_corners.end();
}

Point2f ImageMetadata::GeoToWarpedImage(const Point2f& geo_coords) const {
	cv::Mat geo_to_pixels_affine_map = cv::Mat(2, 3, CV_64F);
	cv::invertAffineTransform(pixels_to_geo_affine_map, geo_to_pixels_affine_map);
	return MatToPoint(image_warp_affine_map * Homogenize(MatToPoint(geo_to_pixels_affine_map *  Homogenize(geo_coords))));
}

Point2f ImageMetadata::WarpedImageToGeo(const Point2f& pixel_coords) const {
	cv::Mat geo_corners_to_image_corners_map = cv::Mat(2, 3, CV_64F);
	cv::invertAffineTransform(image_warp_affine_map, geo_corners_to_image_corners_map);
	return MatToPoint(pixels_to_geo_affine_map * Homogenize(MatToPoint(geo_corners_to_image_corners_map * Homogenize(pixel_coords))));
}

cv::Size ImageMetadata::ImageSize() const {
	return size;
}

std::string ImageMetadata::Filename() const {
    return image_filename;
}

}  // namespace image_metadata
}  // namespace ggck
