#ifndef INCLUDE_OVERLAPPING_IMAGE_SET_
#define INCLUDE_OVERLAPPING_IMAGE_SET_

#include <vector>

#include "opencv2/opencv.hpp"

#include "geometry.h"
#include "image_metadata.h"

namespace ggck {
namespace overlapping_image_set {

struct CvPolygon {
	std::vector<cv::Point> points;
	int* npoints;
	int ncontours;
};

struct MaskedImage {
	image_metadata::ImageMetadata metadata;
	cv::Mat image;
	cv::Mat mask;
};

typedef std::pair<image_metadata::ImageMetadata, image_metadata::ImageMetadata> ImagePair;
typedef std::vector<ImagePair>::const_iterator ConstImagePairIterator;

// An OverlappingImageSet is a polygon in geographic coordinates, along with a set of images
// which depict that territory. Each image is represented by an ImageMetadata object, which
// knows which part of the image represents the territory.
class OverlappingImageSet {
 public:
	OverlappingImageSet(const std::vector<image_metadata::ImageMetadata>& image_metadata, const Polygon_2& geo_face_polygon);

	// Convenience functions for iterating over all pairs of images in this set.
	// This is useful for point matching, which takes images in pairs.
	ConstImagePairIterator ImagePairsBegin() const;
	ConstImagePairIterator ImagePairsEnd() const;

  // Generate the mask for the image, and return the mask along with the image and
  // its metadata. Note that this method is expensive because it reads the image
	// from disk.
	MaskedImage ComputeImageMask(const image_metadata::ImageMetadata& image_metadata) const;

 private:
	const std::vector<image_metadata::ImageMetadata> image_metadata;

	// A polygon in the projected geo coordinates which represents the
	// area of overlap shared by all images in this set.
	const Polygon_2 geo_face_polygon;

	// TODO: Use delegation and a custom iterator class to avoid storing all of
	// the pairs of image_metadata::ImageMetadata (it should be sufficient to store the integer
	// indices of the pairs).
	const std::vector<ImagePair> image_pairs;
};

}  // namespace overlapping_image_set
}  // namespace ggck

#endif  // INCLUDE_OVERLAPPING_IMAGE_SET_
