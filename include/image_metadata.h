#ifndef INCLUDE_IMAGE_METADATA_H_
#define INCLUDE_IMAGE_METADATA_H_

#include <string>
#include "opencv2/core.hpp"

namespace ggck {
namespace image_metadata {

typedef std::vector<cv::Point2i>::const_iterator ConstCornerIterator;

class ImageMetadata {
 public:
	// Note that this constructor may throw an exception if there is an error
	// opening the file.
	ImageMetadata(const std::string& image_filename);

	ConstCornerIterator GetPixelCornersBegin() const;

	ConstCornerIterator GetPixelCornersEnd() const;

	cv::Point2f GeoToPixel(const cv::Point2f& geo_coords) const;

	cv::Point2f PixelToGeo(const cv::Point2f& pixel_coords) const;
	
 private:
	const std::string image_filename;

	// A list of 4 points indicating the corners of the image in pixel coordinates.
	std::vector<cv::Point2i> pixel_corners;

	// A 2x3 matrix which orthorectifies the image and maps the raw image
	// into geographic coordinates.
	cv::Mat pixels_to_geo_affine_map;
};

}  // namespace image_metadata
}  // namespace ggck

#endif  // INCLUDE_IMAGE_METADATA_H_
