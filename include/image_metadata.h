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

	ConstCornerIterator GetWarpedImageCornersBegin() const;

	ConstCornerIterator GetWarpedImageCornersEnd() const;

	// Transforms points from the geographic coordinate system into the
	// reference frame of the warped image inside the GeoTIFF.
	cv::Point2f GeoToWarpedImage(const cv::Point2f& geo_coords) const;

	// Projects points from the reference frame of the warped image inside
	// the GeoTIFF into the geographic coordinate system.
	cv::Point2f WarpedImageToGeo(const cv::Point2f& pixel_coords) const;

	// Returns the size of the GeoTIFF image.
	cv::Size GetImageSize() const;

	std::string GetFilename() const;
	
 private:
	const std::string image_filename;

	cv::Size size;

	// A list of 4 points representing the corners of the warped image. Note
	// that these corners are different from the corners of the GeoTIFF (i.e.
	// (0, 0), (width, 0), etc). Since GeoTIFF is partly transparent, the
	// corners of the warped image will be strictly inside the bounding box
	// given by the width and height of the image.  See image_warp_affine_map
	// below.
	std::vector<cv::Point2i> warped_image_corners;

	// A 2x3 matrix which maps the raw image into geographic coordinates.
	cv::Mat pixels_to_geo_affine_map;

	// A 2x3 matrix representing an affine transformation which maps the
	// corners of the GeoTIFF (i.e. (0, 0), (width 0), etc), onto the
	// corners of the warped image inside the GeoTIFF.
	cv::Mat image_warp_affine_map;
};

}  // namespace image_metadata
}  // namespace ggck

#endif  // INCLUDE_IMAGE_METADATA_H_
