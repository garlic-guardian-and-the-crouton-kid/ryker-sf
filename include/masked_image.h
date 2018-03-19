/*
 * Copyright 2018 Justin Manley and Joseph Bolling.
 */
#ifndef INCLUDE_MASKED_IMAGE_H_
#define INCLUDE_MASKED_IMAGE_H_

#include "image_metadata.h"

namespace ggck {
namespace masked_image {

struct MaskedImage {
	image_metadata::ImageMetadata metadata;
	cv::Mat mask;
	cv::Mat image;
};

}  // namespace masked_image
}  // namespace ggck

#endif  // INCLUDE_MASKED_IMAGE_H_

