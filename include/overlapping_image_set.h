/*
 * Copyright 2018 Justin Manley and Joseph Bolling.
 */
#ifndef INCLUDE_OVERLAPPING_IMAGE_SET_H_
#define INCLUDE_OVERLAPPING_IMAGE_SET_H_

#include <utility>
#include <vector>

#include "opencv2/opencv.hpp"

#include "geometry.h"
#include "image_metadata.h"

namespace ggck {

struct CvPolygon {
  std::vector<cv::Point> points;
  std::vector<int> npoints;
  int ncontours;
};

struct MaskedImage {
  ImageMetadata metadata;
  cv::Mat image;
  cv::Mat mask;
};

typedef std::pair<ImageMetadata, ImageMetadata> ImagePair;

// An OverlappingImageSet is a polygon in geographic coordinates, along with a
// set of images which depict that territory. Each image is represented by an
// ImageMetadata object, which knows which part of the image represents the
// territory.
class OverlappingImageSet {
 public:
  OverlappingImageSet(const std::vector<ImageMetadata>& image_metadata,
                      const Polygon_2& geo_face_polygon);

  // All pairs of images in this set.
  std::vector<ImagePair> ImagePairs() const;

  // Generate the mask for the image, and return the mask along with the image
  // and its metadata. Note that this method is expensive because it reads the
  // image from disk.
  MaskedImage ComputeImageMask(const ImageMetadata& image_metadata) const;

 private:
  const std::vector<ImageMetadata> image_metadata;

  // A polygon in the projected geo coordinates which represents the
  // area of overlap shared by all images in this set.
  const Polygon_2 geo_face_polygon;

  // TODO(justinmanley): Use delegation and a custom iterator class to avoid
  // storing all of the pairs of ImageMetadata (it should be sufficient to store
  // the integer indices of the pairs).
  const std::vector<ImagePair> image_pairs;
};

}  // namespace ggck

#endif  // INCLUDE_OVERLAPPING_IMAGE_SET_H_
