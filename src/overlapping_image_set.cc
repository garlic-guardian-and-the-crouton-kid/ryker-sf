/*
 * Copyright 2018 Justin Manley and Joseph Bolling.
 */
#include "overlapping_image_set.h"

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

namespace ggck {

namespace {

CvPolygon CgalToCvPolygon(const ImageMetadata& image,
                          const Polygon_2& face_polygon) {
  std::vector<cv::Point> points;
  std::transform(face_polygon.vertices_begin(), face_polygon.vertices_end(),
                 std::back_inserter(points), [image](const Point_2& point) {
                   return image.GeoToImage(cv::Point2f(
                       CGAL::to_double(point.x()), CGAL::to_double(point.y())));
                 });

  int npoints[1] = {static_cast<int>(points.size())};
  int ncontours = 1;

  return CvPolygon{
      points, npoints, ncontours,
  };
}

// TODO(justinmanley): Replace this with an implementation which constructs
// a vector of integer index pairs and delegates iteration to that vector in
// order to save memory.
std::vector<ImagePair> ImagePairs(
    const std::vector<ImageMetadata>& image_metadata) {
  std::vector<ImagePair> image_pairs;
  for (auto im1 = image_metadata.begin(); im1 != image_metadata.end(); im1++) {
    for (auto im2 = im1 + 1; im2 != image_metadata.end(); im2++) {
      image_pairs.push_back(std::make_pair(*im1, *im2));
    }
  }
  return image_pairs;
}

}  // namespace

OverlappingImageSet::OverlappingImageSet(
    const std::vector<ImageMetadata>& image_metadata,
    const Polygon_2& geo_face_polygon)
    : image_metadata(image_metadata),
      geo_face_polygon(geo_face_polygon),
      image_pairs(ImagePairs(image_metadata)) {}

ConstImagePairIterator OverlappingImageSet::ImagePairsBegin() const {
  return image_pairs.begin();
}

ConstImagePairIterator OverlappingImageSet::ImagePairsEnd() const {
  return image_pairs.end();
}

MaskedImage OverlappingImageSet::ComputeImageMask(
    const ImageMetadata& image_metadata) const {
  cv::Size image_size = image_metadata.ImageSize();
  // NOTE: We may want to swap the height and width of this matrix, depending
  // on how OpenCV loads images for matching, etc. If we swap the height and
  // width of the matrix, then we also need to swap the coordinates of the
  // points in the mask polygons.
  cv::Mat mask = cv::Mat::zeros(image_size, CV_8U);

  CvPolygon polygon = CgalToCvPolygon(image_metadata, geo_face_polygon);

  // TODO(justinmanley): Why is dynamic allocation necessary here?
  const cv::Point** points = new const cv::Point*[polygon.points.size()];
  for (int k = 0; k < polygon.points.size(); k++) {
    points[k] = &polygon.points[k];
  }

  cv::Scalar color = cv::Scalar(255);
  cv::fillPoly(mask, points, polygon.npoints, polygon.ncontours, color);

  delete[] points;

  cv::Mat image =
      cv::imread(image_metadata.Filename(), CV_LOAD_IMAGE_GRAYSCALE);

  return MaskedImage{
      image_metadata, image, mask,
  };
}

}  // namespace ggck
