/*
 * Copyright 2018 Justin Manley and Joseph Bolling.
 */
#include "gdal.h"
#include "gdal_priv.h"

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include "image_metadata.h"
#include "overlapping_image_set.h"
#include "partition.h"
#include "point_match.h"

using ggck::ImageMetadata;
using ggck::OverlappingImageSet;

const std::vector<std::string> image_paths = {
    "/home/justin/p/rykers-sf/images/512w/001.tif",
    "/home/justin/p/rykers-sf/images/512w/002.tif",
};

int main() {
  GDALAllRegister();

  std::vector<ImageMetadata> image_metadata;
  std::transform(image_paths.begin(), image_paths.end(),
                 std::back_inserter(image_metadata),
                 [](const std::string& image_path) {
                   ImageMetadata metadata(image_path);
                   return metadata;
                 });

  std::vector<OverlappingImageSet> overlaps = ComputeOverlaps(image_metadata);

  for (auto overlap = overlaps.begin(); overlap != overlaps.end(); overlap++) {
    for (auto image_pair = overlap->ImagePairsBegin();
         image_pair != overlap->ImagePairsEnd(); image_pair++) {
      cv::Mat matches =
          PointMatches(overlap->ComputeImageMask(image_pair->first),
                       overlap->ComputeImageMask(image_pair->second));
      cv::imshow("Matching points", matches);
      cv::waitKey(0);
    }
  }

  return 0;
}
