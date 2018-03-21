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

// The image filenames should be provided as the only command-line arguments
// to the program.
std::vector<ImageMetadata> GetImageMetadata(int argc, char* argv[]) {
	std::vector<ImageMetadata> image_metadata;
	for (int i = 1; i < argc; i++) {
		std::string image_filename = std::string(argv[i]);
		image_metadata.push_back(ImageMetadata(image_filename));
	}
	return image_metadata;
}

int main(int argc, char* argv[]) {
  GDALAllRegister();

  std::vector<OverlappingImageSet> overlaps = ComputeOverlaps(
			GetImageMetadata(argc, argv));

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
