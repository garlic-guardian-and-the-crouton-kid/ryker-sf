/*
 * Copyright 2018 Justin Manley and Joseph Bolling.
 */
#include "gdal.h"
#include "gdal_priv.h"

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"

#include "partition.h"
#include "image_metadata.h"
#include "point_match.h"
#include "overlapping_image_set.h"

using ggck::partition::ComputeOverlaps;
using ggck::image_metadata::ImageMetadata;
using ggck::point_match::GetPointMatches;
using ggck::overlapping_image_set::OverlappingImageSet;

const std::vector<std::string> image_paths = {
	"/home/justin/p/rykers-sf/images/512w/001.tif",
	"/home/justin/p/rykers-sf/images/512w/002.tif",
};

int main() {
  GDALAllRegister();

	std::vector<ImageMetadata> image_metadata;
	std::transform(image_paths.begin(), image_paths.end(), std::back_inserter(image_metadata),
			[](const std::string& image_path) {
				ImageMetadata metadata(image_path);
				return metadata;
			});

	std::vector<OverlappingImageSet> overlaps = ComputeOverlaps(image_metadata);

	for (auto overlap = overlaps.begin(); overlap != overlaps.end(); overlap++) {
		for (auto image_pair = overlap->GetImagePairsBegin(); image_pair != overlap->GetImagePairsEnd(); image_pair++) {
			cv::Mat matches = GetPointMatches(
					overlap->GetMaskedImage(image_pair->first),
					overlap->GetMaskedImage(image_pair->second));
			cv::imshow("Matching points", matches);
			cv::waitKey(0);
		}
	}

  return 0;
}
