/*
 * Copyright 2018 Justin Manley and Joseph Bolling.
 */
#include "point_match.h"

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "image_metadata.h"
#include "overlapping_image_set.h"

using ggck::ImageMetadata;
using ggck::MaskedImage;
using cv::Mat;

int main() {
  Mat im_out;
  const char* const im1_fname =
      "C:\\Users\\Joseph\\Documents\\Stanford\\CS231A\\Final_Project\\ryker-"
      "sf\\data\\im1.png";  // test images are already trimmed with street names
                            // removed
  const char* const im2_fname =
      "C:\\Users\\Joseph\\Documents\\Stanford\\CS231A\\Final_Project\\ryker-"
      "sf\\data\\im2.png";
  const char* const im_out_fname =
      "C:\\Users\\Joseph\\Documents\\Stanford\\CS231A\\Final_Project\\ryker-"
      "sf\\data\\matched_points.png";

  ImageMetadata im1_metadata = ImageMetadata{im1_fname};
  ImageMetadata im2_metadata = ImageMetadata{im2_fname};

  // Load test images
  MaskedImage im1 = {
      im1_metadata, cv::imread(im1_fname, CV_LOAD_IMAGE_GRAYSCALE),
      Mat::ones(im1_metadata.ImageSize(), CV_8U),
  };
  MaskedImage im2 = {
      im2_metadata, cv::imread(im2_fname, CV_LOAD_IMAGE_GRAYSCALE),
      Mat::ones(im2_metadata.ImageSize(), CV_8U),
  };

  if (!im1.image.data) {
    std::cout << "Could not open or find image 1" << std::endl;
    return 1;
  }
  if (!im2.image.data) {
    std::cout << "Could not open or find image 2" << std::endl;
    return 1;
  }
  std::cout << "About to get points" << std::endl;
  im_out = PointMatches(im1, im2);

  cv::namedWindow("Display window");
  cv::imshow("Display window", im_out);

  cv::waitKey(0);

  try {
    cv::imwrite(im_out_fname, im_out);
  } catch (std::runtime_error& ex) {
    fprintf(stderr, "Exception converting image to PNG format: %s\n",
            ex.what());
    return 1;
  }

  return 0;
}
