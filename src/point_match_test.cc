#include "point_match.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace ggck::point_match;
using namespace std;

int main() {

  Mat im1, im2, im_out;
  const char* const im1_fname = "C:\\Users\\Joseph\\Documents\\Stanford\\CS231A\\Final_Project\\ryker-sf\\data\\im1.png"; //test images are already trimmed with street names removed
  const char* const im2_fname = "C:\\Users\\Joseph\\Documents\\Stanford\\CS231A\\Final_Project\\ryker-sf\\data\\im2.png";
  const char* const im_out_fname = "C:\\Users\\Joseph\\Documents\\Stanford\\CS231A\\Final_Project\\ryker-sf\\data\\matched_points.png";

  // Load test images
  im1 = cv::imread(im1_fname, CV_LOAD_IMAGE_GRAYSCALE);
  im2 = cv::imread(im2_fname, CV_LOAD_IMAGE_GRAYSCALE);
  if (!im1.data)
  {
    cout << "Could not open or find image 1" << std::endl;
    return 1;
  }
  if (!im2.data)
  {
    cout << "Could not open or find image 2" << std::endl;
    return 1;
  }
  cout << "About to get points" << std::endl;
  im_out = GetPoints(im1, im2);

  cv::namedWindow("Display window");
  cv::imshow("Display window", im_out);

  cv::waitKey(0);

  try {
    cv::imwrite(im_out_fname, im_out);
  }
  catch (runtime_error& ex) {
    fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
    return 1;
  }

  return 0;
}
