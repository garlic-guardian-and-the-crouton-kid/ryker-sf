#include "point_match.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace ggck::point_match;
using namespace std;

int main() {

  Mat image;
  const char* const im1_fname = "data\\im1.png";
  image = cv::imread(im1_fname, CV_LOAD_IMAGE_GRAYSCALE);

  if (!image.data)
  {
    cout << "Could not open or find the image" << std::endl;
    return -1;
  }


  cv::namedWindow("Display window");// Create a window for display.
  cv::imshow("Display window", image);                   // Show our image inside it.

  cv::waitKey(0);                                          // Wait for a keystroke in the window
  return 0;
}
