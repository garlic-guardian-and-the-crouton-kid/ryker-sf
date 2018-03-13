#include <opencv2/features2d.hpp>
#include "point_match.h"
#include <memory>


namespace ggck {
namespace point_match {
 
Mat GetPoints(Mat im1, Mat im2) {

  // UNCERTAIN: does create() get called properly by make_unique?
  cv::ORB* detector = cv::ORB::create();
  cv::DescriptorMatcher* matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

  Mat d1, d2;
  std::vector<cv::KeyPoint> kp1, kp2;

  // UNCERTAIN: NULL for mask input?
  detector->detectAndCompute(im1, NULL, kp1, d1);

  Mat points;
  return points;
}

} //namespace point_match
} //namespace ggck