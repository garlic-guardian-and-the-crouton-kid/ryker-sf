#include <opencv2/features2d.hpp>
#include "point_match.h"
#include <memory>
#include <iostream>


namespace ggck {
namespace point_match {
 
Mat GetPoints(Mat im1, Mat im2) {
  cv::Ptr<cv::ORB> detector = cv::ORB::create();
  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

  Mat d1, d2;
  std::vector<cv::KeyPoint> kp1, kp2;
  Mat out_image;

  // Get keypoints and descriptors for both images
  detector->detectAndCompute(im1, cv::noArray(), kp1, d1);
  detector->detectAndCompute(im2, cv::noArray(), kp2, d2);

  cv::drawKeypoints(im1, kp1, out_image);

  return out_image;
}

} //namespace point_match
} //namespace ggck