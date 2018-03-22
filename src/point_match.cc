/*
 * Copyright 2018 Justin Manley and Joseph Bolling.
 */
#include "point_match.h"

#include <iostream>
#include <memory>
#include <utility>

#include <opencv2/features2d.hpp>

namespace ggck {

typedef cv::Mat Mat;

DensePointsAndMatches ComputePointMatches(const MaskedImage& im1,
                                          const MaskedImage& im2) {
  cv::Ptr<cv::AKAZE> detector = cv::AKAZE::create();
  cv::Ptr<cv::DescriptorMatcher> matcher =
      cv::DescriptorMatcher::create("BruteForce-Hamming");
  const double match_ratio = 0.8;
  const double ransac_threshold = 10.0;

  Mat d1, d2;
  std::vector<cv::KeyPoint> kp1, kp2;
  Mat out_image;
  std::vector<std::vector<cv::DMatch>> matches;
  std::vector<cv::DMatch> pruned_matches;
  std::vector<cv::DMatch> ransaced_matches;
  std::vector<cv::Point2d> p1, p2;

  //  keypoints and descriptors for both images
  detector->detectAndCompute(im1.image, im1.mask, kp1, d1);
  detector->detectAndCompute(im2.image, im2.mask, kp2, d2);

  //  coarse matches
  matcher->knnMatch(d1, d2, matches, 2);

  // Trim out matches that aren't significantly better than the next nearest
  for (unsigned i = 0; i < matches.size(); i++) {
    if (matches[i][0].distance < match_ratio * matches[i][1].distance) {
      pruned_matches.push_back(matches[i][0]);
      p1.push_back(kp1[matches[i][0].queryIdx].pt);
      p2.push_back(kp2[matches[i][0].trainIdx].pt);
    }
  }

  // Use RANSAC to fit a homography and further trim matches
  Mat mask;
  Mat H = cv::findHomography(p1, p2, mask, cv::RANSAC, ransac_threshold);

  for (int i = 0; i < mask.rows; i++) {
    if (mask.at<bool>(i)) {
      ransaced_matches.push_back(pruned_matches[i]);
    }
  }

  // cv::drawMatches(im1.image, kp1, im2.image, kp2, ransaced_matches,
  // out_image);
  // cv::drawKeypoints(im1, kp1, out_image);

  return DensePointsAndMatches{ransaced_matches, kp1, kp2};
}

}  // namespace ggck
