/*
 * Copyright 2018 Justin Manley and Joseph Bolling.
 */
#include "point_match.h"

#include <iostream>
#include <memory>
#include <utility>

#include <opencv2/features2d.hpp>

namespace ggck {

using cv::Mat;
using cv::DMatch;
using cv::DescriptorMatcher;
using cv::AKAZE;
using cv::Ptr;

constexpr double match_ratio = 0.8;
constexpr double ransac_threshold = 10.0;
constexpr int min_required_matches = 8;

DensePointsAndMatches ComputePointMatches(const MaskedImage& im1,
                                          const MaskedImage& im2) {
  Ptr<AKAZE> detector = AKAZE::create();
  Ptr<DescriptorMatcher> matcher =
      DescriptorMatcher::create("BruteForce-Hamming");

  // Keypoints and descriptors for both images.
  Mat descriptors1, descriptors2;
  std::vector<cv::KeyPoint> keypoints1, keypoints2;
  detector->detectAndCompute(im1.image, im1.mask, keypoints1, descriptors1);
  detector->detectAndCompute(im2.image, im2.mask, keypoints2, descriptors2);
  // If the feature extractor is not able to find keypoints for one or both
  // images, knn matching will still generate matches between the descriptor
  // matrices, but the matches will be spurious, because one (or both)
  // descriptor matrices will be uninitialized.
  if (keypoints1.empty() || keypoints2.empty()) {
    std::vector<DMatch> no_matches;
    return DensePointsAndMatches{no_matches, keypoints1, keypoints2};
  }

  // Coarse matches
  std::vector<std::vector<DMatch>> matches;
  matcher->knnMatch(descriptors1, descriptors2, matches, 2);

  // Trim out matches that aren't significantly better than the next nearest
  std::vector<DMatch> pruned_matches;
  std::vector<cv::Point2d> matches_im1, matches_im2;
  for (const std::vector<DMatch>& match : matches) {
    // If there are fewer than k keypoints in either image, then match will have
    // fewer than k elements. In extremely feature-poor images, there may not
    // even be two keypoints to compare; we ignore such keypoints.
    if (match.size() > 1 &&
        match[0].distance < match_ratio * match[1].distance) {
      pruned_matches.push_back(match[0]);

      matches_im1.push_back(keypoints1[match[0].queryIdx].pt);
      matches_im2.push_back(keypoints2[match[0].trainIdx].pt);
    }
  }

  // Use RANSAC to fit a homography and further trim matches
  // We only want to fit a homography if there is more than one pruned match,
  // since findHomography throws an error if it is given empty point sets.
  std::vector<DMatch> ransaced_matches;
  if (!pruned_matches.empty()) {
    Mat mask;
    Mat H = cv::findHomography(matches_im1, matches_im2, mask, cv::RANSAC,
                               ransac_threshold);

    for (int i = 0; i < mask.rows; i++) {
      if (mask.at<bool>(i)) {
        ransaced_matches.push_back(pruned_matches[i]);
      }
    }
  }

  // Draw image
  /*
  Mat out_image;
  cv::drawMatches(im1.image, keypoints1, im2.image, keypoints2,
                  ransaced_matches, out_image);
  cv::imshow("Point matches", out_image);
  cv::waitKey(0);*/

  // If less than min_required_matches, throw out the matches before returning
  // since the histogram was likely overfitted.
  if (ransaced_matches.size() < min_required_matches) {
    ransaced_matches.clear();
  }

  return DensePointsAndMatches{ransaced_matches, keypoints1, keypoints2};
}

}  // namespace ggck
