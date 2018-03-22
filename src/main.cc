/*
 * Copyright 2018 Justin Manley and Joseph Bolling.
 */
#include <fstream>
#include <iostream>

#include "gdal.h"
#include "gdal_priv.h"

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include "image_metadata.h"
#include "overlapping_image_set.h"
#include "partition.h"
#include "point_match.h"
#include "point_set.h"
#include "triangulate.h"

using ggck::ImageMetadata;
using ggck::OverlappingImageSet;
using ggck::ImagePair;

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

void WritePointCloudToFile(std::vector<cv::Point3d> pc, const char* fname) {
  int prec = 12;
  std::ofstream file(fname);
  if (file.is_open()) {
    for (auto& point : pc) {
      file << std::setprecision(prec) << point.x << "," 
           << std::setprecision(prec) << point.y << "," 
           << std::setprecision(prec) << point.z << std::endl;
    }
    file.close();
  } else {
    std::cout << "Unable to open output file";
  }
}

int TotalImagePairs(const std::vector<OverlappingImageSet> overlaps) {
  int total_image_pairs = 0;
  for (int i = 0; i < overlaps.size(); i++) {
    std::vector<ImagePair> image_pairs = overlaps[i].ImagePairs();
    for (int j = 0; j < image_pairs.size(); j++) {
      total_image_pairs++;
    }
  }
  return total_image_pairs;
}

void PrintPointMatchingStatus(const int current_image_pair,
                              const int num_image_pairs) {
  std::cout << "Finding point matches in image pair " << current_image_pair + 1
            << " / " << num_image_pairs << "\r" << std::flush;
}

int main(int argc, char* argv[]) {
  GDALAllRegister();
  std::vector<ImageMetadata> metadata = GetImageMetadata(argc, argv);

  std::vector<OverlappingImageSet> overlaps =
      ComputeOverlaps(GetImageMetadata(argc, argv));

  auto points = std::make_unique<ggck::PointSet>(metadata);
  int total_image_pairs = TotalImagePairs(overlaps);
  int current_image_pair = 0;
  for (int i = 0; i < overlaps.size(); i++) {
    OverlappingImageSet overlap = overlaps[i];
    std::vector<ImagePair> image_pairs = overlap.ImagePairs();
    for (int j = 0; j < image_pairs.size(); j++) {
      ImagePair image_pair = image_pairs[j];
      PrintPointMatchingStatus(current_image_pair, total_image_pairs);
      ggck::DensePointsAndMatches matches =
          ComputePointMatches(overlap.ComputeImageMask(image_pair.first),
                              overlap.ComputeImageMask(image_pair.second));
      points->Add(matches, image_pair);
      current_image_pair++;
    }
  }

  std::vector<cv::Point3d> adjustedPointCloud = RunSba(points.get());
  std::vector<cv::Point3d> initialPointCloud = points->GetPointCloud();
  WritePointCloudToFile(adjustedPointCloud, "adjusted_points.csv");
  WritePointCloudToFile(initialPointCloud, "initial_points.csv");

  return 0;
}
