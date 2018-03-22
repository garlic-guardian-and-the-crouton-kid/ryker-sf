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
#include "point_set.h"
#include "triangulate.h"

#include <iostream>
#include <fstream>

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

void WritePointCloudToFile(std::vector<cv::Point3d> pc, const char * fname) {

  std::ofstream file(fname);
  if (file.is_open())
  {
    for (auto & point : pc)
    {
      file << point.x << "," << point.y << "," << point.z << std::endl;
    }
    file.close();
  }
  else
  {
    std::cout << "Unable to open output file";
  }
}

int main(int argc, char* argv[]) {
  GDALAllRegister();
  std::vector<ImageMetadata> metadata = GetImageMetadata(argc, argv);

  std::vector<OverlappingImageSet> overlaps =
      ComputeOverlaps(GetImageMetadata(argc, argv));
  auto points = std::make_unique<ggck::PointSet>(metadata);
  
  for (auto overlap = overlaps.begin(); overlap != overlaps.end(); overlap++) {
    for (auto image_pair = overlap->ImagePairsBegin();
         image_pair != overlap->ImagePairsEnd(); image_pair++) {
      ggck::DensePointsAndMatches dpMatches =
          ComputePointMatches(overlap->ComputeImageMask(image_pair->first),
                              overlap->ComputeImageMask(image_pair->second));
                              
      points->Add(dpMatches,*image_pair);
    }
  }

  std::vector<cv::Point3d> adjustedPointCloud = RunSba(points.get());
  std::vector<cv::Point3d> initialPointCloud = points->GetPointCloud();
  WritePointCloudToFile(adjustedPointCloud, "adjusted_points.csv");
  WritePointCloudToFile(initialPointCloud, "initial_points.csv");

  return 0;
}
