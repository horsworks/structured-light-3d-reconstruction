#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <string>

#include "structured_light/triangulation.hpp"

namespace sl3d
{

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

struct PassThroughFilterOptions
{
  bool enabled = false;

  std::string axis = "z";

  float min_limit = 0.0F;
  float max_limit = 3000.0F;
};

struct StatisticalOutlierFilterOptions
{
  bool enabled = true;

  int mean_k = 30;
  double stddev_mul = 1.0;
};

struct VoxelGridFilterOptions
{
  bool enabled = false;

  float leaf_size = 0.5F;
};

struct PointCloudProcessingOptions
{
  PassThroughFilterOptions pass_through;
  StatisticalOutlierFilterOptions statistical_outlier;
  VoxelGridFilterOptions voxel_grid;
};

struct PointCloudProcessingResult
{
  PointCloud::Ptr cloud;

  std::size_t input_points = 0;
  std::size_t after_remove_nan = 0;
  std::size_t after_pass_through = 0;
  std::size_t after_statistical_outlier = 0;
  std::size_t after_voxel_grid = 0;
};

[[nodiscard]]
PointCloud::Ptr ConvertTriangulationResultToPointCloud(
    const TriangulationResult& triangulation_result);

[[nodiscard]]
PointCloud::Ptr LoadPointCloud(const std::string& file_path);

void SavePointCloud(const std::string& file_path, const PointCloud& cloud, bool binary = false);

[[nodiscard]]
PointCloudProcessingResult ProcessPointCloud(const PointCloud& input_cloud,
                                             const PointCloudProcessingOptions& options);

}  // namespace sl3d