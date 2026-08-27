#include <filesystem>
#include <iostream>
#include <opencv2/core.hpp>
#include <string>

#include "structured_light/point_cloud_process.hpp"

namespace
{

bool Require(bool condition, const std::string& message)
{
  if (!condition)
  {
    std::cerr << "[FAILED] " << message << '\n';

    return false;
  }

  return true;
}

bool TestTriangulationConversion()
{
  sl3d::TriangulationResult triangulation;

  triangulation.points_3d = cv::Mat::zeros(2, 2, CV_64FC3);

  triangulation.valid_mask = cv::Mat::zeros(2, 2, CV_8UC1);

  triangulation.points_3d.at<cv::Vec3d>(0, 0) = cv::Vec3d(1.0, 2.0, 3.0);

  triangulation.points_3d.at<cv::Vec3d>(1, 1) = cv::Vec3d(4.0, 5.0, 6.0);

  triangulation.valid_mask.at<unsigned char>(0, 0) = 255;

  triangulation.valid_mask.at<unsigned char>(1, 1) = 255;

  triangulation.point_count = 2;

  const auto cloud = sl3d::ConvertTriangulationResultToPointCloud(triangulation);

  return Require(cloud->size() == 2, "Triangulation conversion produced incorrect point count.");
}

bool TestPassThrough()
{
  sl3d::PointCloud cloud;

  cloud.emplace_back(0.0F, 0.0F, 100.0F);

  cloud.emplace_back(0.0F, 0.0F, 200.0F);

  cloud.emplace_back(0.0F, 0.0F, 300.0F);

  cloud.emplace_back(0.0F, 0.0F, 400.0F);

  sl3d::PointCloudProcessingOptions options;

  options.pass_through.enabled = true;

  options.pass_through.axis = "z";

  options.pass_through.min_limit = 150.0F;

  options.pass_through.max_limit = 350.0F;

  options.statistical_outlier.enabled = false;

  options.voxel_grid.enabled = false;

  const auto result = sl3d::ProcessPointCloud(cloud, options);

  return Require(result.cloud->size() == 2, "PassThrough filter produced incorrect point count.");
}

bool TestStatisticalOutlierRemoval()
{
  sl3d::PointCloud cloud;

  for (int i = 0; i < 20; ++i)
  {
    cloud.emplace_back(static_cast<float>(i % 5) * 0.1F, static_cast<float>(i / 5) * 0.1F, 500.0F);
  }

  cloud.emplace_back(1000.0F, 1000.0F, 1500.0F);

  sl3d::PointCloudProcessingOptions options;

  options.pass_through.enabled = false;

  options.statistical_outlier.enabled = true;

  options.statistical_outlier.mean_k = 5;

  options.statistical_outlier.stddev_mul = 1.0;

  options.voxel_grid.enabled = false;

  const auto result = sl3d::ProcessPointCloud(cloud, options);

  bool passed = true;

  passed &= Require(result.cloud->size() < cloud.size(),
                    "StatisticalOutlierRemoval did not remove the outlier.");

  for (const auto& point : *result.cloud)
  {
    passed &=
        Require(point.x < 100.0F, "StatisticalOutlierRemoval retained the synthetic outlier.");
  }

  return passed;
}

bool TestVoxelGrid()
{
  sl3d::PointCloud cloud;

  cloud.emplace_back(0.1F, 0.1F, 0.1F);

  cloud.emplace_back(0.2F, 0.2F, 0.2F);

  cloud.emplace_back(1.2F, 0.1F, 0.1F);

  sl3d::PointCloudProcessingOptions options;

  options.pass_through.enabled = false;

  options.statistical_outlier.enabled = false;

  options.voxel_grid.enabled = true;

  options.voxel_grid.leaf_size = 1.0F;

  const auto result = sl3d::ProcessPointCloud(cloud, options);

  return Require(result.cloud->size() == 2, "VoxelGrid produced incorrect point count.");
}

bool TestPointCloudIo()
{
  sl3d::PointCloud cloud;

  cloud.emplace_back(1.0F, 2.0F, 3.0F);

  cloud.emplace_back(4.0F, 5.0F, 6.0F);

  cloud.width = static_cast<std::uint32_t>(cloud.size());

  cloud.height = 1;
  cloud.is_dense = true;

  const std::filesystem::path temp_dir = std::filesystem::temp_directory_path();

  const std::filesystem::path ascii_ply_path = temp_dir / "sl3d_test_ascii.ply";

  const std::filesystem::path binary_ply_path = temp_dir / "sl3d_test_binary.ply";

  const std::filesystem::path ascii_pcd_path = temp_dir / "sl3d_test_ascii.pcd";

  const std::filesystem::path binary_pcd_path = temp_dir / "sl3d_test_binary.pcd";

  sl3d::SavePointCloud(ascii_ply_path.string(), cloud, false);

  sl3d::SavePointCloud(binary_ply_path.string(), cloud, true);

  sl3d::SavePointCloud(ascii_pcd_path.string(), cloud, false);

  sl3d::SavePointCloud(binary_pcd_path.string(), cloud, true);

  const auto ascii_ply_cloud = sl3d::LoadPointCloud(ascii_ply_path.string());

  const auto binary_ply_cloud = sl3d::LoadPointCloud(binary_ply_path.string());

  const auto ascii_pcd_cloud = sl3d::LoadPointCloud(ascii_pcd_path.string());

  const auto binary_pcd_cloud = sl3d::LoadPointCloud(binary_pcd_path.string());

  bool passed = true;

  passed &= Require(ascii_ply_cloud->size() == cloud.size(), "ASCII PLY round-trip failed.");

  passed &= Require(binary_ply_cloud->size() == cloud.size(), "Binary PLY round-trip failed.");

  passed &= Require(ascii_pcd_cloud->size() == cloud.size(), "ASCII PCD round-trip failed.");

  passed &= Require(binary_pcd_cloud->size() == cloud.size(), "Binary PCD round-trip failed.");

  std::filesystem::remove(ascii_ply_path);

  std::filesystem::remove(binary_ply_path);

  std::filesystem::remove(ascii_pcd_path);

  std::filesystem::remove(binary_pcd_path);

  return passed;
}

}  // namespace

int main()
{
  bool passed = true;

  passed &= TestTriangulationConversion();

  passed &= TestPassThrough();

  passed &= TestStatisticalOutlierRemoval();

  passed &= TestVoxelGrid();

  passed &= TestPointCloudIo();

  if (!passed)
  {
    return 1;
  }

  std::cout << "[PASSED] Point cloud tests.\n";

  return 0;
}