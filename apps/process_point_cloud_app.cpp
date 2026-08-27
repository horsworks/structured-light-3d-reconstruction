#include <iostream>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <string>

#include "structured_light/point_cloud_process.hpp"

namespace
{

struct ProcessingConfig
{
  std::string input_file;
  std::string output_file;

  bool binary = false;

  sl3d::PointCloudProcessingOptions processing;
};

bool ReadBool(const cv::FileNode& node, const std::string& name, bool default_value)
{
  const cv::FileNode value_node = node[name];

  if (value_node.empty())
  {
    return default_value;
  }

  int value = default_value ? 1 : 0;

  value_node >> value;

  return value != 0;
}

ProcessingConfig LoadConfig(const std::string& config_path)
{
  cv::FileStorage file_storage(config_path, cv::FileStorage::READ);

  if (!file_storage.isOpened())
  {
    throw std::runtime_error("Failed to open point cloud processing config: " + config_path);
  }

  ProcessingConfig config;

  const cv::FileNode input_node = file_storage["input"];

  const cv::FileNode pass_node = file_storage["pass_through"];

  const cv::FileNode statistical_node = file_storage["statistical_outlier"];

  const cv::FileNode voxel_node = file_storage["voxel_grid"];

  const cv::FileNode output_node = file_storage["output"];

  input_node["point_cloud_file"] >> config.input_file;

  config.processing.pass_through.enabled =
      ReadBool(pass_node, "enabled", config.processing.pass_through.enabled);

  pass_node["axis"] >> config.processing.pass_through.axis;

  pass_node["min_limit"] >> config.processing.pass_through.min_limit;

  pass_node["max_limit"] >> config.processing.pass_through.max_limit;

  config.processing.statistical_outlier.enabled =
      ReadBool(statistical_node, "enabled", config.processing.statistical_outlier.enabled);

  statistical_node["mean_k"] >> config.processing.statistical_outlier.mean_k;

  statistical_node["stddev_mul"] >> config.processing.statistical_outlier.stddev_mul;

  config.processing.voxel_grid.enabled =
      ReadBool(voxel_node, "enabled", config.processing.voxel_grid.enabled);

  voxel_node["leaf_size"] >> config.processing.voxel_grid.leaf_size;

  output_node["point_cloud_file"] >> config.output_file;

  config.binary = ReadBool(output_node, "binary", config.binary);

  if (config.input_file.empty())
  {
    throw std::runtime_error("Input point cloud file must not be empty.");
  }

  if (config.output_file.empty())
  {
    throw std::runtime_error("Output point cloud file must not be empty.");
  }

  return config;
}

}  // namespace

int main(int argc, char* argv[])
{
  try
  {
    if (argc != 2)
    {
      std::cerr << "Usage:\n"
                << "  process_point_cloud <config.yaml>\n";

      return 1;
    }

    const ProcessingConfig config = LoadConfig(argv[1]);

    const sl3d::PointCloud::Ptr input_cloud = sl3d::LoadPointCloud(config.input_file);

    const sl3d::PointCloudProcessingResult result =
        sl3d::ProcessPointCloud(*input_cloud, config.processing);

    if (!result.cloud || result.cloud->empty())
    {
      throw std::runtime_error("Point cloud processing produced an empty cloud.");
    }

    sl3d::SavePointCloud(config.output_file, *result.cloud, config.binary);

    std::cout << "Point cloud processing completed.\n"
              << "Input points: " << result.input_points << '\n'
              << "After NaN removal: " << result.after_remove_nan << '\n';

    if (config.processing.pass_through.enabled)
    {
      std::cout << "After PassThrough: " << result.after_pass_through << '\n';
    }

    if (config.processing.statistical_outlier.enabled)
    {
      std::cout << "After StatisticalOutlierRemoval: " << result.after_statistical_outlier << '\n';
    }

    if (config.processing.voxel_grid.enabled)
    {
      std::cout << "After VoxelGrid: " << result.after_voxel_grid << '\n';
    }

    std::cout << "Output points: " << result.cloud->size() << '\n'
              << "Output format: " << (config.binary ? "binary" : "ascii") << '\n'
              << "Saved to: " << config.output_file << '\n';
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error: " << e.what() << '\n';

    return 1;
  }

  return 0;
}