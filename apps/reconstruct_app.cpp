#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "structured_light/phase.hpp"
#include "structured_light/phase_unwrap.hpp"
#include "structured_light/point_cloud_process.hpp"
#include "structured_light/triangulation.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

constexpr double kTwoPi = 2.0 * kPi;

struct ReconstructionConfig
{
  std::string image_dir;

  std::string camera_calibration_file;
  std::string projector_calibration_file;

  std::array<int, 3> frequencies = {70, 64, 59};

  int phase_steps = 4;

  double modulation_threshold = 10.0;

  sl3d::ProjectorCoordinateAxis axis = sl3d::ProjectorCoordinateAxis::kU;

  sl3d::TriangulationOptions triangulation;

  std::string point_cloud_file;

  bool point_cloud_binary = false;

  std::string depth_image;
  std::string valid_mask;
  std::string projector_coordinate_image;
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

bool IsImageFile(const std::filesystem::path& path)
{
  std::string extension = path.extension().string();

  std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char character)
                 { return static_cast<char>(std::tolower(character)); });

  return extension == ".bmp" || extension == ".png" || extension == ".jpg" ||
         extension == ".jpeg" || extension == ".tif" || extension == ".tiff";
}

bool ParseNumericStem(const std::filesystem::path& path, long long& value)
{
  const std::string stem = path.stem().string();

  if (stem.empty())
  {
    return false;
  }

  const bool is_numeric = std::all_of(stem.begin(), stem.end(), [](unsigned char character)
                                      { return std::isdigit(character) != 0; });

  if (!is_numeric)
  {
    return false;
  }

  try
  {
    value = std::stoll(stem);

    return true;
  }
  catch (const std::exception&)
  {
    return false;
  }
}

bool ImagePathLess(const std::filesystem::path& lhs, const std::filesystem::path& rhs)
{
  long long lhs_value = 0;
  long long rhs_value = 0;

  const bool lhs_numeric = ParseNumericStem(lhs, lhs_value);

  const bool rhs_numeric = ParseNumericStem(rhs, rhs_value);

  if (lhs_numeric && rhs_numeric)
  {
    return lhs_value < rhs_value;
  }

  return lhs.filename().string() < rhs.filename().string();
}

ReconstructionConfig LoadConfig(const std::string& config_path)
{
  cv::FileStorage file_storage(config_path, cv::FileStorage::READ);

  if (!file_storage.isOpened())
  {
    throw std::runtime_error("Failed to open reconstruction config: " + config_path);
  }

  ReconstructionConfig config;

  const cv::FileNode input_node = file_storage["input"];

  const cv::FileNode fringe_node = file_storage["fringe"];

  const cv::FileNode triangulation_node = file_storage["triangulation"];

  const cv::FileNode output_node = file_storage["output"];

  input_node["image_dir"] >> config.image_dir;

  input_node["camera_calibration_file"] >> config.camera_calibration_file;

  input_node["projector_calibration_file"] >> config.projector_calibration_file;

  std::string orientation;

  fringe_node["orientation"] >> orientation;

  if (orientation == "vertical")
  {
    config.axis = sl3d::ProjectorCoordinateAxis::kU;
  }
  else if (orientation == "horizontal")
  {
    config.axis = sl3d::ProjectorCoordinateAxis::kV;
  }
  else
  {
    throw std::runtime_error("Fringe orientation must be vertical or horizontal.");
  }

  std::vector<int> frequencies;

  fringe_node["frequencies"] >> frequencies;

  if (frequencies.size() != 3)
  {
    throw std::runtime_error("Current reconstruction requires exactly three frequencies.");
  }

  for (std::size_t i = 0; i < 3; ++i)
  {
    config.frequencies[i] = frequencies[i];
  }

  fringe_node["phase_steps"] >> config.phase_steps;

  fringe_node["modulation_threshold"] >> config.modulation_threshold;

  triangulation_node["min_depth"] >> config.triangulation.min_depth;

  triangulation_node["max_depth"] >> config.triangulation.max_depth;

  triangulation_node["max_iterations"] >> config.triangulation.max_iterations;

  triangulation_node["projector_residual_threshold"] >>
      config.triangulation.projector_residual_threshold;

  triangulation_node["pixel_step"] >> config.triangulation.pixel_step;

  config.triangulation.axis = config.axis;

  output_node["point_cloud_file"] >> config.point_cloud_file;

  config.point_cloud_binary =
      ReadBool(output_node, "point_cloud_binary", config.point_cloud_binary);

  output_node["depth_image"] >> config.depth_image;

  output_node["valid_mask"] >> config.valid_mask;

  output_node["projector_coordinate_image"] >> config.projector_coordinate_image;

  if (config.image_dir.empty() || config.camera_calibration_file.empty() ||
      config.projector_calibration_file.empty())
  {
    throw std::runtime_error("Reconstruction input paths must not be empty.");
  }

  if (config.point_cloud_file.empty())
  {
    throw std::runtime_error("Point cloud output file must not be empty.");
  }

  if (config.phase_steps < 3)
  {
    throw std::runtime_error("phase_steps must be at least 3.");
  }

  return config;
}

std::vector<std::filesystem::path> CollectImagePaths(const std::filesystem::path& image_dir)
{
  if (!std::filesystem::exists(image_dir))
  {
    throw std::runtime_error("Reconstruction image directory does not exist: " +
                             image_dir.string());
  }

  std::vector<std::filesystem::path> image_paths;

  for (const auto& entry : std::filesystem::directory_iterator(image_dir))
  {
    if (entry.is_regular_file() && IsImageFile(entry.path()))
    {
      image_paths.push_back(entry.path());
    }
  }

  std::sort(image_paths.begin(), image_paths.end(), ImagePathLess);

  return image_paths;
}

std::vector<cv::Mat> LoadImages(const std::vector<std::filesystem::path>& image_paths)
{
  std::vector<cv::Mat> images;

  images.reserve(image_paths.size());

  cv::Size image_size;

  for (const auto& path : image_paths)
  {
    cv::Mat image = cv::imread(path.string(), cv::IMREAD_GRAYSCALE);

    if (image.empty())
    {
      throw std::runtime_error("Failed to read image: " + path.string());
    }

    if (image_size.empty())
    {
      image_size = image.size();
    }
    else if (image.size() != image_size)
    {
      throw std::runtime_error("All reconstruction images must have the same size.");
    }

    images.emplace_back(std::move(image));
  }

  return images;
}

struct DecodedCoordinate
{
  cv::Mat coordinate;
  cv::Mat valid_mask;
};

DecodedCoordinate DecodeProjectorCoordinate(const std::vector<cv::Mat>& images,
                                            const ReconstructionConfig& config,
                                            const cv::Size& projector_size)
{
  const std::size_t expected_image_count = 3 * static_cast<std::size_t>(config.phase_steps);

  if (images.size() != expected_image_count)
  {
    throw std::runtime_error("Unexpected reconstruction image count.");
  }

  std::array<sl3d::PhaseResult, 3> phase_results;

  for (std::size_t frequency = 0; frequency < 3; ++frequency)
  {
    std::vector<cv::Mat> phase_images;

    phase_images.reserve(static_cast<std::size_t>(config.phase_steps));

    const std::size_t start_index = frequency * static_cast<std::size_t>(config.phase_steps);

    for (int phase = 0; phase < config.phase_steps; ++phase)
    {
      phase_images.push_back(images[start_index + static_cast<std::size_t>(phase)]);
    }

    phase_results[frequency] = sl3d::ComputeWrappedPhase(phase_images, config.modulation_threshold);
  }

  const sl3d::PhaseUnwrapResult unwrapped =
      sl3d::UnwrapThreeFrequencyPhase(phase_results, config.frequencies);

  DecodedCoordinate result;

  result.coordinate = cv::Mat::zeros(images.front().size(), CV_64FC1);

  result.valid_mask = unwrapped.valid_mask.clone();

  const double dimension = config.axis == sl3d::ProjectorCoordinateAxis::kU
                               ? static_cast<double>(projector_size.width)
                               : static_cast<double>(projector_size.height);

  const double scale = dimension / (kTwoPi * static_cast<double>(config.frequencies[0]));

  for (int y = 0; y < result.coordinate.rows; ++y)
  {
    const auto* phase_row = unwrapped.absolute_phase.ptr<double>(y);

    auto* coordinate_row = result.coordinate.ptr<double>(y);

    auto* valid_row = result.valid_mask.ptr<unsigned char>(y);

    for (int x = 0; x < result.coordinate.cols; ++x)
    {
      if (valid_row[x] == 0)
      {
        continue;
      }

      const double coordinate = phase_row[x] * scale;

      if (!std::isfinite(coordinate) || coordinate < 0.0 || coordinate >= dimension)
      {
        valid_row[x] = 0;

        continue;
      }

      coordinate_row[x] = coordinate;
    }
  }

  return result;
}

void EnsureParentDirectory(const std::string& file_path)
{
  const std::filesystem::path path = file_path;

  if (!path.parent_path().empty())
  {
    std::filesystem::create_directories(path.parent_path());
  }
}

void SaveDepthImage(const std::string& file_path, const sl3d::TriangulationResult& result,
                    double min_depth, double max_depth)
{
  EnsureParentDirectory(file_path);

  cv::Mat depth_image = cv::Mat::zeros(result.depth.size(), CV_8UC1);

  const double scale = 255.0 / (max_depth - min_depth);

  for (int y = 0; y < result.depth.rows; ++y)
  {
    for (int x = 0; x < result.depth.cols; ++x)
    {
      if (result.valid_mask.at<unsigned char>(y, x) == 0)
      {
        continue;
      }

      const double depth = result.depth.at<double>(y, x);

      const double normalized = std::clamp((depth - min_depth) * scale, 0.0, 255.0);

      depth_image.at<unsigned char>(y, x) = static_cast<unsigned char>(std::lround(normalized));
    }
  }

  cv::imwrite(file_path, depth_image);
}

void SaveCoordinateImage(const std::string& file_path, const DecodedCoordinate& decoded,
                         double dimension)
{
  EnsureParentDirectory(file_path);

  cv::Mat coordinate_image = cv::Mat::zeros(decoded.coordinate.size(), CV_8UC1);

  decoded.coordinate.convertTo(coordinate_image, CV_8U, 255.0 / dimension);

  cv::Mat invalid_mask;

  cv::bitwise_not(decoded.valid_mask, invalid_mask);

  coordinate_image.setTo(0, invalid_mask);

  cv::imwrite(file_path, coordinate_image);
}

}  // namespace

int main(int argc, char* argv[])
{
  try
  {
    if (argc != 2)
    {
      std::cerr << "Usage:\n"
                << "  reconstruct <config.yaml>\n";

      return 1;
    }

    const ReconstructionConfig config = LoadConfig(argv[1]);

    const sl3d::CameraProjectorCalibration calibration = sl3d::LoadCameraProjectorCalibration(
        config.camera_calibration_file, config.projector_calibration_file);

    const auto image_paths = CollectImagePaths(config.image_dir);

    const std::size_t expected_image_count = 3 * static_cast<std::size_t>(config.phase_steps);

    if (image_paths.size() != expected_image_count)
    {
      throw std::runtime_error("Expected " + std::to_string(expected_image_count) +
                               " reconstruction images, but found " +
                               std::to_string(image_paths.size()) + ".");
    }

    std::cout << "Images: " << image_paths.size() << '\n'
              << "Orientation: "
              << (config.axis == sl3d::ProjectorCoordinateAxis::kU ? "vertical -> projector u"
                                                                   : "horizontal -> projector v")
              << '\n'
              << "Phase steps: " << config.phase_steps << '\n'
              << "Frequencies: " << config.frequencies[0] << ", " << config.frequencies[1] << ", "
              << config.frequencies[2] << "\n\n";

    const auto images = LoadImages(image_paths);

    if (images.front().size() != calibration.camera_size)
    {
      throw std::runtime_error(
          "Reconstruction image size does not match "
          "camera calibration image size.");
    }

    const DecodedCoordinate decoded =
        DecodeProjectorCoordinate(images, config, calibration.projector_size);

    sl3d::TriangulationOptions triangulation_options = config.triangulation;

    triangulation_options.axis = config.axis;

    const sl3d::TriangulationResult reconstruction = sl3d::TriangulateSingleProjectorCoordinate(
        decoded.coordinate, decoded.valid_mask, calibration, triangulation_options);

    const sl3d::PointCloud::Ptr cloud =
        sl3d::ConvertTriangulationResultToPointCloud(reconstruction);

    sl3d::SavePointCloud(config.point_cloud_file, *cloud, config.point_cloud_binary);

    SaveDepthImage(config.depth_image, reconstruction, config.triangulation.min_depth,
                   config.triangulation.max_depth);

    EnsureParentDirectory(config.valid_mask);

    cv::imwrite(config.valid_mask, reconstruction.valid_mask);

    const double projector_dimension = config.axis == sl3d::ProjectorCoordinateAxis::kU
                                           ? static_cast<double>(calibration.projector_size.width)
                                           : static_cast<double>(calibration.projector_size.height);

    SaveCoordinateImage(config.projector_coordinate_image, decoded, projector_dimension);

    std::cout << "Reconstruction completed.\n"
              << "Valid 3D points: " << reconstruction.point_count << '\n'
              << "Point cloud points: " << cloud->size() << '\n'
              << "Point cloud format: " << (config.point_cloud_binary ? "binary" : "ascii") << '\n'
              << "Point cloud file: " << config.point_cloud_file << '\n'
              << "Depth image: " << config.depth_image << '\n';
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error: " << e.what() << '\n';

    return 1;
  }

  return 0;
}