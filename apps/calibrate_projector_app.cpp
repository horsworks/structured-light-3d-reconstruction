#include <algorithm>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "structured_light/camera_calibration.hpp"
#include "structured_light/projector_calibration.hpp"

namespace
{

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
  long long lhs_number = 0;
  long long rhs_number = 0;

  const bool lhs_numeric = ParseNumericStem(lhs, lhs_number);

  const bool rhs_numeric = ParseNumericStem(rhs, rhs_number);

  if (lhs_numeric && rhs_numeric)
  {
    return lhs_number < rhs_number;
  }

  return lhs.filename().string() < rhs.filename().string();
}

std::vector<std::filesystem::path> CollectImagePaths(const std::filesystem::path& image_dir)
{
  if (!std::filesystem::exists(image_dir))
  {
    throw std::runtime_error("Projector calibration image directory does not exist: " +
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

std::vector<cv::Mat> LoadPoseImages(const std::vector<std::filesystem::path>& image_paths,
                                    std::size_t start_index, std::size_t image_count)
{
  std::vector<cv::Mat> images;

  images.reserve(image_count);

  for (std::size_t i = 0; i < image_count; ++i)
  {
    const std::filesystem::path& image_path = image_paths[start_index + i];

    cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_UNCHANGED);

    if (image.empty())
    {
      throw std::runtime_error("Failed to read image: " + image_path.string());
    }

    images.emplace_back(std::move(image));
  }

  return images;
}

void SaveCoordinateMap(const cv::Mat& map, const cv::Mat& valid_mask, double maximum_coordinate,
                       const std::filesystem::path& output_path)
{
  cv::Mat display;

  map.convertTo(display, CV_8U, 255.0 / maximum_coordinate);

  cv::Mat invalid_mask;

  cv::bitwise_not(valid_mask, invalid_mask);

  display.setTo(0, invalid_mask);

  cv::imwrite(output_path.string(), display);
}

}  // namespace

int main(int argc, char* argv[])
{
  try
  {
    if (argc != 2)
    {
      std::cerr << "Usage:\n"
                << "  calibrate_projector <config.yaml>\n";

      return 1;
    }

    const sl3d::ProjectorCalibrationConfig config = sl3d::LoadProjectorCalibrationConfig(argv[1]);

    const sl3d::CameraCalibrationParameters camera_parameters =
        sl3d::LoadCameraCalibrationParameters(config.camera_calibration_file);

    const std::filesystem::path image_dir = config.image_dir;

    const auto image_paths = CollectImagePaths(image_dir);

    if (image_paths.empty())
    {
      throw std::runtime_error("No projector calibration images found.");
    }

    constexpr std::size_t kOrientationCount = 2;
    constexpr std::size_t kFrequencyCount = 3;

    const std::size_t images_per_pose =
        kOrientationCount * kFrequencyCount * static_cast<std::size_t>(config.fringe.phase_steps);

    if (image_paths.size() % images_per_pose != 0)
    {
      throw std::runtime_error(
          "Image count is not divisible by images_per_pose. "
          "Check image ordering, frequency count and phase steps.");
    }

    const std::size_t pose_count = image_paths.size() / images_per_pose;

    std::cout << "Total images: " << image_paths.size() << '\n'
              << "Images per pose: " << images_per_pose << '\n'
              << "Detected poses: " << pose_count << '\n'
              << "Camera calibration views: " << camera_parameters.views.size() << "\n\n";

    if (pose_count < 3)
    {
      throw std::runtime_error("At least 3 projector calibration poses are required.");
    }

    if (camera_parameters.views.size() != pose_count)
    {
      throw std::runtime_error(
          "Camera calibration view count does not match "
          "projector calibration pose count.");
    }

    if (config.save_phase_debug)
    {
      std::filesystem::create_directories(config.phase_dir);
    }

    const std::vector<cv::Point3f> board_points =
        sl3d::GenerateCalibrationObjectPoints(camera_parameters.board);

    std::vector<std::vector<cv::Point3f>> object_points;

    std::vector<std::vector<cv::Point2f>> camera_points;

    std::vector<std::vector<cv::Point2f>> projector_points;

    std::vector<int> used_poses;

    std::vector<std::string> used_camera_reference_images;

    const cv::Size camera_image_size = camera_parameters.image_size;

    for (std::size_t pose = 0; pose < pose_count; ++pose)
    {
      const std::size_t start_index = pose * images_per_pose;

      const auto pose_images = LoadPoseImages(image_paths, start_index, images_per_pose);

      if (pose_images.front().size() != camera_image_size)
      {
        throw std::runtime_error(
            "Projector calibration image size does not match "
            "camera calibration image size.");
      }

      const sl3d::ProjectorCoordinateResult coordinate_result = sl3d::DecodeProjectorCoordinates(
          pose_images, config.projector_width, config.projector_height, config.fringe);

      const auto& camera_view = camera_parameters.views[pose];

      if (camera_view.image_points.size() != board_points.size())
      {
        throw std::runtime_error(
            "Camera calibration point count does not match "
            "calibration board point count.");
      }

      const sl3d::ProjectorCorrespondenceResult correspondences =
          sl3d::BuildProjectorCorrespondences(coordinate_result, camera_view.image_points,
                                              board_points);

      const std::string pose_name = "pose_" + std::to_string(pose + 1);

      if (config.save_phase_debug)
      {
        const std::filesystem::path phase_dir = config.phase_dir;

        SaveCoordinateMap(coordinate_result.projector_u, coordinate_result.valid_mask,
                          static_cast<double>(config.projector_width),
                          phase_dir / (pose_name + "_projector_u.png"));

        SaveCoordinateMap(coordinate_result.projector_v, coordinate_result.valid_mask,
                          static_cast<double>(config.projector_height),
                          phase_dir / (pose_name + "_projector_v.png"));
      }

      if (static_cast<int>(correspondences.projector_points.size()) <
          config.calibration.min_valid_points_per_pose)
      {
        std::cout << "[SKIP] " << pose_name << " - camera reference: " << camera_view.image_name
                  << ", insufficient valid projector points: "
                  << correspondences.projector_points.size() << " / "
                  << camera_view.image_points.size() << '\n';

        continue;
      }

      object_points.push_back(correspondences.object_points);

      camera_points.push_back(correspondences.camera_points);

      projector_points.push_back(correspondences.projector_points);

      used_poses.push_back(static_cast<int>(pose + 1));

      used_camera_reference_images.push_back(camera_view.image_name);

      std::cout << "[OK]   " << pose_name << " - camera reference: " << camera_view.image_name
                << ", camera points: " << camera_view.image_points.size()
                << ", valid projector points: " << correspondences.projector_points.size() << '\n';
    }

    if (object_points.size() < 3)
    {
      throw std::runtime_error("Not enough valid projector calibration poses.");
    }

    const sl3d::ProjectorCalibrationResult projector_result = sl3d::CalibrateProjector(
        object_points, projector_points, cv::Size(config.projector_width, config.projector_height),
        config.calibration);

    const sl3d::StereoCalibrationResult stereo_result = sl3d::CalibrateCameraProjectorStereo(
        object_points, camera_points, projector_points, camera_parameters, projector_result,
        camera_image_size);

    const std::filesystem::path result_path = config.result_file;

    if (!result_path.parent_path().empty())
    {
      std::filesystem::create_directories(result_path.parent_path());
    }

    cv::FileStorage file_storage(result_path.string(), cv::FileStorage::WRITE);

    if (!file_storage.isOpened())
    {
      throw std::runtime_error("Failed to open projector calibration output file.");
    }

    std::vector<int> frequencies(config.fringe.frequencies.begin(),
                                 config.fringe.frequencies.end());

    file_storage << "projector_width" << config.projector_width;

    file_storage << "projector_height" << config.projector_height;

    file_storage << "camera_image_width" << camera_image_size.width;

    file_storage << "camera_image_height" << camera_image_size.height;

    file_storage << "frequencies" << frequencies;

    file_storage << "phase_steps" << config.fringe.phase_steps;

    file_storage << "projector_rms" << projector_result.rms;

    file_storage << "projector_reprojection_error" << projector_result.reprojection_error;

    file_storage << "projector_matrix" << projector_result.projector_matrix;

    file_storage << "projector_distortion_coefficients" << projector_result.distortion_coefficients;

    file_storage << "projector_rotation_vectors" << projector_result.rotation_vectors;

    file_storage << "projector_translation_vectors" << projector_result.translation_vectors;

    file_storage << "projector_per_view_errors" << projector_result.per_view_errors;

    file_storage << "stereo_rms" << stereo_result.rms;

    file_storage << "camera_to_projector_rotation" << stereo_result.rotation;

    file_storage << "camera_to_projector_translation" << stereo_result.translation;

    file_storage << "essential_matrix" << stereo_result.essential_matrix;

    file_storage << "fundamental_matrix" << stereo_result.fundamental_matrix;

    file_storage << "used_poses" << used_poses;

    file_storage << "camera_reference_images" << used_camera_reference_images;

    file_storage.release();

    std::cout << "\nProjector calibration completed.\n"
              << "Valid poses: " << object_points.size() << " / " << pose_count << '\n'
              << "Projector RMS: " << projector_result.rms << '\n'
              << "Projector reprojection error: " << projector_result.reprojection_error << " px\n"
              << "Stereo RMS: " << stereo_result.rms << '\n'
              << "\nProjector matrix:\n"
              << projector_result.projector_matrix << '\n'
              << "\nProjector distortion coefficients:\n"
              << projector_result.distortion_coefficients << '\n'
              << "\nCamera -> Projector rotation:\n"
              << stereo_result.rotation << '\n'
              << "\nCamera -> Projector translation:\n"
              << stereo_result.translation << '\n'
              << "\nPer-view projector errors:\n";

    for (std::size_t i = 0; i < projector_result.per_view_errors.size(); ++i)
    {
      std::cout << "  pose_" << used_poses[i] << " (" << used_camera_reference_images[i]
                << "): " << projector_result.per_view_errors[i] << " px\n";
    }

    std::cout << "\nSaved to: " << result_path.string() << '\n';
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error: " << e.what() << '\n';

    return 1;
  }

  return 0;
}