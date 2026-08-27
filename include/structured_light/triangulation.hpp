#pragma once

#include <cstddef>
#include <opencv2/core.hpp>
#include <string>

namespace sl3d
{

enum class ProjectorCoordinateAxis
{
  kU,
  kV
};

struct CameraProjectorCalibration
{
  cv::Mat camera_matrix;
  cv::Mat camera_distortion;

  cv::Mat projector_matrix;
  cv::Mat projector_distortion;

  cv::Mat camera_to_projector_rotation;
  cv::Mat camera_to_projector_translation;

  cv::Size camera_size;
  cv::Size projector_size;
};

struct TriangulationOptions
{
  ProjectorCoordinateAxis axis = ProjectorCoordinateAxis::kU;

  double min_depth = 100.0;
  double max_depth = 3000.0;

  int max_iterations = 8;

  double projector_residual_threshold = 0.25;

  int pixel_step = 1;
};

struct TriangulationResult
{
  cv::Mat points_3d;
  cv::Mat depth;

  cv::Mat valid_mask;
  cv::Mat projector_residual;

  std::size_t point_count = 0;
};

[[nodiscard]]
CameraProjectorCalibration LoadCameraProjectorCalibration(
    const std::string& camera_calibration_file, const std::string& projector_calibration_file);

[[nodiscard]]
TriangulationResult TriangulateSingleProjectorCoordinate(
    const cv::Mat& projector_coordinate, const cv::Mat& phase_valid_mask,
    const CameraProjectorCalibration& calibration, const TriangulationOptions& options);

}  // namespace sl3d