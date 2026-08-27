#pragma once

#include <array>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "structured_light/camera_calibration.hpp"

namespace sl3d
{

struct ProjectorFringeParameters
{
  std::array<int, 3> frequencies = {70, 64, 59};

  int phase_steps = 4;
  double modulation_threshold = 10.0;
};

struct ProjectorCalibrationOptions
{
  bool fix_k3 = true;

  int min_valid_points_per_pose = 20;
};

struct ProjectorCalibrationConfig
{
  std::string image_dir;
  std::string camera_calibration_file;

  int projector_width = 0;
  int projector_height = 0;

  ProjectorFringeParameters fringe;
  ProjectorCalibrationOptions calibration;

  std::string result_file;
  std::string phase_dir;

  bool save_phase_debug = true;
};

struct CameraCalibrationView
{
  std::string image_name;

  std::vector<cv::Point2f> image_points;
};

struct CameraCalibrationParameters
{
  cv::Mat camera_matrix;
  cv::Mat distortion_coefficients;

  cv::Size image_size;

  CalibrationBoard board;

  std::vector<CameraCalibrationView> views;
};

struct ProjectorCoordinateResult
{
  cv::Mat projector_u;
  cv::Mat projector_v;

  cv::Mat valid_mask;
};

struct ProjectorCorrespondenceResult
{
  std::vector<cv::Point3f> object_points;

  std::vector<cv::Point2f> camera_points;
  std::vector<cv::Point2f> projector_points;
};

struct ProjectorCalibrationResult
{
  cv::Mat projector_matrix;
  cv::Mat distortion_coefficients;

  std::vector<cv::Mat> rotation_vectors;
  std::vector<cv::Mat> translation_vectors;

  double rms = 0.0;
  double reprojection_error = 0.0;

  std::vector<double> per_view_errors;
};

struct StereoCalibrationResult
{
  double rms = 0.0;

  cv::Mat rotation;
  cv::Mat translation;

  cv::Mat essential_matrix;
  cv::Mat fundamental_matrix;
};

[[nodiscard]]
ProjectorCalibrationConfig LoadProjectorCalibrationConfig(const std::string& config_path);

[[nodiscard]]
CameraCalibrationParameters LoadCameraCalibrationParameters(const std::string& calibration_path);

[[nodiscard]]
ProjectorCoordinateResult DecodeProjectorCoordinates(
    const std::vector<cv::Mat>& pose_images, int projector_width, int projector_height,
    const ProjectorFringeParameters& fringe_params);

[[nodiscard]]
ProjectorCorrespondenceResult BuildProjectorCorrespondences(
    const ProjectorCoordinateResult& coordinate_result,
    const std::vector<cv::Point2f>& camera_points, const std::vector<cv::Point3f>& object_points);

[[nodiscard]]
ProjectorCalibrationResult CalibrateProjector(
    const std::vector<std::vector<cv::Point3f>>& object_points,
    const std::vector<std::vector<cv::Point2f>>& projector_points, const cv::Size& projector_size,
    const ProjectorCalibrationOptions& options);

[[nodiscard]]
StereoCalibrationResult CalibrateCameraProjectorStereo(
    const std::vector<std::vector<cv::Point3f>>& object_points,
    const std::vector<std::vector<cv::Point2f>>& camera_points,
    const std::vector<std::vector<cv::Point2f>>& projector_points,
    const CameraCalibrationParameters& camera_parameters,
    const ProjectorCalibrationResult& projector_parameters, const cv::Size& camera_image_size);

}  // namespace sl3d