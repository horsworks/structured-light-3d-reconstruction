#pragma once

#include <cstddef>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <string>
#include <vector>

namespace sl3d
{

enum class CalibrationPattern
{
  kChessboard,
  kSymmetricCircles,
  kAsymmetricCircles
};

struct CalibrationBoard
{
  int columns = 0;
  int rows = 0;
  double spacing = 0.0;

  CalibrationPattern pattern = CalibrationPattern::kSymmetricCircles;
};

struct CircleDetectorParameters
{
  float min_threshold = 5.0F;
  float max_threshold = 250.0F;
  float threshold_step = 5.0F;

  std::size_t min_repeatability = 2;
  float min_dist_between_blobs = 5.0F;

  bool filter_by_area = true;
  float min_area = 20.0F;
  float max_area = 5000.0F;

  bool filter_by_circularity = true;
  float min_circularity = 0.35F;

  bool filter_by_convexity = true;
  float min_convexity = 0.60F;

  bool filter_by_inertia = true;
  float min_inertia_ratio = 0.10F;

  bool filter_by_color = false;
  unsigned char blob_color = 255;
};

struct CameraCalibrationOptions
{
  bool fix_k3 = true;
};

struct CameraCalibrationConfig
{
  std::string image_dir;

  CalibrationBoard board;
  CircleDetectorParameters circle_detector;
  CameraCalibrationOptions calibration;

  std::string result_file;
  std::string detection_dir;
  std::string blob_dir;

  bool save_detection_debug = true;
  bool save_blob_debug = true;
};

struct CalibrationDetectionResult
{
  bool found = false;

  std::vector<cv::Point2f> points;
  std::vector<cv::KeyPoint> blob_keypoints;
};

struct CameraCalibrationResult
{
  cv::Mat camera_matrix;
  cv::Mat distortion_coefficients;

  std::vector<cv::Mat> rotation_vectors;
  std::vector<cv::Mat> translation_vectors;

  double rms = 0.0;
  double reprojection_error = 0.0;

  std::vector<double> per_view_errors;
};

[[nodiscard]]
CameraCalibrationConfig LoadCameraCalibrationConfig(const std::string& config_path);

[[nodiscard]]
std::vector<cv::Point3f> GenerateCalibrationObjectPoints(const CalibrationBoard& board);

[[nodiscard]]
CalibrationDetectionResult DetectCalibrationPoints(const cv::Mat& image,
                                                   const CalibrationBoard& board,
                                                   const CircleDetectorParameters& detector_params);

[[nodiscard]]
CameraCalibrationResult CalibrateCamera(const std::vector<std::vector<cv::Point3f>>& object_points,
                                        const std::vector<std::vector<cv::Point2f>>& image_points,
                                        const cv::Size& image_size,
                                        const CameraCalibrationOptions& options);

}  // namespace sl3d