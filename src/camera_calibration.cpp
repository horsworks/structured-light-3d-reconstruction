#include "structured_light/camera_calibration.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace sl3d
{
namespace
{

void ValidateBoard(const CalibrationBoard& board)
{
  if (board.columns <= 1 || board.rows <= 1)
  {
    throw std::invalid_argument("Calibration board rows and columns must be greater than 1.");
  }

  if (!std::isfinite(board.spacing) || board.spacing <= 0.0)
  {
    throw std::invalid_argument("Calibration board spacing must be positive and finite.");
  }
}

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

CalibrationPattern ParsePattern(const std::string& pattern)
{
  if (pattern == "chessboard")
  {
    return CalibrationPattern::kChessboard;
  }

  if (pattern == "circles")
  {
    return CalibrationPattern::kSymmetricCircles;
  }

  if (pattern == "asymmetric_circles")
  {
    return CalibrationPattern::kAsymmetricCircles;
  }

  throw std::invalid_argument("Unknown calibration pattern: " + pattern);
}

cv::Mat ConvertToGray(const cv::Mat& image)
{
  if (image.empty())
  {
    throw std::invalid_argument("Calibration image must not be empty.");
  }

  if (image.channels() == 1)
  {
    return image.clone();
  }

  cv::Mat gray;

  if (image.channels() == 3)
  {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  }
  else if (image.channels() == 4)
  {
    cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
  }
  else
  {
    throw std::invalid_argument("Unsupported calibration image format.");
  }

  return gray;
}

cv::Ptr<cv::SimpleBlobDetector> CreateCircleDetector(
    const CircleDetectorParameters& detector_params)
{
  cv::SimpleBlobDetector::Params params;

  params.minThreshold = detector_params.min_threshold;

  params.maxThreshold = detector_params.max_threshold;

  params.thresholdStep = detector_params.threshold_step;

  params.minRepeatability = detector_params.min_repeatability;

  params.minDistBetweenBlobs = detector_params.min_dist_between_blobs;

  params.filterByArea = detector_params.filter_by_area;

  params.minArea = detector_params.min_area;

  params.maxArea = detector_params.max_area;

  params.filterByCircularity = detector_params.filter_by_circularity;

  params.minCircularity = detector_params.min_circularity;

  params.filterByConvexity = detector_params.filter_by_convexity;

  params.minConvexity = detector_params.min_convexity;

  params.filterByInertia = detector_params.filter_by_inertia;

  params.minInertiaRatio = detector_params.min_inertia_ratio;

  params.filterByColor = detector_params.filter_by_color;

  params.blobColor = detector_params.blob_color;

  return cv::SimpleBlobDetector::create(params);
}

}  // namespace

CameraCalibrationConfig LoadCameraCalibrationConfig(const std::string& config_path)
{
  cv::FileStorage file_storage(config_path, cv::FileStorage::READ);

  if (!file_storage.isOpened())
  {
    throw std::runtime_error("Failed to open camera calibration config: " + config_path);
  }

  CameraCalibrationConfig config;

  const cv::FileNode input_node = file_storage["input"];

  const cv::FileNode board_node = file_storage["board"];

  const cv::FileNode detector_node = file_storage["circle_detector"];

  const cv::FileNode calibration_node = file_storage["calibration"];

  const cv::FileNode output_node = file_storage["output"];

  input_node["image_dir"] >> config.image_dir;

  std::string pattern;

  board_node["pattern"] >> pattern;

  board_node["columns"] >> config.board.columns;

  board_node["rows"] >> config.board.rows;

  board_node["spacing"] >> config.board.spacing;

  config.board.pattern = ParsePattern(pattern);

  detector_node["min_threshold"] >> config.circle_detector.min_threshold;

  detector_node["max_threshold"] >> config.circle_detector.max_threshold;

  detector_node["threshold_step"] >> config.circle_detector.threshold_step;

  int min_repeatability = static_cast<int>(config.circle_detector.min_repeatability);

  detector_node["min_repeatability"] >> min_repeatability;

  config.circle_detector.min_repeatability =
      static_cast<std::size_t>(std::max(min_repeatability, 1));

  detector_node["min_dist_between_blobs"] >> config.circle_detector.min_dist_between_blobs;

  config.circle_detector.filter_by_area =
      ReadBool(detector_node, "filter_by_area", config.circle_detector.filter_by_area);

  detector_node["min_area"] >> config.circle_detector.min_area;

  detector_node["max_area"] >> config.circle_detector.max_area;

  config.circle_detector.filter_by_circularity = ReadBool(
      detector_node, "filter_by_circularity", config.circle_detector.filter_by_circularity);

  detector_node["min_circularity"] >> config.circle_detector.min_circularity;

  config.circle_detector.filter_by_convexity =
      ReadBool(detector_node, "filter_by_convexity", config.circle_detector.filter_by_convexity);

  detector_node["min_convexity"] >> config.circle_detector.min_convexity;

  config.circle_detector.filter_by_inertia =
      ReadBool(detector_node, "filter_by_inertia", config.circle_detector.filter_by_inertia);

  detector_node["min_inertia_ratio"] >> config.circle_detector.min_inertia_ratio;

  config.circle_detector.filter_by_color =
      ReadBool(detector_node, "filter_by_color", config.circle_detector.filter_by_color);

  int blob_color = static_cast<int>(config.circle_detector.blob_color);

  detector_node["blob_color"] >> blob_color;

  blob_color = std::clamp(blob_color, 0, 255);

  config.circle_detector.blob_color = static_cast<unsigned char>(blob_color);

  config.calibration.fix_k3 = ReadBool(calibration_node, "fix_k3", config.calibration.fix_k3);

  output_node["result_file"] >> config.result_file;

  output_node["detection_dir"] >> config.detection_dir;

  output_node["blob_dir"] >> config.blob_dir;

  config.save_detection_debug =
      ReadBool(output_node, "save_detection_debug", config.save_detection_debug);

  config.save_blob_debug = ReadBool(output_node, "save_blob_debug", config.save_blob_debug);

  ValidateBoard(config.board);

  if (config.image_dir.empty())
  {
    throw std::runtime_error("Camera calibration image directory is empty.");
  }

  if (config.result_file.empty())
  {
    throw std::runtime_error("Camera calibration result file is empty.");
  }

  return config;
}

std::vector<cv::Point3f> GenerateCalibrationObjectPoints(const CalibrationBoard& board)
{
  ValidateBoard(board);

  std::vector<cv::Point3f> points;

  points.reserve(static_cast<std::size_t>(board.columns * board.rows));

  for (int row = 0; row < board.rows; ++row)
  {
    for (int column = 0; column < board.columns; ++column)
    {
      double x = 0.0;
      double y = 0.0;

      if (board.pattern == CalibrationPattern::kAsymmetricCircles)
      {
        x = (2.0 * static_cast<double>(column) + static_cast<double>(row % 2)) * board.spacing;

        y = static_cast<double>(row) * board.spacing;
      }
      else
      {
        x = static_cast<double>(column) * board.spacing;

        y = static_cast<double>(row) * board.spacing;
      }

      points.emplace_back(static_cast<float>(x), static_cast<float>(y), 0.0F);
    }
  }

  return points;
}

CalibrationDetectionResult DetectCalibrationPoints(const cv::Mat& image,
                                                   const CalibrationBoard& board,
                                                   const CircleDetectorParameters& detector_params)
{
  ValidateBoard(board);

  const cv::Mat gray = ConvertToGray(image);

  CalibrationDetectionResult result;

  const cv::Size pattern_size(board.columns, board.rows);

  if (board.pattern == CalibrationPattern::kChessboard)
  {
    result.found =
        cv::findChessboardCorners(gray, pattern_size, result.points,
                                  cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

    if (result.found)
    {
      cv::cornerSubPix(
          gray, result.points, cv::Size(5, 5), cv::Size(-1, -1),
          cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001));
    }

    return result;
  }

  const auto detector = CreateCircleDetector(detector_params);

  detector->detect(gray, result.blob_keypoints);

  int flags = cv::CALIB_CB_CLUSTERING;

  if (board.pattern == CalibrationPattern::kSymmetricCircles)
  {
    flags |= cv::CALIB_CB_SYMMETRIC_GRID;
  }
  else
  {
    flags |= cv::CALIB_CB_ASYMMETRIC_GRID;
  }

  result.found = cv::findCirclesGrid(gray, pattern_size, result.points, flags, detector);

  if (result.found)
  {
    return result;
  }

  cv::Mat inverted_gray;

  cv::bitwise_not(gray, inverted_gray);

  std::vector<cv::KeyPoint> inverted_keypoints;

  detector->detect(inverted_gray, inverted_keypoints);

  std::vector<cv::Point2f> inverted_points;

  const bool inverted_found =
      cv::findCirclesGrid(inverted_gray, pattern_size, inverted_points, flags, detector);

  if (inverted_found)
  {
    result.found = true;

    result.points = std::move(inverted_points);

    result.blob_keypoints = std::move(inverted_keypoints);

    return result;
  }

  const std::size_t expected_count = static_cast<std::size_t>(board.columns * board.rows);

  const auto original_difference = result.blob_keypoints.size() > expected_count
                                       ? result.blob_keypoints.size() - expected_count
                                       : expected_count - result.blob_keypoints.size();

  const auto inverted_difference = inverted_keypoints.size() > expected_count
                                       ? inverted_keypoints.size() - expected_count
                                       : expected_count - inverted_keypoints.size();

  if (inverted_difference < original_difference)
  {
    result.blob_keypoints = std::move(inverted_keypoints);
  }

  return result;
}

CameraCalibrationResult CalibrateCamera(const std::vector<std::vector<cv::Point3f>>& object_points,
                                        const std::vector<std::vector<cv::Point2f>>& image_points,
                                        const cv::Size& image_size,
                                        const CameraCalibrationOptions& options)
{
  if (object_points.size() != image_points.size())
  {
    throw std::invalid_argument(
        "Object points and image points must have the same number of views.");
  }

  if (object_points.size() < 3)
  {
    throw std::invalid_argument("At least 3 valid calibration views are required.");
  }

  if (image_size.width <= 0 || image_size.height <= 0)
  {
    throw std::invalid_argument("Invalid calibration image size.");
  }

  for (std::size_t i = 0; i < object_points.size(); ++i)
  {
    if (object_points[i].empty() || image_points[i].empty())
    {
      throw std::invalid_argument("Calibration point sets must not be empty.");
    }

    if (object_points[i].size() != image_points[i].size())
    {
      throw std::invalid_argument("Object points and image points must correspond one-to-one.");
    }
  }

  CameraCalibrationResult result;

  result.camera_matrix = cv::Mat::eye(3, 3, CV_64F);

  result.distortion_coefficients = cv::Mat::zeros(1, 5, CV_64F);

  int calibration_flags = 0;

  if (options.fix_k3)
  {
    calibration_flags |= cv::CALIB_FIX_K3;
  }

  const cv::TermCriteria criteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-9);

  result.rms = cv::calibrateCamera(object_points, image_points, image_size, result.camera_matrix,
                                   result.distortion_coefficients, result.rotation_vectors,
                                   result.translation_vectors, calibration_flags, criteria);

  double total_squared_error = 0.0;
  std::size_t total_point_count = 0;

  result.per_view_errors.reserve(object_points.size());

  for (std::size_t i = 0; i < object_points.size(); ++i)
  {
    std::vector<cv::Point2f> projected_points;

    cv::projectPoints(object_points[i], result.rotation_vectors[i], result.translation_vectors[i],
                      result.camera_matrix, result.distortion_coefficients, projected_points);

    const double l2_error = cv::norm(image_points[i], projected_points, cv::NORM_L2);

    const double view_error = l2_error / std::sqrt(static_cast<double>(object_points[i].size()));

    result.per_view_errors.push_back(view_error);

    total_squared_error += l2_error * l2_error;

    total_point_count += object_points[i].size();
  }

  result.reprojection_error =
      std::sqrt(total_squared_error / static_cast<double>(total_point_count));

  return result;
}

}  // namespace sl3d