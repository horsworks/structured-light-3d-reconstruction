#include "structured_light/projector_calibration.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "structured_light/phase.hpp"
#include "structured_light/phase_unwrap.hpp"

namespace sl3d
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

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

CalibrationPattern ParseCalibrationPattern(const std::string& pattern)
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

  throw std::runtime_error("Unknown calibration board pattern: " + pattern);
}

cv::Mat ConvertToGray(const cv::Mat& image)
{
  if (image.empty())
  {
    throw std::invalid_argument("Projector calibration image must not be empty.");
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
    throw std::invalid_argument("Unsupported projector calibration image format.");
  }

  return gray;
}

void ValidateFringeParameters(const ProjectorFringeParameters& params)
{
  const int f1 = params.frequencies[0];

  const int f2 = params.frequencies[1];

  const int f3 = params.frequencies[2];

  if (!(f1 > f2 && f2 > f3 && f3 > 0))
  {
    throw std::invalid_argument("Frequencies must satisfy f1 > f2 > f3 > 0.");
  }

  if (f1 - 2 * f2 + f3 != 1)
  {
    throw std::invalid_argument(
        "Current three-frequency heterodyne requires "
        "f1 - 2*f2 + f3 = 1.");
  }

  if (params.phase_steps < 3)
  {
    throw std::invalid_argument("Phase steps must be at least 3.");
  }

  if (!std::isfinite(params.modulation_threshold) || params.modulation_threshold < 0.0)
  {
    throw std::invalid_argument("Modulation threshold must be finite and non-negative.");
  }
}

bool SampleBilinear(const cv::Mat& map, const cv::Mat& valid_mask, const cv::Point2f& point,
                    double& value)
{
  const int x0 = static_cast<int>(std::floor(point.x));

  const int y0 = static_cast<int>(std::floor(point.y));

  const int x1 = x0 + 1;

  const int y1 = y0 + 1;

  if (x0 < 0 || y0 < 0 || x1 >= map.cols || y1 >= map.rows)
  {
    return false;
  }

  if (valid_mask.at<unsigned char>(y0, x0) == 0 || valid_mask.at<unsigned char>(y0, x1) == 0 ||
      valid_mask.at<unsigned char>(y1, x0) == 0 || valid_mask.at<unsigned char>(y1, x1) == 0)
  {
    return false;
  }

  const double dx = static_cast<double>(point.x) - static_cast<double>(x0);

  const double dy = static_cast<double>(point.y) - static_cast<double>(y0);

  const double value_00 = map.at<double>(y0, x0);

  const double value_01 = map.at<double>(y0, x1);

  const double value_10 = map.at<double>(y1, x0);

  const double value_11 = map.at<double>(y1, x1);

  value = (1.0 - dx) * (1.0 - dy) * value_00 + dx * (1.0 - dy) * value_01 +
          (1.0 - dx) * dy * value_10 + dx * dy * value_11;

  return std::isfinite(value);
}

void ValidateCalibrationPointSets(const std::vector<std::vector<cv::Point3f>>& object_points,
                                  const std::vector<std::vector<cv::Point2f>>& image_points)
{
  if (object_points.size() != image_points.size())
  {
    throw std::invalid_argument(
        "Object points and image points must have "
        "the same number of views.");
  }

  if (object_points.size() < 3)
  {
    throw std::invalid_argument("At least 3 valid calibration views are required.");
  }

  for (std::size_t i = 0; i < object_points.size(); ++i)
  {
    if (object_points[i].size() != image_points[i].size())
    {
      throw std::invalid_argument("Calibration points must correspond one-to-one.");
    }

    if (object_points[i].size() < 6)
    {
      throw std::invalid_argument("Each calibration view must contain at least 6 points.");
    }
  }
}

}  // namespace

ProjectorCalibrationConfig LoadProjectorCalibrationConfig(const std::string& config_path)
{
  cv::FileStorage file_storage(config_path, cv::FileStorage::READ);

  if (!file_storage.isOpened())
  {
    throw std::runtime_error("Failed to open projector calibration config: " + config_path);
  }

  ProjectorCalibrationConfig config;

  const cv::FileNode input_node = file_storage["input"];

  const cv::FileNode projector_node = file_storage["projector"];

  const cv::FileNode fringe_node = file_storage["fringe"];

  const cv::FileNode calibration_node = file_storage["calibration"];

  const cv::FileNode output_node = file_storage["output"];

  input_node["image_dir"] >> config.image_dir;

  input_node["camera_calibration_file"] >> config.camera_calibration_file;

  projector_node["width"] >> config.projector_width;

  projector_node["height"] >> config.projector_height;

  std::vector<int> frequencies;

  fringe_node["frequencies"] >> frequencies;

  if (frequencies.size() != 3)
  {
    throw std::runtime_error("Exactly three fringe frequencies are required.");
  }

  for (std::size_t i = 0; i < frequencies.size(); ++i)
  {
    config.fringe.frequencies[i] = frequencies[i];
  }

  fringe_node["phase_steps"] >> config.fringe.phase_steps;

  fringe_node["modulation_threshold"] >> config.fringe.modulation_threshold;

  config.calibration.fix_k3 = ReadBool(calibration_node, "fix_k3", config.calibration.fix_k3);

  calibration_node["min_valid_points_per_pose"] >> config.calibration.min_valid_points_per_pose;

  output_node["result_file"] >> config.result_file;

  output_node["phase_dir"] >> config.phase_dir;

  config.save_phase_debug = ReadBool(output_node, "save_phase_debug", config.save_phase_debug);

  if (config.image_dir.empty())
  {
    throw std::runtime_error("Projector calibration image directory is empty.");
  }

  if (config.camera_calibration_file.empty())
  {
    throw std::runtime_error("Camera calibration file is empty.");
  }

  if (config.projector_width <= 0 || config.projector_height <= 0)
  {
    throw std::runtime_error("Projector width and height must be positive.");
  }

  if (config.calibration.min_valid_points_per_pose < 6)
  {
    throw std::runtime_error("min_valid_points_per_pose must be at least 6.");
  }

  if (config.result_file.empty())
  {
    throw std::runtime_error("Projector calibration result file is empty.");
  }

  ValidateFringeParameters(config.fringe);

  return config;
}

CameraCalibrationParameters LoadCameraCalibrationParameters(const std::string& calibration_path)
{
  cv::FileStorage file_storage(calibration_path, cv::FileStorage::READ);

  if (!file_storage.isOpened())
  {
    throw std::runtime_error("Failed to open camera calibration file: " + calibration_path);
  }

  CameraCalibrationParameters parameters;

  file_storage["camera_matrix"] >> parameters.camera_matrix;

  file_storage["distortion_coefficients"] >> parameters.distortion_coefficients;

  int image_width = 0;
  int image_height = 0;

  file_storage["image_width"] >> image_width;

  file_storage["image_height"] >> image_height;

  parameters.image_size = cv::Size(image_width, image_height);

  file_storage["board_columns"] >> parameters.board.columns;

  file_storage["board_rows"] >> parameters.board.rows;

  file_storage["board_spacing"] >> parameters.board.spacing;

  std::string board_pattern;

  file_storage["board_pattern"] >> board_pattern;

  parameters.board.pattern = ParseCalibrationPattern(board_pattern);

  if (parameters.camera_matrix.empty() || parameters.distortion_coefficients.empty())
  {
    throw std::runtime_error(
        "Camera calibration file does not contain "
        "valid intrinsic parameters.");
  }

  if (parameters.image_size.width <= 0 || parameters.image_size.height <= 0)
  {
    throw std::runtime_error("Camera calibration file contains invalid image size.");
  }

  if (parameters.board.columns <= 1 || parameters.board.rows <= 1 ||
      parameters.board.spacing <= 0.0)
  {
    throw std::runtime_error("Camera calibration file contains invalid board parameters.");
  }

  const cv::FileNode views_node = file_storage["views"];

  if (views_node.empty() || !views_node.isSeq())
  {
    throw std::runtime_error(
        "Camera calibration file does not contain calibration views. "
        "Run camera calibration again with the updated application.");
  }

  const std::size_t expected_point_count =
      static_cast<std::size_t>(parameters.board.columns * parameters.board.rows);

  for (const auto& view_node : views_node)
  {
    CameraCalibrationView view;

    view_node["image"] >> view.image_name;

    view_node["image_points"] >> view.image_points;

    if (view.image_name.empty())
    {
      throw std::runtime_error("Camera calibration view has no image name.");
    }

    if (view.image_points.size() != expected_point_count)
    {
      throw std::runtime_error("Camera calibration view has an unexpected point count: " +
                               view.image_name);
    }

    parameters.views.emplace_back(std::move(view));
  }

  if (parameters.views.empty())
  {
    throw std::runtime_error("No camera calibration views found.");
  }

  return parameters;
}

ProjectorCoordinateResult DecodeProjectorCoordinates(const std::vector<cv::Mat>& pose_images,
                                                     int projector_width, int projector_height,
                                                     const ProjectorFringeParameters& fringe_params)
{
  ValidateFringeParameters(fringe_params);

  if (projector_width <= 0 || projector_height <= 0)
  {
    throw std::invalid_argument("Projector size must be positive.");
  }

  constexpr std::size_t kOrientationCount = 2;
  constexpr std::size_t kFrequencyCount = 3;

  const std::size_t images_per_orientation =
      kFrequencyCount * static_cast<std::size_t>(fringe_params.phase_steps);

  const std::size_t expected_image_count = kOrientationCount * images_per_orientation;

  if (pose_images.size() != expected_image_count)
  {
    throw std::invalid_argument("Incorrect number of images for one projector pose.");
  }

  std::vector<cv::Mat> gray_images;

  gray_images.reserve(pose_images.size());

  cv::Size image_size;

  for (const auto& image : pose_images)
  {
    cv::Mat gray = ConvertToGray(image);

    if (image_size.empty())
    {
      image_size = gray.size();
    }
    else if (gray.size() != image_size)
    {
      throw std::invalid_argument(
          "All projector calibration images must have "
          "the same camera image size.");
    }

    gray_images.emplace_back(std::move(gray));
  }

  std::array<PhaseUnwrapResult, 2> unwrapped_results;

  for (std::size_t orientation = 0; orientation < kOrientationCount; ++orientation)
  {
    std::array<PhaseResult, 3> phase_results;

    for (std::size_t frequency = 0; frequency < kFrequencyCount; ++frequency)
    {
      std::vector<cv::Mat> phase_images;

      phase_images.reserve(static_cast<std::size_t>(fringe_params.phase_steps));

      const std::size_t start_index =
          orientation * images_per_orientation +
          frequency * static_cast<std::size_t>(fringe_params.phase_steps);

      for (int phase = 0; phase < fringe_params.phase_steps; ++phase)
      {
        phase_images.push_back(gray_images[start_index + static_cast<std::size_t>(phase)]);
      }

      phase_results[frequency] =
          ComputeWrappedPhase(phase_images, fringe_params.modulation_threshold);
    }

    unwrapped_results[orientation] =
        UnwrapThreeFrequencyPhase(phase_results, fringe_params.frequencies);
  }

  ProjectorCoordinateResult result;

  result.projector_u = cv::Mat::zeros(image_size, CV_64FC1);

  result.projector_v = cv::Mat::zeros(image_size, CV_64FC1);

  cv::bitwise_and(unwrapped_results[0].valid_mask, unwrapped_results[1].valid_mask,
                  result.valid_mask);

  const double high_frequency = static_cast<double>(fringe_params.frequencies[0]);

  const double u_scale = static_cast<double>(projector_width) / (kTwoPi * high_frequency);

  const double v_scale = static_cast<double>(projector_height) / (kTwoPi * high_frequency);

  for (int y = 0; y < image_size.height; ++y)
  {
    const auto* phase_u_row = unwrapped_results[0].absolute_phase.ptr<double>(y);

    const auto* phase_v_row = unwrapped_results[1].absolute_phase.ptr<double>(y);

    auto* projector_u_row = result.projector_u.ptr<double>(y);

    auto* projector_v_row = result.projector_v.ptr<double>(y);

    auto* valid_row = result.valid_mask.ptr<unsigned char>(y);

    for (int x = 0; x < image_size.width; ++x)
    {
      if (valid_row[x] == 0)
      {
        continue;
      }

      const double projector_u = phase_u_row[x] * u_scale;

      const double projector_v = phase_v_row[x] * v_scale;

      if (!std::isfinite(projector_u) || !std::isfinite(projector_v) || projector_u < 0.0 ||
          projector_u >= static_cast<double>(projector_width) || projector_v < 0.0 ||
          projector_v >= static_cast<double>(projector_height))
      {
        valid_row[x] = 0;

        continue;
      }

      projector_u_row[x] = projector_u;

      projector_v_row[x] = projector_v;
    }
  }

  return result;
}

ProjectorCorrespondenceResult BuildProjectorCorrespondences(
    const ProjectorCoordinateResult& coordinate_result,
    const std::vector<cv::Point2f>& camera_points, const std::vector<cv::Point3f>& object_points)
{
  if (camera_points.size() != object_points.size())
  {
    throw std::invalid_argument("Camera points and object points must correspond one-to-one.");
  }

  if (coordinate_result.projector_u.empty() || coordinate_result.projector_v.empty() ||
      coordinate_result.valid_mask.empty())
  {
    throw std::invalid_argument("Projector coordinate maps must not be empty.");
  }

  ProjectorCorrespondenceResult result;

  result.object_points.reserve(object_points.size());

  result.camera_points.reserve(camera_points.size());

  result.projector_points.reserve(camera_points.size());

  for (std::size_t i = 0; i < camera_points.size(); ++i)
  {
    double projector_u = 0.0;
    double projector_v = 0.0;

    const bool has_u = SampleBilinear(coordinate_result.projector_u, coordinate_result.valid_mask,
                                      camera_points[i], projector_u);

    const bool has_v = SampleBilinear(coordinate_result.projector_v, coordinate_result.valid_mask,
                                      camera_points[i], projector_v);

    if (!has_u || !has_v)
    {
      continue;
    }

    result.object_points.push_back(object_points[i]);

    result.camera_points.push_back(camera_points[i]);

    result.projector_points.emplace_back(static_cast<float>(projector_u),
                                         static_cast<float>(projector_v));
  }

  return result;
}

ProjectorCalibrationResult CalibrateProjector(
    const std::vector<std::vector<cv::Point3f>>& object_points,
    const std::vector<std::vector<cv::Point2f>>& projector_points, const cv::Size& projector_size,
    const ProjectorCalibrationOptions& options)
{
  ValidateCalibrationPointSets(object_points, projector_points);

  if (projector_size.width <= 0 || projector_size.height <= 0)
  {
    throw std::invalid_argument("Invalid projector image size.");
  }

  ProjectorCalibrationResult result;

  result.projector_matrix = cv::Mat::eye(3, 3, CV_64F);

  result.distortion_coefficients = cv::Mat::zeros(1, 5, CV_64F);

  int calibration_flags = 0;

  if (options.fix_k3)
  {
    calibration_flags |= cv::CALIB_FIX_K3;
  }

  const cv::TermCriteria criteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-9);

  result.rms =
      cv::calibrateCamera(object_points, projector_points, projector_size, result.projector_matrix,
                          result.distortion_coefficients, result.rotation_vectors,
                          result.translation_vectors, calibration_flags, criteria);

  double total_squared_error = 0.0;
  std::size_t total_point_count = 0;

  result.per_view_errors.reserve(object_points.size());

  for (std::size_t i = 0; i < object_points.size(); ++i)
  {
    std::vector<cv::Point2f> projected_points;

    cv::projectPoints(object_points[i], result.rotation_vectors[i], result.translation_vectors[i],
                      result.projector_matrix, result.distortion_coefficients, projected_points);

    const double l2_error = cv::norm(projector_points[i], projected_points, cv::NORM_L2);

    const double view_error = l2_error / std::sqrt(static_cast<double>(object_points[i].size()));

    result.per_view_errors.push_back(view_error);

    total_squared_error += l2_error * l2_error;

    total_point_count += object_points[i].size();
  }

  result.reprojection_error =
      std::sqrt(total_squared_error / static_cast<double>(total_point_count));

  return result;
}

StereoCalibrationResult CalibrateCameraProjectorStereo(
    const std::vector<std::vector<cv::Point3f>>& object_points,
    const std::vector<std::vector<cv::Point2f>>& camera_points,
    const std::vector<std::vector<cv::Point2f>>& projector_points,
    const CameraCalibrationParameters& camera_parameters,
    const ProjectorCalibrationResult& projector_parameters, const cv::Size& camera_image_size)
{
  ValidateCalibrationPointSets(object_points, camera_points);

  ValidateCalibrationPointSets(object_points, projector_points);

  if (camera_points.size() != projector_points.size())
  {
    throw std::invalid_argument("Camera and projector view counts must match.");
  }

  for (std::size_t i = 0; i < camera_points.size(); ++i)
  {
    if (camera_points[i].size() != projector_points[i].size())
    {
      throw std::invalid_argument("Camera and projector points must correspond one-to-one.");
    }
  }

  cv::Mat camera_matrix = camera_parameters.camera_matrix.clone();

  cv::Mat camera_distortion = camera_parameters.distortion_coefficients.clone();

  cv::Mat projector_matrix = projector_parameters.projector_matrix.clone();

  cv::Mat projector_distortion = projector_parameters.distortion_coefficients.clone();

  StereoCalibrationResult result;

  const cv::TermCriteria criteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-9);

  result.rms = cv::stereoCalibrate(object_points, camera_points, projector_points, camera_matrix,
                                   camera_distortion, projector_matrix, projector_distortion,
                                   camera_image_size, result.rotation, result.translation,
                                   result.essential_matrix, result.fundamental_matrix,
                                   cv::CALIB_FIX_INTRINSIC, criteria);

  return result;
}

}  // namespace sl3d