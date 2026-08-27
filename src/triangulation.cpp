#include "structured_light/triangulation.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <stdexcept>
#include <vector>

namespace sl3d
{
namespace
{

constexpr double kEpsilon = 1e-12;

struct ProjectorModel
{
  double fx = 0.0;
  double fy = 0.0;
  double skew = 0.0;

  double cx = 0.0;
  double cy = 0.0;

  double k1 = 0.0;
  double k2 = 0.0;

  double p1 = 0.0;
  double p2 = 0.0;

  double k3 = 0.0;

  cv::Matx33d rotation = cv::Matx33d::eye();

  cv::Vec3d translation = cv::Vec3d::all(0.0);
};

double GetDistortionCoefficient(const cv::Mat& distortion, std::size_t index)
{
  if (distortion.empty() || index >= distortion.total())
  {
    return 0.0;
  }

  cv::Mat distortion_64f;

  distortion.convertTo(distortion_64f, CV_64F);

  const cv::Mat flattened = distortion_64f.reshape(1, 1);

  return flattened.at<double>(0, static_cast<int>(index));
}

ProjectorModel BuildProjectorModel(const CameraProjectorCalibration& calibration)
{
  cv::Mat projector_matrix;
  cv::Mat rotation;
  cv::Mat translation;

  calibration.projector_matrix.convertTo(projector_matrix, CV_64F);

  calibration.camera_to_projector_rotation.convertTo(rotation, CV_64F);

  calibration.camera_to_projector_translation.convertTo(translation, CV_64F);

  if (projector_matrix.rows != 3 || projector_matrix.cols != 3)
  {
    throw std::invalid_argument("Projector matrix must be 3x3.");
  }

  if (rotation.rows != 3 || rotation.cols != 3)
  {
    throw std::invalid_argument("Camera-to-projector rotation must be 3x3.");
  }

  if (translation.total() != 3)
  {
    throw std::invalid_argument("Camera-to-projector translation must contain 3 values.");
  }

  translation = translation.reshape(1, 3);

  ProjectorModel model;

  model.fx = projector_matrix.at<double>(0, 0);

  model.skew = projector_matrix.at<double>(0, 1);

  model.cx = projector_matrix.at<double>(0, 2);

  model.fy = projector_matrix.at<double>(1, 1);

  model.cy = projector_matrix.at<double>(1, 2);

  model.k1 = GetDistortionCoefficient(calibration.projector_distortion, 0);

  model.k2 = GetDistortionCoefficient(calibration.projector_distortion, 1);

  model.p1 = GetDistortionCoefficient(calibration.projector_distortion, 2);

  model.p2 = GetDistortionCoefficient(calibration.projector_distortion, 3);

  model.k3 = GetDistortionCoefficient(calibration.projector_distortion, 4);

  for (int row = 0; row < 3; ++row)
  {
    for (int column = 0; column < 3; ++column)
    {
      model.rotation(row, column) = rotation.at<double>(row, column);
    }
  }

  model.translation = cv::Vec3d(translation.at<double>(0, 0), translation.at<double>(1, 0),
                                translation.at<double>(2, 0));

  return model;
}

bool ProjectAxisAndDerivative(const ProjectorModel& model, const cv::Vec3d& transformed_ray,
                              double depth, ProjectorCoordinateAxis axis, double& coordinate,
                              double& derivative)
{
  const double x_projector = depth * transformed_ray[0] + model.translation[0];

  const double y_projector = depth * transformed_ray[1] + model.translation[1];

  const double z_projector = depth * transformed_ray[2] + model.translation[2];

  if (z_projector <= kEpsilon)
  {
    return false;
  }

  const double inverse_z = 1.0 / z_projector;

  const double x = x_projector * inverse_z;

  const double y = y_projector * inverse_z;

  const double inverse_z_squared = inverse_z * inverse_z;

  const double dx =
      (transformed_ray[0] * z_projector - x_projector * transformed_ray[2]) * inverse_z_squared;

  const double dy =
      (transformed_ray[1] * z_projector - y_projector * transformed_ray[2]) * inverse_z_squared;

  const double radius_squared = x * x + y * y;

  const double radius_fourth = radius_squared * radius_squared;

  const double radius_sixth = radius_fourth * radius_squared;

  const double radial =
      1.0 + model.k1 * radius_squared + model.k2 * radius_fourth + model.k3 * radius_sixth;

  const double distorted_x =
      x * radial + 2.0 * model.p1 * x * y + model.p2 * (radius_squared + 2.0 * x * x);

  const double distorted_y =
      y * radial + model.p1 * (radius_squared + 2.0 * y * y) + 2.0 * model.p2 * x * y;

  const double radius_squared_derivative = 2.0 * (x * dx + y * dy);

  const double radial_derivative =
      (model.k1 + 2.0 * model.k2 * radius_squared + 3.0 * model.k3 * radius_fourth) *
      radius_squared_derivative;

  const double distorted_x_derivative = dx * radial + x * radial_derivative +
                                        2.0 * model.p1 * (dx * y + x * dy) +
                                        model.p2 * (radius_squared_derivative + 4.0 * x * dx);

  const double distorted_y_derivative = dy * radial + y * radial_derivative +
                                        model.p1 * (radius_squared_derivative + 4.0 * y * dy) +
                                        2.0 * model.p2 * (dx * y + x * dy);

  const double projector_u = model.fx * distorted_x + model.skew * distorted_y + model.cx;

  const double projector_v = model.fy * distorted_y + model.cy;

  const double projector_u_derivative =
      model.fx * distorted_x_derivative + model.skew * distorted_y_derivative;

  const double projector_v_derivative = model.fy * distorted_y_derivative;

  if (axis == ProjectorCoordinateAxis::kU)
  {
    coordinate = projector_u;

    derivative = projector_u_derivative;
  }
  else
  {
    coordinate = projector_v;

    derivative = projector_v_derivative;
  }

  return std::isfinite(coordinate) && std::isfinite(derivative);
}

double EstimateInitialDepth(const ProjectorModel& model, const cv::Vec3d& transformed_ray,
                            double observed_coordinate, ProjectorCoordinateAxis axis,
                            double min_depth, double max_depth)
{
  double normalized_coordinate = 0.0;

  double denominator = 0.0;
  double numerator = 0.0;

  if (axis == ProjectorCoordinateAxis::kU)
  {
    normalized_coordinate = (observed_coordinate - model.cx) / model.fx;

    denominator = transformed_ray[0] - normalized_coordinate * transformed_ray[2];

    numerator = normalized_coordinate * model.translation[2] - model.translation[0];
  }
  else
  {
    normalized_coordinate = (observed_coordinate - model.cy) / model.fy;

    denominator = transformed_ray[1] - normalized_coordinate * transformed_ray[2];

    numerator = normalized_coordinate * model.translation[2] - model.translation[1];
  }

  const double midpoint = 0.5 * (min_depth + max_depth);

  if (std::abs(denominator) < kEpsilon)
  {
    return midpoint;
  }

  const double depth = numerator / denominator;

  if (!std::isfinite(depth))
  {
    return midpoint;
  }

  return std::clamp(depth, min_depth, max_depth);
}

void ValidateCalibration(const CameraProjectorCalibration& calibration)
{
  if (calibration.camera_matrix.empty() || calibration.projector_matrix.empty() ||
      calibration.camera_to_projector_rotation.empty() ||
      calibration.camera_to_projector_translation.empty())
  {
    throw std::invalid_argument("Camera-projector calibration is incomplete.");
  }

  if (calibration.camera_size.width <= 0 || calibration.camera_size.height <= 0)
  {
    throw std::invalid_argument("Invalid camera image size.");
  }

  if (calibration.projector_size.width <= 0 || calibration.projector_size.height <= 0)
  {
    throw std::invalid_argument("Invalid projector image size.");
  }
}

void ValidateOptions(const TriangulationOptions& options)
{
  if (!std::isfinite(options.min_depth) || !std::isfinite(options.max_depth) ||
      options.min_depth <= 0.0 || options.max_depth <= options.min_depth)
  {
    throw std::invalid_argument("Invalid triangulation depth range.");
  }

  if (options.max_iterations <= 0)
  {
    throw std::invalid_argument("max_iterations must be positive.");
  }

  if (!std::isfinite(options.projector_residual_threshold) ||
      options.projector_residual_threshold <= 0.0)
  {
    throw std::invalid_argument("Projector residual threshold must be positive.");
  }

  if (options.pixel_step <= 0)
  {
    throw std::invalid_argument("pixel_step must be positive.");
  }
}

}  // namespace

CameraProjectorCalibration LoadCameraProjectorCalibration(
    const std::string& camera_calibration_file, const std::string& projector_calibration_file)
{
  CameraProjectorCalibration calibration;

  {
    cv::FileStorage file_storage(camera_calibration_file, cv::FileStorage::READ);

    if (!file_storage.isOpened())
    {
      throw std::runtime_error("Failed to open camera calibration file: " +
                               camera_calibration_file);
    }

    file_storage["camera_matrix"] >> calibration.camera_matrix;

    file_storage["distortion_coefficients"] >> calibration.camera_distortion;

    int image_width = 0;
    int image_height = 0;

    file_storage["image_width"] >> image_width;

    file_storage["image_height"] >> image_height;

    calibration.camera_size = cv::Size(image_width, image_height);
  }

  {
    cv::FileStorage file_storage(projector_calibration_file, cv::FileStorage::READ);

    if (!file_storage.isOpened())
    {
      throw std::runtime_error("Failed to open projector calibration file: " +
                               projector_calibration_file);
    }

    file_storage["projector_matrix"] >> calibration.projector_matrix;

    file_storage["projector_distortion_coefficients"] >> calibration.projector_distortion;

    file_storage["camera_to_projector_rotation"] >> calibration.camera_to_projector_rotation;

    file_storage["camera_to_projector_translation"] >> calibration.camera_to_projector_translation;

    int projector_width = 0;
    int projector_height = 0;

    file_storage["projector_width"] >> projector_width;

    file_storage["projector_height"] >> projector_height;

    calibration.projector_size = cv::Size(projector_width, projector_height);
  }

  calibration.camera_matrix.convertTo(calibration.camera_matrix, CV_64F);

  calibration.camera_distortion.convertTo(calibration.camera_distortion, CV_64F);

  calibration.projector_matrix.convertTo(calibration.projector_matrix, CV_64F);

  calibration.projector_distortion.convertTo(calibration.projector_distortion, CV_64F);

  calibration.camera_to_projector_rotation.convertTo(calibration.camera_to_projector_rotation,
                                                     CV_64F);

  calibration.camera_to_projector_translation.convertTo(calibration.camera_to_projector_translation,
                                                        CV_64F);

  ValidateCalibration(calibration);

  return calibration;
}

TriangulationResult TriangulateSingleProjectorCoordinate(
    const cv::Mat& projector_coordinate, const cv::Mat& phase_valid_mask,
    const CameraProjectorCalibration& calibration, const TriangulationOptions& options)
{
  ValidateCalibration(calibration);

  ValidateOptions(options);

  if (projector_coordinate.empty() || phase_valid_mask.empty())
  {
    throw std::invalid_argument("Projector coordinate map and valid mask must not be empty.");
  }

  if (projector_coordinate.size() != phase_valid_mask.size())
  {
    throw std::invalid_argument("Projector coordinate map and valid mask sizes must match.");
  }

  if (phase_valid_mask.type() != CV_8UC1)
  {
    throw std::invalid_argument("Phase valid mask must be CV_8UC1.");
  }

  if (projector_coordinate.size() != calibration.camera_size)
  {
    throw std::invalid_argument("Projector coordinate map size does not match camera calibration.");
  }

  cv::Mat coordinate_64f;

  projector_coordinate.convertTo(coordinate_64f, CV_64F);

  TriangulationResult result;

  result.points_3d = cv::Mat::zeros(projector_coordinate.size(), CV_64FC3);

  result.depth = cv::Mat::zeros(projector_coordinate.size(), CV_64FC1);

  result.valid_mask = cv::Mat::zeros(projector_coordinate.size(), CV_8UC1);

  result.projector_residual = cv::Mat::zeros(projector_coordinate.size(), CV_64FC1);

  std::vector<cv::Point2f> camera_pixels;

  std::vector<cv::Point> pixel_indices;

  for (int y = 0; y < projector_coordinate.rows; y += options.pixel_step)
  {
    for (int x = 0; x < projector_coordinate.cols; x += options.pixel_step)
    {
      if (phase_valid_mask.at<unsigned char>(y, x) == 0)
      {
        continue;
      }

      const double coordinate = coordinate_64f.at<double>(y, x);

      if (!std::isfinite(coordinate))
      {
        continue;
      }

      camera_pixels.emplace_back(static_cast<float>(x), static_cast<float>(y));

      pixel_indices.emplace_back(x, y);
    }
  }

  if (camera_pixels.empty())
  {
    return result;
  }

  std::vector<cv::Point2f> normalized_camera_points;

  cv::undistortPoints(camera_pixels, normalized_camera_points, calibration.camera_matrix,
                      calibration.camera_distortion);

  const ProjectorModel projector_model = BuildProjectorModel(calibration);

  const double maximum_newton_step = 0.25 * (options.max_depth - options.min_depth);

  for (std::size_t i = 0; i < normalized_camera_points.size(); ++i)
  {
    const cv::Point pixel = pixel_indices[i];

    const double observed_coordinate = coordinate_64f.at<double>(pixel.y, pixel.x);

    const cv::Vec3d camera_ray(static_cast<double>(normalized_camera_points[i].x),
                               static_cast<double>(normalized_camera_points[i].y), 1.0);

    const cv::Vec3d transformed_ray = projector_model.rotation * camera_ray;

    double depth = EstimateInitialDepth(projector_model, transformed_ray, observed_coordinate,
                                        options.axis, options.min_depth, options.max_depth);

    bool iteration_valid = true;

    for (int iteration = 0; iteration < options.max_iterations; ++iteration)
    {
      double predicted_coordinate = 0.0;
      double derivative = 0.0;

      if (!ProjectAxisAndDerivative(projector_model, transformed_ray, depth, options.axis,
                                    predicted_coordinate, derivative))
      {
        iteration_valid = false;

        break;
      }

      const double residual = predicted_coordinate - observed_coordinate;

      if (std::abs(residual) < 1e-7)
      {
        break;
      }

      if (std::abs(derivative) < kEpsilon)
      {
        iteration_valid = false;

        break;
      }

      double step = residual / derivative;

      step = std::clamp(step, -maximum_newton_step, maximum_newton_step);

      depth = std::clamp(depth - step, options.min_depth, options.max_depth);
    }

    if (!iteration_valid)
    {
      continue;
    }

    double final_coordinate = 0.0;
    double final_derivative = 0.0;

    if (!ProjectAxisAndDerivative(projector_model, transformed_ray, depth, options.axis,
                                  final_coordinate, final_derivative))
    {
      continue;
    }

    const double residual = std::abs(final_coordinate - observed_coordinate);

    if (residual > options.projector_residual_threshold)
    {
      continue;
    }

    if (depth <= options.min_depth || depth >= options.max_depth)
    {
      continue;
    }

    const cv::Vec3d point_3d = depth * camera_ray;

    result.points_3d.at<cv::Vec3d>(pixel.y, pixel.x) = point_3d;

    result.depth.at<double>(pixel.y, pixel.x) = point_3d[2];

    result.valid_mask.at<unsigned char>(pixel.y, pixel.x) = 255;

    result.projector_residual.at<double>(pixel.y, pixel.x) = residual;

    ++result.point_count;
  }

  return result;
}

}  // namespace sl3d