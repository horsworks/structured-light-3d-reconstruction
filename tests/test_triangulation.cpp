#include <cmath>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "structured_light/triangulation.hpp"

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

sl3d::CameraProjectorCalibration CreateCalibration()
{
  sl3d::CameraProjectorCalibration calibration;

  calibration.camera_size = cv::Size(320, 240);

  calibration.projector_size = cv::Size(640, 480);

  calibration.camera_matrix =
      (cv::Mat_<double>(3, 3) << 800.0, 0.0, 160.0, 0.0, 805.0, 120.0, 0.0, 0.0, 1.0);

  calibration.camera_distortion = cv::Mat::zeros(1, 5, CV_64F);

  calibration.projector_matrix =
      (cv::Mat_<double>(3, 3) << 1000.0, 0.0, 320.0, 0.0, 995.0, 240.0, 0.0, 0.0, 1.0);

  calibration.projector_distortion = cv::Mat::zeros(1, 5, CV_64F);

  const cv::Vec3d rotation_vector(0.01, 0.04, -0.005);

  cv::Rodrigues(rotation_vector, calibration.camera_to_projector_rotation);

  calibration.camera_to_projector_translation = (cv::Mat_<double>(3, 1) << 120.0, 5.0, 10.0);

  return calibration;
}

bool TestAxis(sl3d::ProjectorCoordinateAxis axis)
{
  const auto calibration = CreateCalibration();

  cv::Mat coordinate = cv::Mat::zeros(calibration.camera_size, CV_64FC1);

  cv::Mat valid_mask = cv::Mat::zeros(calibration.camera_size, CV_8UC1);

  std::vector<cv::Point> pixels = {{100, 80}, {130, 100}, {160, 120}, {190, 140}, {220, 160}};

  std::vector<cv::Vec3d> expected_points;

  cv::Mat rotation_vector;

  cv::Rodrigues(calibration.camera_to_projector_rotation, rotation_vector);

  for (std::size_t i = 0; i < pixels.size(); ++i)
  {
    const cv::Point pixel = pixels[i];

    const double depth = 650.0 + 40.0 * static_cast<double>(i);

    const double normalized_x =
        (static_cast<double>(pixel.x) - calibration.camera_matrix.at<double>(0, 2)) /
        calibration.camera_matrix.at<double>(0, 0);

    const double normalized_y =
        (static_cast<double>(pixel.y) - calibration.camera_matrix.at<double>(1, 2)) /
        calibration.camera_matrix.at<double>(1, 1);

    const cv::Vec3d point(depth * normalized_x, depth * normalized_y, depth);

    expected_points.push_back(point);

    std::vector<cv::Point3d> object_points = {cv::Point3d(point[0], point[1], point[2])};

    std::vector<cv::Point2d> projector_points;

    cv::projectPoints(object_points, rotation_vector, calibration.camera_to_projector_translation,
                      calibration.projector_matrix, calibration.projector_distortion,
                      projector_points);

    coordinate.at<double>(pixel.y, pixel.x) =
        axis == sl3d::ProjectorCoordinateAxis::kU ? projector_points[0].x : projector_points[0].y;

    valid_mask.at<unsigned char>(pixel.y, pixel.x) = 255;
  }

  sl3d::TriangulationOptions options;

  options.axis = axis;

  options.min_depth = 100.0;

  options.max_depth = 2000.0;

  options.max_iterations = 10;

  options.projector_residual_threshold = 1e-4;

  const auto result =
      sl3d::TriangulateSingleProjectorCoordinate(coordinate, valid_mask, calibration, options);

  bool passed = true;

  passed &=
      Require(result.point_count == pixels.size(), "Unexpected number of triangulated points.");

  for (std::size_t i = 0; i < pixels.size(); ++i)
  {
    const cv::Point pixel = pixels[i];

    const cv::Vec3d estimated = result.points_3d.at<cv::Vec3d>(pixel.y, pixel.x);

    const double error = cv::norm(estimated - expected_points[i]);

    passed &= Require(error < 1e-3, "Triangulated 3D point is inaccurate.");
  }

  return passed;
}

}  // namespace

int main()
{
  bool passed = true;

  passed &= TestAxis(sl3d::ProjectorCoordinateAxis::kU);

  passed &= TestAxis(sl3d::ProjectorCoordinateAxis::kV);

  if (!passed)
  {
    return 1;
  }

  std::cout << "[PASSED] Single-axis triangulation tests.\n";

  return 0;
}