#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "structured_light/camera_calibration.hpp"

namespace
{

bool Require(
    bool condition,
    const std::string& message)
{
  if (!condition)
  {
    std::cerr
        << "[FAILED] "
        << message
        << '\n';

    return false;
  }

  return true;
}

}  // namespace

int main()
{
  sl3d::CalibrationBoard board;

  board.columns = 11;
  board.rows = 9;
  board.spacing = 15.0;

  board.pattern =
      sl3d::CalibrationPattern::
          kSymmetricCircles;

  const auto object_points_single =
      sl3d::GenerateCalibrationObjectPoints(
          board);

  const cv::Size image_size(
      1280,
      1024);

  const cv::Mat camera_matrix =
      (cv::Mat_<double>(3, 3)
           << 4000.0, 0.0, 640.0,
              0.0, 4005.0, 512.0,
              0.0, 0.0, 1.0);

  const cv::Mat distortion =
      cv::Mat::zeros(
          1,
          5,
          CV_64F);

  const std::vector<cv::Vec3d>
      rotation_vectors =
  {
    {0.08, -0.05, 0.02},
    {-0.10, 0.07, -0.03},
    {0.12, 0.09, 0.04},
    {-0.07, -0.12, 0.05}
  };

  const std::vector<cv::Vec3d>
      translation_vectors =
  {
    {-70.0, -55.0, 650.0},
    {-30.0, -40.0, 700.0},
    {10.0, -60.0, 750.0},
    {-80.0, -10.0, 680.0}
  };

  std::vector<std::vector<cv::Point3f>>
      object_points;

  std::vector<std::vector<cv::Point2f>>
      image_points;

  for (std::size_t i = 0;
       i < rotation_vectors.size();
       ++i)
  {
    std::vector<cv::Point2f>
        projected_points;

    cv::projectPoints(
        object_points_single,
        rotation_vectors[i],
        translation_vectors[i],
        camera_matrix,
        distortion,
        projected_points);

    object_points.push_back(
        object_points_single);

    image_points.push_back(
        projected_points);
  }

  sl3d::CameraCalibrationOptions options;
  options.fix_k3 = true;

  const auto result =
      sl3d::CalibrateCamera(
          object_points,
          image_points,
          image_size,
          options);

  bool passed = true;

  passed &=
      Require(
          result.rms < 1e-3,
          "Synthetic calibration RMS is too large.");

  passed &=
      Require(
          result.reprojection_error < 1e-3,
          "Synthetic reprojection error is too large.");

  const double fx_error =
      std::abs(
          result.camera_matrix.at<double>(
              0,
              0) -
          camera_matrix.at<double>(
              0,
              0));

  const double fy_error =
      std::abs(
          result.camera_matrix.at<double>(
              1,
              1) -
          camera_matrix.at<double>(
              1,
              1));

  passed &=
      Require(
          fx_error < 1.0 &&
              fy_error < 1.0,
          "Recovered focal length is inaccurate.");

  if (!passed)
  {
    return 1;
  }

  std::cout
      << "[PASSED] Camera calibration test.\n"
      << "RMS: "
      << result.rms
      << '\n'
      << "Reprojection error: "
      << result.reprojection_error
      << " px\n";

  return 0;
}