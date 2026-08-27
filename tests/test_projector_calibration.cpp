#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "structured_light/fringe.hpp"
#include "structured_light/projector_calibration.hpp"

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

cv::Mat VecToColumn(
    const cv::Vec3d& value)
{
  return
      (cv::Mat_<double>(3, 1)
           << value[0],
              value[1],
              value[2]);
}

}  // namespace

int main()
{
  bool passed = true;

  {
    constexpr int kWidth = 320;
    constexpr int kHeight = 240;

    const std::array<int, 3>
        frequencies =
    {
      10,
      7,
      5
    };

    constexpr int kPhaseSteps = 4;

    std::vector<cv::Mat>
        pose_images;

    for (int orientation = 0;
         orientation < 2;
         ++orientation)
    {
      for (const int frequency :
           frequencies)
      {
        sl3d::FringeParameters params;

        params.width =
            kWidth;

        params.height =
            kHeight;

        params.period =
            orientation == 0
                ? static_cast<double>(
                      kWidth) /
                      static_cast<double>(
                          frequency)
                : static_cast<double>(
                      kHeight) /
                      static_cast<double>(
                          frequency);

        params.phase_steps =
            kPhaseSteps;

        params.min_gray =
            20;

        params.max_gray =
            220;

        params.orientation =
            orientation == 0
                ? sl3d::FringeOrientation::
                      kVertical
                : sl3d::FringeOrientation::
                      kHorizontal;

        const auto fringes =
            sl3d::GenerateSinusoidalFringes(
                params);

        pose_images.insert(
            pose_images.end(),
            fringes.begin(),
            fringes.end());
      }
    }

    sl3d::ProjectorFringeParameters
        fringe_params;

    fringe_params.frequencies =
        frequencies;

    fringe_params.phase_steps =
        kPhaseSteps;

    fringe_params.modulation_threshold =
        10.0;

    const auto coordinate_result =
        sl3d::DecodeProjectorCoordinates(
            pose_images,
            kWidth,
            kHeight,
            fringe_params);

    const int sample_x = 123;
    const int sample_y = 87;

    passed &=
        Require(
            coordinate_result.valid_mask
                    .at<unsigned char>(
                        sample_y,
                        sample_x) != 0,
            "Synthetic projector coordinate should be valid.");

    const double estimated_u =
        coordinate_result.projector_u
            .at<double>(
                sample_y,
                sample_x);

    const double estimated_v =
        coordinate_result.projector_v
            .at<double>(
                sample_y,
                sample_x);

    passed &=
        Require(
            std::abs(
                estimated_u -
                static_cast<double>(
                    sample_x)) <
                0.1,
            "Decoded projector u is inaccurate.");

    passed &=
        Require(
            std::abs(
                estimated_v -
                static_cast<double>(
                    sample_y)) <
                0.1,
            "Decoded projector v is inaccurate.");
  }

  {
    const cv::Size camera_size(
        640,
        480);

    const cv::Size projector_size(
        800,
        600);

    sl3d::CalibrationBoard board;

    board.columns = 8;
    board.rows = 6;
    board.spacing = 20.0;

    board.pattern =
        sl3d::CalibrationPattern::
            kSymmetricCircles;

    const auto board_points =
        sl3d::GenerateCalibrationObjectPoints(
            board);

    const cv::Mat camera_matrix =
        (cv::Mat_<double>(3, 3)
             << 800.0, 0.0, 320.0,
                0.0, 805.0, 240.0,
                0.0, 0.0, 1.0);

    const cv::Mat projector_matrix =
        (cv::Mat_<double>(3, 3)
             << 1000.0, 0.0, 400.0,
                0.0, 995.0, 300.0,
                0.0, 0.0, 1.0);

    const cv::Mat camera_distortion =
        cv::Mat::zeros(
            1,
            5,
            CV_64F);

    const cv::Mat projector_distortion =
        cv::Mat::zeros(
            1,
            5,
            CV_64F);

    const cv::Vec3d relative_rvec(
        0.01,
        0.03,
        -0.005);

    cv::Mat relative_rotation;

    cv::Rodrigues(
        relative_rvec,
        relative_rotation);

    const cv::Mat relative_translation =
        (cv::Mat_<double>(3, 1)
             << 120.0,
                5.0,
                10.0);

    const std::vector<cv::Vec3d>
        board_rotation_vectors =
    {
      {0.08, -0.05, 0.02},
      {-0.10, 0.07, -0.03},
      {0.12, 0.09, 0.04},
      {-0.07, -0.12, 0.05},
      {0.15, -0.08, -0.06},
      {-0.12, 0.13, 0.08}
    };

    const std::vector<cv::Vec3d>
        board_translation_vectors =
    {
      {-70.0, -50.0, 700.0},
      {-30.0, -40.0, 760.0},
      {20.0, -55.0, 820.0},
      {-80.0, -10.0, 740.0},
      {10.0, 10.0, 780.0},
      {-50.0, 20.0, 850.0}
    };

    std::vector<std::vector<cv::Point3f>>
        object_points;

    std::vector<std::vector<cv::Point2f>>
        camera_points;

    std::vector<std::vector<cv::Point2f>>
        projector_points;

    for (std::size_t i = 0;
         i < board_rotation_vectors.size();
         ++i)
    {
      std::vector<cv::Point2f>
          camera_projected_points;

      cv::projectPoints(
          board_points,
          board_rotation_vectors[i],
          board_translation_vectors[i],
          camera_matrix,
          camera_distortion,
          camera_projected_points);

      cv::Mat board_to_camera_rotation;

      cv::Rodrigues(
          board_rotation_vectors[i],
          board_to_camera_rotation);

      const cv::Mat board_to_camera_translation =
          VecToColumn(
              board_translation_vectors[i]);

      const cv::Mat board_to_projector_rotation =
          relative_rotation *
          board_to_camera_rotation;

      const cv::Mat board_to_projector_translation =
          relative_rotation *
              board_to_camera_translation +
          relative_translation;

      cv::Mat board_to_projector_rvec;

      cv::Rodrigues(
          board_to_projector_rotation,
          board_to_projector_rvec);

      std::vector<cv::Point2f>
          projector_projected_points;

      cv::projectPoints(
          board_points,
          board_to_projector_rvec,
          board_to_projector_translation,
          projector_matrix,
          projector_distortion,
          projector_projected_points);

      object_points.push_back(
          board_points);

      camera_points.push_back(
          camera_projected_points);

      projector_points.push_back(
          projector_projected_points);
    }

    sl3d::ProjectorCalibrationOptions
        projector_options;

    projector_options.fix_k3 =
        true;

    const auto projector_result =
        sl3d::CalibrateProjector(
            object_points,
            projector_points,
            projector_size,
            projector_options);

    passed &=
        Require(
            projector_result.rms <
                1e-3,
            "Synthetic projector calibration RMS is too large.");

    const double projector_fx_error =
        std::abs(
            projector_result
                    .projector_matrix
                    .at<double>(0, 0) -
            projector_matrix
                .at<double>(0, 0));

    const double projector_fy_error =
        std::abs(
            projector_result
                    .projector_matrix
                    .at<double>(1, 1) -
            projector_matrix
                .at<double>(1, 1));

    passed &=
        Require(
            projector_fx_error < 1.0 &&
                projector_fy_error < 1.0,
            "Recovered projector focal length is inaccurate.");

    sl3d::CameraCalibrationParameters
        camera_parameters;

    camera_parameters.camera_matrix =
        camera_matrix.clone();

    camera_parameters.distortion_coefficients =
        camera_distortion.clone();

    camera_parameters.image_size =
        camera_size;

    sl3d::ProjectorCalibrationResult
        known_projector_parameters;

    known_projector_parameters.projector_matrix =
        projector_matrix.clone();

    known_projector_parameters.distortion_coefficients =
        projector_distortion.clone();

    const auto stereo_result =
        sl3d::CalibrateCameraProjectorStereo(
            object_points,
            camera_points,
            projector_points,
            camera_parameters,
            known_projector_parameters,
            camera_size);

    passed &=
        Require(
            stereo_result.rms <
                1e-3,
            "Synthetic stereo calibration RMS is too large.");

    const double rotation_error =
        cv::norm(
            stereo_result.rotation -
            relative_rotation);

    const double translation_error =
        cv::norm(
            stereo_result.translation -
            relative_translation);

    passed &=
        Require(
            rotation_error <
                1e-3,
            "Recovered camera-projector rotation is inaccurate.");

    passed &=
        Require(
            translation_error <
                0.1,
            "Recovered camera-projector translation is inaccurate.");
  }

  if (!passed)
  {
    return 1;
  }

  std::cout
      << "[PASSED] Projector calibration tests.\n";

  return 0;
}