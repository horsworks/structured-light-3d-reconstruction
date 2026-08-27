#include <algorithm>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "structured_light/camera_calibration.hpp"

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

std::string PatternToString(sl3d::CalibrationPattern pattern)
{
  switch (pattern)
  {
    case sl3d::CalibrationPattern::kChessboard:
      return "chessboard";

    case sl3d::CalibrationPattern::kSymmetricCircles:
      return "circles";

    case sl3d::CalibrationPattern::kAsymmetricCircles:
      return "asymmetric_circles";
  }

  throw std::runtime_error("Unknown calibration pattern.");
}

}  // namespace

int main(int argc, char* argv[])
{
  try
  {
    if (argc != 2)
    {
      std::cerr << "Usage:\n"
                << "  calibrate_camera <config.yaml>\n";

      return 1;
    }

    const sl3d::CameraCalibrationConfig config = sl3d::LoadCameraCalibrationConfig(argv[1]);

    const std::filesystem::path image_dir = config.image_dir;

    if (!std::filesystem::exists(image_dir))
    {
      throw std::runtime_error("Image directory does not exist: " + image_dir.string());
    }

    std::vector<std::filesystem::path> image_paths;

    for (const auto& entry : std::filesystem::directory_iterator(image_dir))
    {
      if (entry.is_regular_file() && IsImageFile(entry.path()))
      {
        image_paths.push_back(entry.path());
      }
    }

    std::sort(image_paths.begin(), image_paths.end());

    if (image_paths.empty())
    {
      throw std::runtime_error("No calibration images found.");
    }

    const std::vector<cv::Point3f> board_points =
        sl3d::GenerateCalibrationObjectPoints(config.board);

    std::vector<std::vector<cv::Point3f>> object_points;

    std::vector<std::vector<cv::Point2f>> image_points;

    std::vector<std::string> used_images;

    cv::Size image_size;

    if (config.save_detection_debug)
    {
      std::filesystem::create_directories(config.detection_dir);
    }

    if (config.save_blob_debug)
    {
      std::filesystem::create_directories(config.blob_dir);
    }

    for (const auto& image_path : image_paths)
    {
      const cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_COLOR);

      if (image.empty())
      {
        std::cout << "[SKIP] " << image_path.filename().string() << " - failed to read\n";

        continue;
      }

      if (image_size.empty())
      {
        image_size = image.size();
      }
      else if (image.size() != image_size)
      {
        std::cout << "[SKIP] " << image_path.filename().string() << " - inconsistent image size\n";

        continue;
      }

      const sl3d::CalibrationDetectionResult detection =
          sl3d::DetectCalibrationPoints(image, config.board, config.circle_detector);

      if (config.save_blob_debug && config.board.pattern != sl3d::CalibrationPattern::kChessboard)
      {
        cv::Mat blob_preview;

        cv::drawKeypoints(image, detection.blob_keypoints, blob_preview, cv::Scalar(0, 0, 255),
                          cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        const std::filesystem::path blob_path =
            std::filesystem::path(config.blob_dir) / (image_path.stem().string() + "_blobs.png");

        cv::imwrite(blob_path.string(), blob_preview);
      }

      if (!detection.found)
      {
        std::cout << "[SKIP] " << image_path.filename().string() << " - pattern not found"
                  << " (blob candidates: " << detection.blob_keypoints.size() << ")\n";

        continue;
      }

      object_points.push_back(board_points);

      image_points.push_back(detection.points);

      used_images.push_back(image_path.filename().string());

      if (config.save_detection_debug)
      {
        cv::Mat detection_preview = image.clone();

        cv::drawChessboardCorners(detection_preview,
                                  cv::Size(config.board.columns, config.board.rows),
                                  detection.points, true);

        const std::filesystem::path detection_path = std::filesystem::path(config.detection_dir) /
                                                     (image_path.stem().string() + "_detected.png");

        cv::imwrite(detection_path.string(), detection_preview);
      }

      std::cout << "[OK]   " << image_path.filename().string()
                << " (blob candidates: " << detection.blob_keypoints.size() << ")\n";
    }

    if (image_points.size() < 3)
    {
      throw std::runtime_error("Not enough valid calibration images.");
    }

    const sl3d::CameraCalibrationResult result =
        sl3d::CalibrateCamera(object_points, image_points, image_size, config.calibration);

    const std::filesystem::path result_path = config.result_file;

    if (!result_path.parent_path().empty())
    {
      std::filesystem::create_directories(result_path.parent_path());
    }

    cv::FileStorage file_storage(result_path.string(), cv::FileStorage::WRITE);

    if (!file_storage.isOpened())
    {
      throw std::runtime_error("Failed to open output calibration file.");
    }

    file_storage << "image_width" << image_size.width;

    file_storage << "image_height" << image_size.height;

    file_storage << "board_pattern" << PatternToString(config.board.pattern);

    file_storage << "board_columns" << config.board.columns;

    file_storage << "board_rows" << config.board.rows;

    file_storage << "board_spacing" << config.board.spacing;

    file_storage << "rms" << result.rms;

    file_storage << "reprojection_error" << result.reprojection_error;

    file_storage << "camera_matrix" << result.camera_matrix;

    file_storage << "distortion_coefficients" << result.distortion_coefficients;

    file_storage << "rotation_vectors" << result.rotation_vectors;

    file_storage << "translation_vectors" << result.translation_vectors;

    file_storage << "per_view_errors" << result.per_view_errors;

    file_storage << "used_images" << used_images;

    file_storage << "views"
                 << "[";

    for (std::size_t i = 0; i < image_points.size(); ++i)
    {
      file_storage << "{";

      file_storage << "image" << used_images[i];

      file_storage << "image_points" << image_points[i];

      file_storage << "}";
    }

    file_storage << "]";

    file_storage.release();

    std::cout << "\nCamera calibration completed.\n"
              << "Valid images: " << image_points.size() << " / " << image_paths.size() << '\n'
              << "RMS: " << result.rms << '\n'
              << "Reprojection error: " << result.reprojection_error << " px\n"
              << "\nCamera matrix:\n"
              << result.camera_matrix << '\n'
              << "\nDistortion coefficients:\n"
              << result.distortion_coefficients << '\n'
              << "\nPer-view errors:\n";

    for (std::size_t i = 0; i < result.per_view_errors.size(); ++i)
    {
      std::cout << "  " << used_images[i] << ": " << result.per_view_errors[i] << " px\n";
    }

    std::cout << "\nSaved calibration views: " << image_points.size() << '\n'
              << "Saved to: " << result_path.string() << '\n';
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error: " << e.what() << '\n';

    return 1;
  }

  return 0;
}