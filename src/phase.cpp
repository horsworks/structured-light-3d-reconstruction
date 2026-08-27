#include "structured_light/phase.hpp"

#include <cmath>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <vector>

namespace sl3d
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

void ValidatePhaseImages(const std::vector<cv::Mat>& phase_images, double modulation_threshold)
{
  if (phase_images.size() < 3)
  {
    throw std::invalid_argument("At least 3 phase-shifting images are required.");
  }

  if (!std::isfinite(modulation_threshold) || modulation_threshold < 0.0)
  {
    throw std::invalid_argument("Modulation threshold must be finite and non-negative.");
  }

  const cv::Size image_size = phase_images.front().size();

  if (phase_images.front().empty())
  {
    throw std::invalid_argument("Phase image must not be empty.");
  }

  for (const auto& image : phase_images)
  {
    if (image.empty())
    {
      throw std::invalid_argument("Phase image must not be empty.");
    }

    if (image.size() != image_size)
    {
      throw std::invalid_argument("All phase images must have the same size.");
    }

    if (image.channels() != 1)
    {
      throw std::invalid_argument("Phase images must be single-channel.");
    }
  }
}

}  // namespace

PhaseResult ComputeWrappedPhase(const std::vector<cv::Mat>& phase_images,
                                double modulation_threshold)
{
  ValidatePhaseImages(phase_images, modulation_threshold);

  const std::size_t phase_steps = phase_images.size();
  const int rows = phase_images.front().rows;
  const int cols = phase_images.front().cols;

  std::vector<cv::Mat> images_64f;
  images_64f.reserve(phase_steps);

  for (const auto& image : phase_images)
  {
    cv::Mat image_64f;
    image.convertTo(image_64f, CV_64F);
    images_64f.emplace_back(std::move(image_64f));
  }

  std::vector<double> cos_shifts(phase_steps);
  std::vector<double> sin_shifts(phase_steps);

  for (std::size_t k = 0; k < phase_steps; ++k)
  {
    const double phase_shift =
        2.0 * kPi * static_cast<double>(k) / static_cast<double>(phase_steps);

    cos_shifts[k] = std::cos(phase_shift);
    sin_shifts[k] = std::sin(phase_shift);
  }

  PhaseResult result;

  result.wrapped_phase = cv::Mat::zeros(rows, cols, CV_64FC1);

  result.modulation = cv::Mat::zeros(rows, cols, CV_64FC1);

  result.valid_mask = cv::Mat::zeros(rows, cols, CV_8UC1);

  for (int y = 0; y < rows; ++y)
  {
    auto* phase_row = result.wrapped_phase.ptr<double>(y);

    auto* modulation_row = result.modulation.ptr<double>(y);

    auto* mask_row = result.valid_mask.ptr<unsigned char>(y);

    for (int x = 0; x < cols; ++x)
    {
      double cosine_sum = 0.0;
      double sine_sum = 0.0;

      for (std::size_t k = 0; k < phase_steps; ++k)
      {
        const double intensity = images_64f[k].at<double>(y, x);

        cosine_sum += intensity * cos_shifts[k];

        sine_sum += intensity * sin_shifts[k];
      }

      const double wrapped_phase = std::atan2(-sine_sum, cosine_sum);

      const double modulation =
          2.0 * std::hypot(cosine_sum, sine_sum) / static_cast<double>(phase_steps);

      phase_row[x] = wrapped_phase;
      modulation_row[x] = modulation;

      if (modulation > modulation_threshold)
      {
        mask_row[x] = 255;
      }
    }
  }

  return result;
}

}  // namespace sl3d