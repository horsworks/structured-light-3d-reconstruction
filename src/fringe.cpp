#include "structured_light/fringe.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace sl3d
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

void ValidateParameters(const FringeParameters& params)
{
  if (params.width <= 0 || params.height <= 0)
  {
    throw std::invalid_argument("Image width and height must be positive.");
  }

  if (!std::isfinite(params.period) || params.period <= 0.0)
  {
    throw std::invalid_argument("Fringe period must be positive and finite.");
  }

  if (params.phase_steps < 3)
  {
    throw std::invalid_argument("Phase steps must be at least 3.");
  }

  if (params.min_gray < 0 || params.max_gray > 255 || params.min_gray >= params.max_gray)
  {
    throw std::invalid_argument("Gray range must satisfy 0 <= min_gray < max_gray <= 255.");
  }

  if (!std::isfinite(params.initial_phase))
  {
    throw std::invalid_argument("Initial phase must be finite.");
  }
}

}  // namespace

std::vector<cv::Mat> GenerateSinusoidalFringes(const FringeParameters& params)
{
  ValidateParameters(params);

  const double offset = 0.5 * static_cast<double>(params.min_gray + params.max_gray);

  const double amplitude = 0.5 * static_cast<double>(params.max_gray - params.min_gray);

  const double angular_frequency = 2.0 * kPi / params.period;

  std::vector<cv::Mat> fringes;
  fringes.reserve(static_cast<std::size_t>(params.phase_steps));

  for (int k = 0; k < params.phase_steps; ++k)
  {
    const double phase_shift =
        2.0 * kPi * static_cast<double>(k) / static_cast<double>(params.phase_steps);

    cv::Mat fringe(params.height, params.width, CV_8UC1);

    for (int y = 0; y < params.height; ++y)
    {
      auto* row = fringe.ptr<unsigned char>(y);

      for (int x = 0; x < params.width; ++x)
      {
        const double coordinate = params.orientation == FringeOrientation::kVertical
                                      ? static_cast<double>(x)
                                      : static_cast<double>(y);

        const double phase = angular_frequency * coordinate + phase_shift + params.initial_phase;

        const double intensity = offset + amplitude * std::cos(phase);

        const int gray = std::clamp(static_cast<int>(std::lround(intensity)), 0, 255);

        row[x] = static_cast<unsigned char>(gray);
      }
    }

    fringes.emplace_back(std::move(fringe));
  }

  return fringes;
}

}  // namespace sl3d