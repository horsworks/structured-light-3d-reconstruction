#include "structured_light/phase_unwrap.hpp"

#include <array>
#include <cmath>
#include <opencv2/core.hpp>
#include <stdexcept>

namespace sl3d
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

double WrapToTwoPi(double phase)
{
  double wrapped = std::fmod(phase, kTwoPi);

  if (wrapped < 0.0)
  {
    wrapped += kTwoPi;
  }

  return wrapped;
}

void ValidatePhaseResults(const std::array<PhaseResult, 3>& phase_results,
                          const std::array<int, 3>& fringe_frequencies)
{
  const cv::Size image_size = phase_results[0].wrapped_phase.size();

  for (const auto& result : phase_results)
  {
    if (result.wrapped_phase.empty())
    {
      throw std::invalid_argument("Wrapped phase must not be empty.");
    }

    if (result.wrapped_phase.size() != image_size)
    {
      throw std::invalid_argument("All wrapped phase maps must have the same size.");
    }

    if (result.wrapped_phase.channels() != 1)
    {
      throw std::invalid_argument("Wrapped phase maps must be single-channel.");
    }

    if (result.valid_mask.empty() || result.valid_mask.size() != image_size ||
        result.valid_mask.type() != CV_8UC1)
    {
      throw std::invalid_argument("Valid masks must be CV_8UC1 and have the same size.");
    }
  }

  const int f1 = fringe_frequencies[0];
  const int f2 = fringe_frequencies[1];
  const int f3 = fringe_frequencies[2];

  if (!(f1 > f2 && f2 > f3 && f3 > 0))
  {
    throw std::invalid_argument("Frequencies must satisfy f1 > f2 > f3 > 0.");
  }

  const int frequency_12 = f1 - f2;
  const int frequency_23 = f2 - f3;
  const int coarse_frequency = frequency_12 - frequency_23;

  if (coarse_frequency != 1)
  {
    throw std::invalid_argument(
        "Three-frequency heterodyne requires "
        "f1 - 2*f2 + f3 = 1.");
  }
}

}  // namespace

PhaseUnwrapResult UnwrapThreeFrequencyPhase(const std::array<PhaseResult, 3>& phase_results,
                                            const std::array<int, 3>& fringe_frequencies)
{
  ValidatePhaseResults(phase_results, fringe_frequencies);

  const int f1 = fringe_frequencies[0];
  const int f2 = fringe_frequencies[1];
  const int f3 = fringe_frequencies[2];

  const int frequency_12 = f1 - f2;
  const int frequency_23 = f2 - f3;
  const int coarse_frequency = frequency_12 - frequency_23;

  const int rows = phase_results[0].wrapped_phase.rows;

  const int cols = phase_results[0].wrapped_phase.cols;

  std::array<cv::Mat, 3> phases_64f;

  for (std::size_t i = 0; i < phase_results.size(); ++i)
  {
    phase_results[i].wrapped_phase.convertTo(phases_64f[i], CV_64F);
  }

  PhaseUnwrapResult result;

  result.absolute_phase = cv::Mat::zeros(rows, cols, CV_64FC1);

  result.fringe_order = cv::Mat::zeros(rows, cols, CV_32SC1);

  result.valid_mask = cv::Mat::zeros(rows, cols, CV_8UC1);

  for (int y = 0; y < rows; ++y)
  {
    auto* absolute_phase_row = result.absolute_phase.ptr<double>(y);

    auto* fringe_order_row = result.fringe_order.ptr<int>(y);

    auto* valid_mask_row = result.valid_mask.ptr<unsigned char>(y);

    for (int x = 0; x < cols; ++x)
    {
      const bool is_valid = phase_results[0].valid_mask.at<unsigned char>(y, x) != 0 &&
                            phase_results[1].valid_mask.at<unsigned char>(y, x) != 0 &&
                            phase_results[2].valid_mask.at<unsigned char>(y, x) != 0;

      if (!is_valid)
      {
        continue;
      }

      const double phase_1 = WrapToTwoPi(phases_64f[0].at<double>(y, x));

      const double phase_2 = WrapToTwoPi(phases_64f[1].at<double>(y, x));

      const double phase_3 = WrapToTwoPi(phases_64f[2].at<double>(y, x));

      const double phase_12 = WrapToTwoPi(phase_1 - phase_2);

      const double phase_23 = WrapToTwoPi(phase_2 - phase_3);

      const double coarse_phase = WrapToTwoPi(phase_12 - phase_23);

      const int order_12 =
          static_cast<int>(std::lround((static_cast<double>(frequency_12) /
                                            static_cast<double>(coarse_frequency) * coarse_phase -
                                        phase_12) /
                                       kTwoPi));

      const double absolute_phase_12 = phase_12 + kTwoPi * static_cast<double>(order_12);

      const int order_1 = static_cast<int>(std::lround(
          (static_cast<double>(f1) / static_cast<double>(frequency_12) * absolute_phase_12 -
           phase_1) /
          kTwoPi));

      absolute_phase_row[x] = phase_1 + kTwoPi * static_cast<double>(order_1);

      fringe_order_row[x] = order_1;
      valid_mask_row[x] = 255;
    }
  }

  return result;
}

}  // namespace sl3d