#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <opencv2/core.hpp>
#include <string>

#include "structured_light/fringe.hpp"
#include "structured_light/phase.hpp"
#include "structured_light/phase_unwrap.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

bool Require(bool condition, const std::string& message)
{
  if (!condition)
  {
    std::cerr << "[FAILED] " << message << '\n';

    return false;
  }

  return true;
}

}  // namespace

int main()
{
  constexpr int kWidth = 640;
  constexpr int kHeight = 480;

  const std::array<int, 3> frequencies = {70, 64, 59};

  std::array<sl3d::PhaseResult, 3> phase_results;

  for (std::size_t i = 0; i < frequencies.size(); ++i)
  {
    sl3d::FringeParameters params;

    params.width = kWidth;
    params.height = kHeight;

    params.period = static_cast<double>(kWidth) / static_cast<double>(frequencies[i]);

    params.phase_steps = 4;

    params.min_gray = 0;
    params.max_gray = 255;

    params.orientation = sl3d::FringeOrientation::kVertical;

    const auto fringes = sl3d::GenerateSinusoidalFringes(params);

    phase_results[i] = sl3d::ComputeWrappedPhase(fringes, 10.0);
  }

  const sl3d::PhaseUnwrapResult result =
      sl3d::UnwrapThreeFrequencyPhase(phase_results, frequencies);

  bool passed = true;

  passed &= Require(result.absolute_phase.type() == CV_64FC1, "Absolute phase must be CV_64FC1.");

  passed &= Require(result.fringe_order.type() == CV_32SC1, "Fringe order must be CV_32SC1.");

  passed &= Require(result.valid_mask.type() == CV_8UC1, "Valid mask must be CV_8UC1.");

  double maximum_phase_error = 0.0;

  for (int y = 0; y < kHeight; ++y)
  {
    for (int x = 0; x < kWidth; ++x)
    {
      const double expected_phase = 2.0 * kPi * static_cast<double>(frequencies[0]) *
                                    static_cast<double>(x) / static_cast<double>(kWidth);

      const double estimated_phase = result.absolute_phase.at<double>(y, x);

      const double phase_error = std::abs(estimated_phase - expected_phase);

      maximum_phase_error = std::max(maximum_phase_error, phase_error);
    }
  }

  passed &= Require(maximum_phase_error < 0.02, "Absolute phase error is too large.");

  passed &= Require(cv::countNonZero(result.valid_mask) == kWidth * kHeight,
                    "All synthetic fringe pixels should be valid.");

  int minimum_order = frequencies[0];
  int maximum_order = -1;

  for (int y = 0; y < kHeight; ++y)
  {
    for (int x = 0; x < kWidth; ++x)
    {
      const int order = result.fringe_order.at<int>(y, x);

      minimum_order = std::min(minimum_order, order);

      maximum_order = std::max(maximum_order, order);
    }
  }

  if (!passed)
  {
    return 1;
  }

  std::cout << "[PASSED] Three-frequency phase unwrapping test.\n"
            << "Maximum phase error: " << maximum_phase_error << " rad\n"
            << "Fringe order range: " << minimum_order << " - " << maximum_order << '\n';

  return 0;
}