#include <cmath>
#include <iostream>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "structured_light/fringe.hpp"
#include "structured_light/phase.hpp"

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

double WrapPhase(double phase)
{
  return std::atan2(std::sin(phase), std::cos(phase));
}

double PhaseDifference(double phase_a, double phase_b)
{
  return WrapPhase(phase_a - phase_b);
}

}  // namespace

int main()
{
  sl3d::FringeParameters params;

  params.width = 128;
  params.height = 64;
  params.period = 16.0;

  params.phase_steps = 5;
  params.initial_phase = 0.35;

  params.min_gray = 20;
  params.max_gray = 220;

  params.orientation = sl3d::FringeOrientation::kVertical;

  const auto fringes = sl3d::GenerateSinusoidalFringes(params);

  const sl3d::PhaseResult result = sl3d::ComputeWrappedPhase(fringes, 10.0);

  bool passed = true;

  passed &= Require(result.wrapped_phase.type() == CV_64FC1, "Wrapped phase must be CV_64FC1.");

  passed &= Require(result.modulation.type() == CV_64FC1, "Modulation must be CV_64FC1.");

  passed &= Require(result.valid_mask.type() == CV_8UC1, "Valid mask must be CV_8UC1.");

  passed &= Require(result.wrapped_phase.size() == cv::Size(params.width, params.height),
                    "Wrapped phase size is incorrect.");

  double maximum_phase_error = 0.0;

  for (int y = 0; y < params.height; ++y)
  {
    for (int x = 0; x < params.width; ++x)
    {
      const double expected_phase =
          WrapPhase(2.0 * kPi * static_cast<double>(x) / params.period + params.initial_phase);

      const double estimated_phase = result.wrapped_phase.at<double>(y, x);

      const double phase_error = std::abs(PhaseDifference(estimated_phase, expected_phase));

      maximum_phase_error = std::max(maximum_phase_error, phase_error);
    }
  }

  passed &= Require(maximum_phase_error < 0.03, "Wrapped phase error is too large.");

  const double mean_modulation = cv::mean(result.modulation, result.valid_mask)[0];

  const double expected_modulation = 0.5 * static_cast<double>(params.max_gray - params.min_gray);

  passed &= Require(std::abs(mean_modulation - expected_modulation) < 1.0,
                    "Estimated modulation is incorrect.");

  passed &= Require(cv::countNonZero(result.valid_mask) == params.width * params.height,
                    "Valid mask should contain all fringe pixels.");

  std::vector<cv::Mat> flat_images(4, cv::Mat(32, 32, CV_8UC1, cv::Scalar(128)));

  const sl3d::PhaseResult flat_result = sl3d::ComputeWrappedPhase(flat_images, 1.0);

  passed &= Require(cv::countNonZero(flat_result.valid_mask) == 0,
                    "Flat images should not produce valid phase pixels.");

  if (!passed)
  {
    return 1;
  }

  std::cout << "[PASSED] Wrapped phase test.\n"
            << "Maximum phase error: " << maximum_phase_error << " rad\n"
            << "Mean modulation: " << mean_modulation << '\n';

  return 0;
}