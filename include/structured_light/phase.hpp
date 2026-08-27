#pragma once

#include <opencv2/core/mat.hpp>
#include <vector>

namespace sl3d
{

struct PhaseResult
{
  cv::Mat wrapped_phase;
  cv::Mat modulation;
  cv::Mat valid_mask;
};

[[nodiscard]]
PhaseResult ComputeWrappedPhase(const std::vector<cv::Mat>& phase_images,
                                double modulation_threshold = 0.0);

}  // namespace sl3d