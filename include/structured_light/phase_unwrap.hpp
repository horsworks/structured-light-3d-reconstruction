#pragma once

#include <array>
#include <opencv2/core/mat.hpp>

#include "structured_light/phase.hpp"

namespace sl3d
{

struct PhaseUnwrapResult
{
  cv::Mat absolute_phase;
  cv::Mat fringe_order;
  cv::Mat valid_mask;
};

[[nodiscard]]
PhaseUnwrapResult UnwrapThreeFrequencyPhase(const std::array<PhaseResult, 3>& phase_results,
                                            const std::array<int, 3>& fringe_frequencies);

}  // namespace sl3d