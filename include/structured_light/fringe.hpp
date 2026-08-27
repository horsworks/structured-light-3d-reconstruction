#pragma once

#include <opencv2/core/mat.hpp>
#include <vector>

namespace sl3d
{

enum class FringeOrientation
{
  kVertical,
  kHorizontal
};

struct FringeParameters
{
  int width = 640;
  int height = 480;

  double period = 32.0;
  int phase_steps = 4;
  double initial_phase = 0.0;

  int min_gray = 0;
  int max_gray = 255;

  FringeOrientation orientation = FringeOrientation::kVertical;
};

[[nodiscard]]
std::vector<cv::Mat> GenerateSinusoidalFringes(const FringeParameters& params);

}  // namespace sl3d