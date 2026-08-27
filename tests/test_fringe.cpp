#include <cmath>
#include <iostream>
#include <opencv2/core.hpp>
#include <string>

#include "structured_light/fringe.hpp"

namespace
{

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
  sl3d::FringeParameters params;

  params.width = 64;
  params.height = 48;
  params.period = 16.0;
  params.phase_steps = 4;

  params.min_gray = 20;
  params.max_gray = 220;

  params.orientation = sl3d::FringeOrientation::kVertical;

  const auto fringes = sl3d::GenerateSinusoidalFringes(params);

  bool passed = true;

  passed &= Require(fringes.size() == static_cast<std::size_t>(params.phase_steps),
                    "Incorrect number of fringe images.");

  for (const auto& fringe : fringes)
  {
    passed &= Require(fringe.rows == params.height && fringe.cols == params.width,
                      "Incorrect fringe image size.");

    passed &= Require(fringe.type() == CV_8UC1, "Fringe image must be CV_8UC1.");

    double min_value = 0.0;
    double max_value = 0.0;

    cv::minMaxLoc(fringe, &min_value, &max_value);

    passed &= Require(min_value >= params.min_gray && max_value <= params.max_gray,
                      "Gray value is outside configured range.");
  }

  // 周期性验证：
  // 相隔一个周期的位置应该具有相同灰度。
  const int sample_y = 10;
  const int x0 = 3;
  const int x1 = x0 + static_cast<int>(params.period);

  const int periodic_difference =
      std::abs(static_cast<int>(fringes[0].at<unsigned char>(sample_y, x0)) -
               static_cast<int>(fringes[0].at<unsigned char>(sample_y, x1)));

  passed &= Require(periodic_difference <= 1, "Fringe periodicity check failed.");

  // N 步均匀相移满足：
  //
  // sum cos(theta + 2*pi*k/N) = 0
  //
  // 因此 N 张图同一像素的平均灰度应约等于直流分量 A。
  const int sample_x = 7;

  double average_intensity = 0.0;

  for (const auto& fringe : fringes)
  {
    average_intensity += fringe.at<unsigned char>(sample_y, sample_x);
  }

  average_intensity /= static_cast<double>(fringes.size());

  const double expected_offset = 0.5 * static_cast<double>(params.min_gray + params.max_gray);

  passed &= Require(std::abs(average_intensity - expected_offset) <= 1.0,
                    "Phase-shift average intensity check failed.");

  // 水平条纹沿 y 变化，因此同一 y 下不同 x 应具有相同灰度。
  params.orientation = sl3d::FringeOrientation::kHorizontal;

  const auto horizontal_fringes = sl3d::GenerateSinusoidalFringes(params);

  passed &= Require(horizontal_fringes[0].at<unsigned char>(5, 10) ==
                        horizontal_fringes[0].at<unsigned char>(5, 30),
                    "Horizontal fringe orientation check failed.");

  if (!passed)
  {
    return 1;
  }

  std::cout << "[PASSED] Fringe generation tests.\n";

  return 0;
}