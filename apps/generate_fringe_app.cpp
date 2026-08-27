#include <filesystem>
#include <iomanip>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <sstream>

#include "structured_light/fringe.hpp"

int main()
{
  try
  {
    sl3d::FringeParameters params;

    params.width = 640;
    params.height = 480;
    params.period = 32.0;
    params.phase_steps = 4;

    params.min_gray = 0;
    params.max_gray = 255;

    params.orientation = sl3d::FringeOrientation::kVertical;

    const auto fringes = sl3d::GenerateSinusoidalFringes(params);

    const std::filesystem::path output_dir = "output/fringes";
    std::filesystem::create_directories(output_dir);

    for (std::size_t i = 0; i < fringes.size(); ++i)
    {
      std::ostringstream filename;

      filename << "fringe_" << std::setw(2) << std::setfill('0') << i << ".png";

      const std::filesystem::path output_path = output_dir / filename.str();

      if (!cv::imwrite(output_path.string(), fringes[i]))
      {
        std::cerr << "Failed to save: " << output_path.string() << '\n';

        return 1;
      }

      std::cout << "Saved: " << output_path.string() << '\n';
    }

    std::cout << "Generated " << fringes.size() << " phase-shifting fringes.\n";
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error: " << e.what() << '\n';

    return 1;
  }

  return 0;
}