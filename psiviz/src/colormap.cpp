// colormap.cpp — psiviz rendering, input, or geometry implementation.

#include "psiviz/colormap.hpp"

#include <algorithm>
#include <cmath>

namespace psiviz {

std::array<float, 3> Colormap::jet(float t) {
  t = std::clamp(t, 0.f, 1.f);
  const auto ch = [](float x) { return std::clamp(1.5f - std::fabs(4.f * x - 2.f), 0.f, 1.f); };
  return {ch(t + 0.5f), ch(t), ch(t - 0.5f)};
}

Colormap::Colormap() {
  for (std::size_t i = 0; i < kSize; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(kSize - 1);
    const auto rgb = jet(t);
    lut_[i * 3 + 0] = rgb[0];
    lut_[i * 3 + 1] = rgb[1];
    lut_[i * 3 + 2] = rgb[2];
  }
}

}  // namespace psiviz
