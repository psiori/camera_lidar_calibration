// colormap.cpp — clc_core session, I/O, fusion, or calibration logic.

#include <clc_core/colormap.hpp>

#include <algorithm>
#include <cmath>

namespace clc {

std::array<float, 3> jet_color(float t) {
  t = std::clamp(t, 0.f, 1.f);
  // Same piecewise JET as psiviz pointcloud.frag (blue at low t, red at high t).
  return {
    std::clamp(1.5f - std::fabs(4.f * t - 3.f), 0.f, 1.f),
    std::clamp(1.5f - std::fabs(4.f * t - 2.f), 0.f, 1.f),
    std::clamp(1.5f - std::fabs(4.f * t - 1.f), 0.f, 1.f),
  };
}

void colormap_jet(const std::vector<float>& values, float min_v, float max_v, std::vector<std::uint8_t>& rgb) {
  rgb.resize(values.size() * 3);
  const float denom = std::max(max_v - min_v, 1e-6f);
  for (std::size_t i = 0; i < values.size(); ++i) {
    const float t = (values[i] - min_v) / denom;
    const auto c = jet_color(t);
    rgb[i * 3 + 0] = static_cast<std::uint8_t>(std::clamp(c[0] * 255.f, 0.f, 255.f));
    rgb[i * 3 + 1] = static_cast<std::uint8_t>(std::clamp(c[1] * 255.f, 0.f, 255.f));
    rgb[i * 3 + 2] = static_cast<std::uint8_t>(std::clamp(c[2] * 255.f, 0.f, 255.f));
  }
}

void colormap_jet_range(const std::vector<float>& range_values, float min_v, float max_v, std::vector<std::uint8_t>& rgb) {
  rgb.resize(range_values.size() * 3);
  const float denom = std::max(max_v - min_v, 1e-6f);
  for (std::size_t i = 0; i < range_values.size(); ++i) {
    float t = 1.f;
    if (range_values[i] > kRangeNoReturnEpsilon) {
      t = std::clamp((range_values[i] - min_v) / denom, 0.f, 1.f);
    }
    const auto c = jet_color(t);
    rgb[i * 3 + 0] = static_cast<std::uint8_t>(std::clamp(c[0] * 255.f, 0.f, 255.f));
    rgb[i * 3 + 1] = static_cast<std::uint8_t>(std::clamp(c[1] * 255.f, 0.f, 255.f));
    rgb[i * 3 + 2] = static_cast<std::uint8_t>(std::clamp(c[2] * 255.f, 0.f, 255.f));
  }
}

}  // namespace clc
