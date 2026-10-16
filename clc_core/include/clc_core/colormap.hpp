#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace clc {

/// JET false-color for a normalized scalar t in [0,1].
std::array<float, 3> jet_color(float t);

/// Map scalar values to packed RGB (3 bytes per pixel) with JET and [min_v, max_v].
void colormap_jet(const std::vector<float>& values, float min_v, float max_v, std::vector<std::uint8_t>& rgb);

/// Euclidean range at or below this (meters) is treated as no-return in range-image JET coloring.
inline constexpr float kRangeNoReturnEpsilon = 1e-3f;

/// JET for structured range images: no-return pixels map to the high end of the scale (far), not min range.
void colormap_jet_range(const std::vector<float>& range_values, float min_v, float max_v, std::vector<std::uint8_t>& rgb);

}  // namespace clc
