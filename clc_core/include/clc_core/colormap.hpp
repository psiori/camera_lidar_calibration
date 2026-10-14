#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace clc {

/// JET false-color for a normalized scalar t in [0,1].
std::array<float, 3> jet_color(float t);

/// Map scalar values to packed RGB (3 bytes per pixel) with JET and [min_v, max_v].
void colormap_jet(const std::vector<float>& values, float min_v, float max_v, std::vector<std::uint8_t>& rgb);

}  // namespace clc
