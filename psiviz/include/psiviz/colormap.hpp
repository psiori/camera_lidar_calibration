#pragma once

#include <array>
#include <cstddef>

namespace psiviz {

/// Lookup table for mapping normalized scalars to JET false-color RGB on the GPU.
class Colormap {
 public:
  /// Number of entries along the scalar axis (0 .. kSize-1).
  static constexpr std::size_t kSize = 256;

  /// Fills the LUT with the JET ramp from 0 (blue) to 1 (red).
  Colormap();

  /// Raw LUT as interleaved RGB floats in [0,1], length kSize * 3.
  [[nodiscard]] const float* data() const { return lut_.data(); }

  /// Sample the analytic JET ramp at `t` in [0,1] without using the LUT.
  [[nodiscard]] static std::array<float, 3> jet(float t);

 private:
  std::array<float, kSize * 3> lut_{};
};

}  // namespace psiviz
