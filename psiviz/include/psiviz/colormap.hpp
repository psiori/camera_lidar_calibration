#pragma once

#include <array>
#include <cstddef>

namespace psiviz {

/// JET colormap LUT (256 RGB triplets, linear 0..1).
class Colormap {
 public:
  static constexpr std::size_t kSize = 256;

  Colormap();

  [[nodiscard]] const float* data() const { return lut_.data(); }
  [[nodiscard]] static std::array<float, 3> jet(float t);

 private:
  std::array<float, kSize * 3> lut_{};
};

}  // namespace psiviz
