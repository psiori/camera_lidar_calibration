#pragma once

#include <cstdint>
#include <vector>

namespace psiviz {

struct PointCloudData {
  std::vector<float> positions;  // xyz interleaved
  std::vector<float> scalars;
  std::vector<float> rgb;        // rgb interleaved, same count as positions/3
  std::uint64_t point_count{0};
};

}  // namespace psiviz
