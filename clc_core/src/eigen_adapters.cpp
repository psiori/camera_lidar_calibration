// eigen_adapters.cpp — clc_core session, I/O, fusion, or calibration logic.

#include "eigen_adapters.hpp"

namespace clc::detail {

Eigen::Isometry3d toIsometry(const Mat4d& T) {
  Eigen::Matrix4d M;
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      M(r, c) = T(r, c);
    }
  }
  return Eigen::Isometry3d(M);
}

Mat4d fromIsometry(const Eigen::Isometry3d& T) {
  const Eigen::Matrix4d M = T.matrix();
  Mat4d out{};
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      out(r, c) = M(r, c);
    }
  }
  return out;
}

Eigen::Vector3d toEigen(const Vec3d& v) { return {v.x, v.y, v.z}; }

Vec3d fromEigen(const Eigen::Vector3d& v) { return {v.x(), v.y(), v.z()}; }

}  // namespace clc::detail
