#pragma once

#include <Eigen/Geometry>

#include <clc/linalg.hpp>

namespace clc::detail {

Eigen::Isometry3d toIsometry(const Mat4d& T);
Mat4d fromIsometry(const Eigen::Isometry3d& T);
Eigen::Vector3d toEigen(const Vec3d& v);
Vec3d fromEigen(const Eigen::Vector3d& v);

}  // namespace clc::detail
