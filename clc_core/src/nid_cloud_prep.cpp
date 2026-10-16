// nid_cloud_prep.cpp — clc_core session, I/O, fusion, or calibration logic.

#include <clc_core/nid_cloud_prep.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace clc {

void equalize_intensities_for_nid(std::vector<double>& intensities) {
  if (intensities.empty()) {
    return;
  }
  std::vector<int> indices(intensities.size());
  std::iota(indices.begin(), indices.end(), 0);
  std::sort(indices.begin(), indices.end(), [&](int a, int b) { return intensities[a] < intensities[b]; });
  const int bins = 256;
  std::vector<double> equalized(intensities.size());
  const double denom = static_cast<double>(indices.size());
  for (int i = 0; i < static_cast<int>(indices.size()); ++i) {
    const double value = std::floor(bins * static_cast<double>(i) / denom) / static_cast<double>(bins);
    equalized[static_cast<std::size_t>(indices[i])] = value;
  }
  intensities = std::move(equalized);
}

void equalize_dense_cloud_intensities(glim_cloud_fusion::DensePointCloud& cloud) {
  if (cloud.intensities.size() != cloud.points.size()) {
    cloud.intensities.resize(cloud.points.size(), 0.0);
  }
  equalize_intensities_for_nid(cloud.intensities);
}

void transform_dense_cloud_from_world(glim_cloud_fusion::DensePointCloud& cloud, const Mat4d& T_world_ref) {
  const Mat4d T_ref_world = inverseRigid(T_world_ref);
  for (auto& p : cloud.points) {
    const Vec3d v = transformPoint(T_ref_world, {p.x(), p.y(), p.z()});
    p = Eigen::Vector3d(v.x, v.y, v.z);
  }
}

vlcal::FrameCPU::Ptr dense_cloud_to_nid_frame(const glim_cloud_fusion::DensePointCloud& cloud) {
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> points;
  std::vector<double> intensities;
  points.reserve(cloud.points.size());
  intensities.reserve(cloud.intensities.size());
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    points.emplace_back(cloud.points[i].x(), cloud.points[i].y(), cloud.points[i].z(), 1.0);
    intensities.push_back(i < cloud.intensities.size() ? cloud.intensities[i] : 0.0);
  }
  auto frame = std::make_shared<vlcal::FrameCPU>(points);
  frame->add_intensities(intensities);
  return frame;
}

}  // namespace clc
