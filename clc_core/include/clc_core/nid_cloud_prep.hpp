#pragma once

#include <vector>

#include <clc/linalg.hpp>
#include <glim_cloud_fusion/fusion.hpp>
#include <vlcal/common/frame_cpu.hpp>

namespace clc {

/// Histogram-equalize intensities to [0, 1), matching vlcal preprocess / NID expectations.
void equalize_intensities_for_nid(std::vector<double>& intensities);

/// Express fused world-frame points in the reference scan sensor frame (`p_ref = T_world_ref^{-1} p_world`).
void transform_dense_cloud_from_world(glim_cloud_fusion::DensePointCloud& cloud, const Mat4d& T_world_ref);

/// Build a vlcal frame for NID (equalized intensities, homogeneous points).
vlcal::FrameCPU::Ptr dense_cloud_to_nid_frame(const glim_cloud_fusion::DensePointCloud& cloud);

}  // namespace clc
