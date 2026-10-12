#include <cassert>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include <Eigen/Geometry>

#include <clc_core/colormap.hpp>
#include <clc_core/fusion_service.hpp>
#include <clc_core/range_image.hpp>
#include <clc_core/scan_geometry.hpp>
#include <clc_core/dataset_util.hpp>
#include <clc_core/session.hpp>

namespace {

void test_colormap_jet() {
  const std::vector<float> values{0.f, 0.5f, 1.f};
  std::vector<std::uint8_t> rgb;
  clc::colormap_jet(values, 0.f, 1.f, rgb);
  assert(rgb.size() == 9);
  assert(rgb[0] == 0 && rgb[1] == 0);  // low end of JET is blue-ish
}

void test_mask_rasterize() {
  clc::Polygon2f poly;
  poly.vertices = {{0.f, 0.f}, {10.f, 0.f}, {10.f, 10.f}, {0.f, 10.f}};
  const auto mask = clc::rasterize_mask(20, 20, {poly}, true);
  assert(mask.size() == 400);
  assert(mask[0] == 255);
  assert(mask[19 * 20 + 19] == 255);
}

void test_dataset_pickable_without_fusion() {
  clc::DatasetEntry ds;
  ds.image_path = "/tmp/img.png";
  ds.cloud_directory = "/tmp/pcds";
  ds.pcd_paths = {"/tmp/pcds/a.pcd"};
  assert(clc::dataset_pickable(ds));
  assert(!clc::dataset_ready(ds));
}

void test_prune_placeholder_calibration_datasets() {
  clc::SessionData session;
  session.calibration_datasets.emplace_back();
  auto& real = session.calibration_datasets.emplace_back();
  real.image_path = "/tmp/real.png";
  real.cloud_directory = "/tmp/pcds";
  assert(session.calibration_datasets.size() == 2);
  clc::prune_placeholder_calibration_datasets(session);
  assert(session.calibration_datasets.size() == 1);
  assert(session.calibration_datasets.front().image_path == real.image_path);
}

void test_session_round_trip() {
  clc::SessionData session;
  session.reference_scan_width = 64;
  session.reference_scan_height = 32;
  session.ensurePrimaryCalibration();
  auto& ds = session.calibration_datasets.front();
  ds.image_path = "/tmp/test_image.png";
  ds.cloud_directory = "/tmp/pcds";
  ds.correspondences.push_back(
    clc::PointCorrespondence{1.0, 2.0, 3.0, 4.0, 5.0});
  session.T_camera_lidar.translation() = Eigen::Vector3d(1, 2, 3);

  const auto path = std::filesystem::temp_directory_path() / "clc_session_test.json";
  assert(clc::save_session(path.string(), session));
  clc::SessionData loaded;
  assert(clc::load_session(path.string(), loaded));
  assert(loaded.calibration_datasets.front().image_path == ds.image_path);
  assert(loaded.calibration_datasets.front().cloud_directory == ds.cloud_directory);
  assert(loaded.reference_scan_width == 64);
  assert(!loaded.calibration_datasets.empty());
  assert(loaded.calibration_datasets.front().correspondences.size() == 1);
  std::filesystem::remove(path);
}

void test_scan_geometry_empty_directory() {
  const auto scan = clc::scan_pcd_directory("/nonexistent_clc_test_dir", 0, 0);
  assert(!scan.error.empty());
  assert(scan.paths.empty());
}

void test_fusion_single_scan_passthrough() {
  glim_cloud_fusion::LidarScan scan;
  scan.width = 2;
  scan.height = 2;
  for (int i = 0; i < 4; ++i) {
    scan.points.emplace_back(static_cast<double>(i) * 0.1, 0.0, 0.0, 1.0);
    scan.times.push_back(static_cast<double>(i) * 0.01);
    scan.intensities.push_back(static_cast<double>(i));
  }
  glim_cloud_fusion::FusionParams params;
  params.voxel_resolution = 0.05;
  const auto result = clc::run_fusion({scan}, params);
  assert(result.error == glim_cloud_fusion::FusionError::Ok);
  assert(!result.cloud.points.empty());
}

}  // namespace

int main() {
  test_colormap_jet();
  test_mask_rasterize();
  test_dataset_pickable_without_fusion();
  test_prune_placeholder_calibration_datasets();
  test_session_round_trip();
  test_scan_geometry_empty_directory();
  test_fusion_single_scan_passthrough();
  std::puts("clc_core_tests: ok");
  return 0;
}
