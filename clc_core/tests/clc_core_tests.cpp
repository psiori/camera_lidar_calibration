#include <cassert>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include <Eigen/Geometry>

#include <clc_core/colormap.hpp>
#include <clc_core/fusion_service.hpp>
#include <clc_core/nid_cloud_prep.hpp>
#include <clc_core/pcd_io.hpp>
#include <clc_core/range_image.hpp>
#include <clc_core/scan_geometry.hpp>
#include <clc_core/dataset_util.hpp>
#include <clc_core/session.hpp>

#include <vlcal/calib/visual_camera_calibration.hpp>

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
  clc::sync_dataset_fusion_status(ds);
  assert(clc::dataset_is_single_scan(ds));
  assert(clc::dataset_ready(ds));
}

void test_multi_scan_not_ready_until_fused() {
  clc::DatasetEntry ds;
  ds.image_path = "/tmp/img.png";
  ds.cloud_directory = "/tmp/pcds";
  ds.pcd_paths = {"/tmp/pcds/a.pcd", "/tmp/pcds/b.pcd"};
  clc::sync_dataset_fusion_status(ds);
  assert(!clc::dataset_ready(ds));
  ds.fusion_complete = true;
  assert(clc::dataset_ready(ds));
}

void test_structured_scan_reference_ignores_single_scan_only() {
  clc::SessionData session;
  session.reference_scan_width = 100;
  session.reference_scan_height = 1;
  clc::DatasetEntry single;
  single.pcd_paths = {"/tmp/one.pcd"};
  session.calibration_datasets.push_back(single);
  int w = 0;
  int h = 0;
  clc::session_structured_scan_reference(session, w, h);
  assert(w == 0 && h == 0);
  assert(!clc::session_has_multi_scan_dataset(session));
  clc::clear_session_structured_scan_reference_if_no_multi_scan(session);
  assert(session.reference_scan_width == 0);
  assert(session.reference_scan_height == 0);
  session.global_mask.path = "/tmp/mask.png";
  session.global_mask.width = 64;
  clc::reset_session_structured_scan_reference(session);
  assert(session.global_mask.path.empty());
  assert(session.global_mask.width == 0);

  clc::DatasetEntry multi;
  multi.pcd_paths = {"/tmp/a.pcd", "/tmp/b.pcd"};
  session.calibration_datasets.push_back(multi);
  session.reference_scan_width = 64;
  session.reference_scan_height = 32;
  clc::session_structured_scan_reference(session, w, h);
  assert(w == 64 && h == 32);
  assert(clc::session_has_multi_scan_dataset(session));
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
  session.scan_time.scan_duration = 0.1;
  session.fusion_params.scan_time = session.scan_time;
  ds.fusion_params.scan_time = session.scan_time;

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

void test_calibration_params_defaults_and_round_trip() {
  vlcal::VisualCameraCalibrationParams params;
  clc::apply_default_calibration_params(params);
  assert(params.num_threads == 15);
  assert(params.nid_bins == 16);
  assert(params.registration_type == vlcal::RegistrationType::NID_BFGS);
  assert(std::abs(params.delta_trans_thresh - 0.005) < 1e-12);
  assert(std::abs(params.delta_rot_thresh - 0.01 * M_PI / 180.0) < 1e-15);
  assert(!params.disable_z_buffer_culling);

  clc::SessionData session;
  session.ensurePrimaryCalibration();
  session.calibration_datasets.front().image_path = "/tmp/nid.png";
  session.calibration_datasets.front().cloud_directory = "/tmp/pcds";
  session.calibration_params = params;
  session.calibration_params.max_outer_iterations = 3;
  session.calibration_params.nid_bins = 24;
  session.calibration_params.registration_type = vlcal::RegistrationType::NID_NELDER_MEAD;
  session.fusion_params.voxel_resolution = 0.01;
  session.fusion_params.k_neighbors = 32;
  session.scan_time.scan_duration = 0.1;
  session.fusion_params.scan_time = session.scan_time;
  session.calibration_datasets.front().fusion_params.scan_time = session.scan_time;

  const auto path = std::filesystem::temp_directory_path() / "clc_session_nid_params.json";
  assert(clc::save_session(path.string(), session));
  clc::SessionData loaded;
  assert(clc::load_session(path.string(), loaded));
  assert(loaded.calibration_params.max_outer_iterations == 3);
  assert(loaded.calibration_params.nid_bins == 24);
  assert(loaded.calibration_params.registration_type == vlcal::RegistrationType::NID_NELDER_MEAD);
  assert(loaded.calibration_params.num_threads == 15);
  assert(std::abs(loaded.fusion_params.voxel_resolution - 0.01) < 1e-12);
  assert(loaded.fusion_params.k_neighbors == 32);
  std::filesystem::remove(path);
}

void test_fusion_k_neighbors_default() {
  glim_cloud_fusion::FusionParams params;
  clc::apply_default_fusion_params(params);
  assert(params.k_neighbors == 20);
  assert(params.backend == glim_cloud_fusion::FusionBackend::CtIntegrator);
  assert(std::abs(params.min_distance - 1.0) < 1e-12);
  assert(params.target_num_points == 10000);
  assert(std::abs(params.voxel_resolution - 0.05) < 1e-12);
  assert(params.num_threads == 16);
  assert(std::abs(params.integrator_ivox_resolution - 1.0) < 1e-12);
  assert(std::abs(params.integrator_ivox_min_points_dist - 0.05) < 1e-12);
  assert(std::abs(params.max_correspondence_distance - 1.0) < 1e-12);
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

void test_equalize_intensities_for_nid() {
  std::vector<double> intensities{100.0, 10.0, 50.0};
  clc::equalize_intensities_for_nid(intensities);
  assert(intensities.size() == 3);
  assert(intensities[0] < intensities[1]);
  assert(intensities[1] < intensities[2]);
  assert(intensities.front() >= 0.0 && intensities.back() < 1.0);
}

void test_dense_pcd_round_trip_non_timestamp_name() {
  const std::filesystem::path path =
    std::filesystem::temp_directory_path() / "clc_fused_test_dataset.pcd";
  glim_cloud_fusion::DensePointCloud written;
  written.points = {Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Vector3d(4.0, 5.0, 6.0)};
  written.intensities = {10.0, 20.0};
  std::string save_err;
  assert(clc::save_dense_pcd(path.string(), written, save_err));

  glim_cloud_fusion::DensePointCloud loaded;
  std::string load_err;
  assert(clc::load_dense_pcd(path.string(), loaded, load_err));
  assert(loaded.points.size() == 2);
  assert(loaded.intensities.size() == 2);
  assert(std::abs(loaded.points[0].x() - 1.0) < 1e-5);
  assert(std::abs(loaded.intensities[1] - 20.0) < 1e-5);

  clc::PointXYZITCloud scan_cloud;
  assert(!clc::load_pcd_xyzit(path.string(), scan_cloud, load_err));

  std::error_code ec;
  std::filesystem::remove(path, ec);
}

}  // namespace

int main() {
  test_colormap_jet();
  test_mask_rasterize();
  test_dataset_pickable_without_fusion();
  test_multi_scan_not_ready_until_fused();
  test_structured_scan_reference_ignores_single_scan_only();
  test_prune_placeholder_calibration_datasets();
  test_session_round_trip();
  test_calibration_params_defaults_and_round_trip();
  test_fusion_k_neighbors_default();
  test_scan_geometry_empty_directory();
  test_fusion_single_scan_passthrough();
  test_equalize_intensities_for_nid();
  test_dense_pcd_round_trip_non_timestamp_name();
  std::puts("clc_core_tests: ok");
  return 0;
}
