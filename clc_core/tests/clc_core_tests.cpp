#include <cassert>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>

#include <Eigen/Core>

#include <clc_core/colormap.hpp>
#include <clc_core/fusion_service.hpp>
#include <clc_core/nid_cloud_prep.hpp>
#include <clc_core/pcd_io.hpp>
#include <clc_core/range_image.hpp>
#include <clc_core/scan_geometry.hpp>
#include <clc_core/dataset_util.hpp>
#include <clc_core/session.hpp>
#include <clc_core/calibration_service.hpp>

#include <nlohmann/json.hpp>

#include <vlcal/calib/visual_camera_calibration.hpp>

namespace {

/// Builds a small structured scan with per-point timestamps for multi-scan fusion tests.
glim_cloud_fusion::LidarScan make_structured_test_scan(int scan_index) {
  glim_cloud_fusion::LidarScan scan;
  scan.width = 12;
  scan.height = 8;
  scan.stamp = static_cast<double>(scan_index);
  const int point_count = scan.width * scan.height;
  for (int i = 0; i < point_count; ++i) {
    const int row = i / scan.width;
    const int col = i % scan.width;
    scan.points.emplace_back(
      static_cast<double>(scan_index) * 3.0 + col * 0.15,
      row * 0.12,
      5.0 + col * 0.02,
      1.0);
    scan.times.push_back(static_cast<double>(i) * 8.0e6);
    scan.intensities.push_back(50.0 + static_cast<double>(i % 200));
  }
  return scan;
}

glim_cloud_fusion::FusionParams make_ct_integrator_test_params() {
  glim_cloud_fusion::FusionParams params;
  clc::apply_default_fusion_params(params);
  params.backend = glim_cloud_fusion::FusionBackend::CtIntegrator;
  params.num_threads = 2;
  params.target_num_points = 500;
  params.integrator_lm_outer_loops = 1;
  params.lm_max_iterations = 2;
  clc::apply_default_scan_time_params(params.scan_time);
  params.scan_time.scan_duration = 0.1;
  return params;
}

std::pair<double, double> intensity_min_max(const glim_cloud_fusion::DensePointCloud& cloud) {
  double min_v = cloud.intensities.empty() ? 0.0 : cloud.intensities.front();
  double max_v = min_v;
  for (const double v : cloud.intensities) {
    min_v = std::min(min_v, v);
    max_v = std::max(max_v, v);
  }
  return {min_v, max_v};
}

void test_colormap_jet() {
  const std::vector<float> values{0.f, 0.5f, 1.f};
  std::vector<std::uint8_t> rgb;
  clc::colormap_jet(values, 0.f, 1.f, rgb);
  assert(rgb.size() == 9);
  assert(rgb[0] == 0 && rgb[1] == 0 && rgb[2] > rgb[0]);  // low end of JET is blue-ish
  assert(rgb[6] > rgb[8]);  // high end is red-ish
}

void test_colormap_jet_range_no_return_at_high_end() {
  const std::vector<float> values{0.f, 10.f, 20.f};
  std::vector<std::uint8_t> rgb_plain;
  std::vector<std::uint8_t> rgb_range;
  clc::colormap_jet(values, 10.f, 20.f, rgb_plain);
  clc::colormap_jet_range(values, 10.f, 20.f, rgb_range);
  assert(rgb_plain.size() == 9);
  assert(rgb_range.size() == 9);
  // No-return (0) uses min in plain JET but max in range JET.
  assert(rgb_plain[0] != rgb_range[0] || rgb_plain[1] != rgb_range[1] || rgb_plain[2] != rgb_range[2]);
  assert(rgb_range[0] == rgb_plain[6] && rgb_range[1] == rgb_plain[7] && rgb_range[2] == rgb_plain[8]);
  assert(clc::range_colormap_data_min(values) == 10.f);
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

void test_image_first_scan_time_validation() {
  const std::string stamp = "2026-10-01T18_46_54_574Z";
  double parsed = 0.0;
  assert(clc::parse_image_filename_stamp(stamp + ".jpg", parsed));
  assert(clc::parse_pcd_filename_stamp(stamp + ".pcd", parsed));

  clc::DatasetEntry single;
  single.image_path = "/data/" + stamp + ".png";
  single.pcd_paths = {"/data/2026-10-01T18_46_54_999Z.pcd"};
  std::string err;
  assert(clc::validate_dataset_image_first_scan_time(single, err));

  clc::DatasetEntry multi_ok;
  multi_ok.image_path = "/data/" + stamp + ".png";
  multi_ok.pcd_paths = {"/data/" + stamp + ".pcd", "/data/2026-10-01T18_46_55_000Z.pcd"};
  err.clear();
  assert(clc::validate_dataset_image_first_scan_time(multi_ok, err));

  clc::DatasetEntry multi_late;
  multi_late.image_path = "/data/2026-10-01T18_46_54_700Z.png";
  multi_late.pcd_paths = {"/data/" + stamp + ".pcd", "/data/2026-10-01T18_46_55_000Z.pcd"};
  err.clear();
  assert(!clc::validate_dataset_image_first_scan_time(multi_late, err));
  assert(err.find("100") != std::string::npos);
}

void test_multi_scan_not_ready_until_fused() {
  clc::DatasetEntry ds;
  ds.image_path = "/tmp/img.png";
  ds.cloud_directory = "/tmp/pcds";
  ds.pcd_paths = {"/tmp/pcds/a.pcd", "/tmp/pcds/b.pcd"};
  clc::sync_dataset_fusion_status(ds);
  assert(!clc::dataset_ready(ds));
  ds.fusion_complete = true;
  assert(!clc::dataset_ready(ds));
}

void test_sync_preserves_in_memory_fusion_complete() {
  clc::DatasetEntry ds;
  ds.image_path = "/tmp/img.png";
  ds.cloud_directory = "/tmp/pcds";
  ds.pcd_paths = {"/tmp/pcds/a.pcd", "/tmp/pcds/b.pcd"};
  ds.fusion_complete = true;
  clc::sync_dataset_fusion_status(ds);
  assert(ds.fusion_complete);
  assert(!clc::dataset_ready(ds));
}

void test_dataset_not_ready_reasons() {
  clc::DatasetEntry ds;
  ds.pcd_paths = {"/tmp/a.pcd", "/tmp/b.pcd"};
  auto reasons = clc::dataset_not_ready_reasons(ds);
  assert(reasons.size() >= 2);
  ds.image_path = "/tmp/img.png";
  ds.cloud_directory = "/tmp/pcds";
  reasons = clc::dataset_not_ready_reasons(ds);
  assert(reasons.size() == 1);
  assert(reasons.front().find("fusion") != std::string::npos);
  ds.fusion_complete = true;
  assert(!clc::dataset_ready(ds));
  assert(clc::dataset_not_ready_reasons(ds).size() == 1);
}

void test_missing_fused_file_marks_not_ready() {
  clc::DatasetEntry ds;
  ds.image_path = "/tmp/img.png";
  ds.cloud_directory = "/tmp/pcds";
  ds.pcd_paths = {"/tmp/pcds/a.pcd", "/tmp/pcds/b.pcd"};
  ds.fused_cloud_path = "/tmp/pcds/does_not_exist_fused.pcd";
  ds.fusion_complete = true;
  clc::sync_dataset_fusion_status(ds);
  assert(!ds.fusion_complete);
  assert(!clc::dataset_ready(ds));
  const auto reasons = clc::dataset_not_ready_reasons(ds);
  assert(reasons.size() == 1);
  assert(reasons.front().find("missing") != std::string::npos);
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

void test_calibration_dataset_id_unique_ignores_placeholder_with_stale_id() {
  clc::SessionData session;
  auto& ghost = session.calibration_datasets.emplace_back();
  ghost.id = "2026-10-01T18_42_13_624Z";
  assert(clc::calibration_dataset_is_placeholder(ghost));
  assert(clc::calibration_dataset_id_unique(session, ghost.id));
  clc::erase_placeholder_calibration_datasets(session);
  assert(session.calibration_datasets.empty());
}

void test_sync_calibration_dataset_identity_clears_placeholder_id() {
  clc::DatasetEntry entry;
  entry.id = "stale-id";
  clc::sync_calibration_dataset_identity(entry);
  assert(entry.id.empty());
}

void test_global_mask_ops_session_round_trip() {
  clc::SessionData session;
  session.reference_scan_width = 64;
  session.reference_scan_height = 32;
  session.ensurePrimaryCalibration();
  session.scan_time.scan_duration = 0.1;
  session.fusion_params.scan_time = session.scan_time;
  session.calibration_datasets.front().fusion_params.scan_time = session.scan_time;
  clc::MaskPolygonOp op;
  op.keep_inside = true;
  op.polygon.vertices = {
    clc::Vec2f(10.f, 10.f),
    clc::Vec2f(50.f, 10.f),
    clc::Vec2f(30.f, 20.f),
  };
  session.global_mask.ops.push_back(op);
  session.global_mask.width = 64;
  session.global_mask.height = 32;
  const auto path = std::filesystem::temp_directory_path() / "clc_session_global_mask_ops.json";
  assert(clc::save_session(path.string(), session));
  clc::SessionData loaded;
  std::string error;
  assert(clc::load_session(path.string(), loaded, error));
  assert(loaded.global_mask.ops.size() == 1);
  assert(loaded.global_mask.ops.front().polygon.vertices.size() == 3);
  assert(loaded.global_mask.width == 64);
  std::vector<std::uint8_t> raster;
  assert(clc::rasterize_global_mask(loaded.global_mask, path.parent_path().string(), raster, error));
  assert(raster.size() == static_cast<std::size_t>(64 * 32));
  std::filesystem::remove(path);
}

void test_image_mask_session_round_trip() {
  clc::SessionData session;
  session.ensurePrimaryCalibration();
  auto& ds = session.calibration_datasets.front();
  ds.image_mask.path = "/tmp/image_mask_test.png";
  ds.image_mask.width = 640;
  ds.image_mask.height = 480;
  const auto path = std::filesystem::temp_directory_path() / "clc_session_image_mask.json";
  assert(clc::save_session(path.string(), session));
  clc::SessionData loaded;
  std::string error;
  assert(clc::load_session(path.string(), loaded, error));
  assert(loaded.calibration_datasets.front().image_mask.width == 640);
  assert(loaded.calibration_datasets.front().image_mask.path == ds.image_mask.path);
  std::filesystem::remove(path);
}

void test_validate_alignment_image_masks() {
  clc::AlignmentInput input;
  input.images.push_back(cv::Mat(10, 20, CV_8UC3));
  input.image_masks.push_back(std::vector<std::uint8_t>(10 * 20, 255));
  std::string error;
  assert(clc::validate_alignment_image_masks(input, error));
  input.image_masks[0].resize(5);
  assert(!clc::validate_alignment_image_masks(input, error));
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
  session.T_camera_lidar(0, 3) = 1.;
  session.T_camera_lidar(1, 3) = 2.;
  session.T_camera_lidar(2, 3) = 3.;
  session.extrinsic_available = true;
  session.scan_time.scan_duration = 0.1;
  session.fusion_params.scan_time = session.scan_time;
  ds.fusion_params.scan_time = session.scan_time;

  const auto path = std::filesystem::temp_directory_path() / "clc_session_test.json";
  assert(clc::save_session(path.string(), session));
  {
    std::ifstream in(path.string());
    const nlohmann::json saved = nlohmann::json::parse(in);
    assert(saved["T_camera_lidar"].is_array());
    assert(saved["T_camera_lidar"].size() == 16);
    assert(std::abs(saved["T_camera_lidar"][3].get<double>() - 1.) < 1e-12);
  }
  clc::SessionData loaded;
  std::string error;
  assert(clc::load_session(path.string(), loaded, error));
  assert(loaded.calibration_datasets.front().image_path == ds.image_path);
  assert(loaded.calibration_datasets.front().cloud_directory == ds.cloud_directory);
  assert(loaded.reference_scan_width == 64);
  assert(!loaded.calibration_datasets.empty());
  assert(loaded.calibration_datasets.front().correspondences.size() == 1);
  assert(loaded.extrinsic_available);
  assert(std::abs(loaded.T_camera_lidar(0, 3) - 1.) < 1e-12);
  assert(std::abs(loaded.T_camera_lidar(1, 3) - 2.) < 1e-12);
  assert(std::abs(loaded.T_camera_lidar(2, 3) - 3.) < 1e-12);
  std::filesystem::remove(path);
}

void test_extrinsic_available_default_identity() {
  clc::SessionData session;
  session.ensurePrimaryCalibration();
  session.scan_time.scan_duration = 0.1;
  session.fusion_params.scan_time = session.scan_time;
  const auto path =
      std::filesystem::temp_directory_path() / "clc_session_extrinsic_flag.json";
  assert(clc::save_session(path.string(), session));
  clc::SessionData loaded;
  std::string error;
  assert(clc::load_session(path.string(), loaded, error));
  assert(!loaded.extrinsic_available);
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
  std::string error;
  assert(clc::load_session(path.string(), loaded, error));
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

void test_session_inspection_load_without_fusion_params() {
  const auto path = std::filesystem::temp_directory_path() / "clc_session_insp_minimal.json";
  const std::string json_text = R"({
  "camera": {
    "model": "plumb_bob",
    "intrinsics": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    "distortion_coeffs": [0.0, 0.0, 0.0, 0.0, 0.0]
  },
  "calibration_datasets": [],
  "inspection_datasets": [
    {
      "image_path": "/tmp/insp2.png",
      "cloud_directory": "/tmp/insp2",
      "pcd_paths": ["/tmp/insp2/s.pcd"]
    }
  ],
  "scan_time": { "time_unit": 0, "time_origin": 0, "scan_duration": 0.1 },
  "T_camera_lidar": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
  "T_camera_lidar_initial": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
})";
  {
    std::ofstream out(path);
    out << json_text;
  }

  clc::SessionData loaded;
  std::string error;
  assert(clc::load_session(path.string(), loaded, error));
  assert(loaded.inspection_datasets.size() == 1);
  assert(loaded.inspection_datasets.front().image_path == "/tmp/insp2.png");
  assert(loaded.inspection_datasets.front().pcd_paths.size() == 1);
  std::filesystem::remove(path);
}

void test_session_inspection_ignores_invalid_fusion_params() {
  const auto path = std::filesystem::temp_directory_path() / "clc_session_insp_bad_fusion.json";
  const std::string json_text = R"({
  "camera": {
    "model": "plumb_bob",
    "intrinsics": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    "distortion_coeffs": [0.0, 0.0, 0.0, 0.0, 0.0]
  },
  "calibration_datasets": [],
  "inspection_datasets": [
    {
      "image_path": "/tmp/insp.png",
      "cloud_directory": "/tmp/insp",
      "pcd_paths": ["/tmp/insp/a.pcd"],
      "fusion_params": { "backend": "not_a_real_backend" }
    }
  ],
  "scan_time": { "time_unit": 0, "time_origin": 0, "scan_duration": 0.1 },
  "T_camera_lidar": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
  "T_camera_lidar_initial": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
})";
  {
    std::ofstream out(path);
    out << json_text;
  }

  clc::SessionData before;
  before.ensurePrimaryCalibration();
  before.calibration_datasets.front().image_path = "/tmp/keep.png";

  clc::SessionData loaded = before;
  std::string error;
  assert(clc::load_session(path.string(), loaded, error));
  assert(loaded.inspection_datasets.size() == 1);
  assert(loaded.inspection_datasets.front().image_path == "/tmp/insp.png");
  std::filesystem::remove(path);
}

void test_session_fusion_ivox_and_voxel_round_trip() {
  clc::SessionData session;
  session.ensurePrimaryCalibration();
  session.calibration_datasets.front().image_path = "/tmp/fusion_params.png";
  session.calibration_datasets.front().cloud_directory = "/tmp/pcds";
  auto& fusion = session.calibration_datasets.front().fusion_params;
  fusion.integrator_ivox_resolution = 0.75;
  fusion.voxel_resolution = 0.03;
  fusion.ivox_resolution = 0.25;
  fusion.backend = glim_cloud_fusion::FusionBackend::GlimSlam;

  const auto path = std::filesystem::temp_directory_path() / "clc_session_fusion_ivox_voxel.json";
  assert(clc::save_session(path.string(), session));
  clc::SessionData loaded;
  std::string error;
  assert(clc::load_session(path.string(), loaded, error));
  const auto& loaded_fusion = loaded.calibration_datasets.front().fusion_params;
  assert(std::abs(loaded_fusion.integrator_ivox_resolution - 0.75) < 1e-12);
  assert(std::abs(loaded_fusion.voxel_resolution - 0.03) < 1e-12);
  assert(std::abs(loaded_fusion.ivox_resolution - 0.25) < 1e-12);
  assert(loaded_fusion.backend == glim_cloud_fusion::FusionBackend::GlimSlam);
  std::filesystem::remove(path);
}

void test_fusion_ct_integrator_multi_scan() {
  auto params = make_ct_integrator_test_params();
  const auto scan_a = make_structured_test_scan(0);
  const auto scan_b = make_structured_test_scan(1);
  const auto result = clc::run_fusion({scan_a, scan_b}, params);
  assert(result.error == glim_cloud_fusion::FusionError::Ok);
  assert(result.cloud.points.size() > 50);
  assert(result.cloud.intensities.size() == result.cloud.points.size());
  const auto [min_i, max_i] = intensity_min_max(result.cloud);
  assert(min_i >= 0.0 && max_i > min_i);
}

void test_fusion_output_voxel_changes_point_count() {
  auto coarse = make_ct_integrator_test_params();
  auto fine = coarse;
  coarse.voxel_resolution = 0.2;
  fine.voxel_resolution = 0.01;
  const auto scan_a = make_structured_test_scan(0);
  const auto scan_b = make_structured_test_scan(1);
  const auto coarse_result = clc::run_fusion({scan_a, scan_b}, coarse);
  const auto fine_result = clc::run_fusion({scan_a, scan_b}, fine);
  assert(coarse_result.error == glim_cloud_fusion::FusionError::Ok);
  assert(fine_result.error == glim_cloud_fusion::FusionError::Ok);
  assert(fine_result.cloud.points.size() > coarse_result.cloud.points.size());
}

void test_fusion_integrator_ignores_slam_smoother_params() {
  auto base = make_ct_integrator_test_params();
  auto with_slam_smoother = base;
  with_slam_smoother.enable_fixed_lag_smoother = true;
  with_slam_smoother.smoother_lag = 5.0;
  with_slam_smoother.ivox_resolution = 0.3;
  const auto scan_a = make_structured_test_scan(0);
  const auto scan_b = make_structured_test_scan(1);
  const auto base_result = clc::run_fusion({scan_a, scan_b}, base);
  const auto smoother_result = clc::run_fusion({scan_a, scan_b}, with_slam_smoother);
  assert(base_result.error == glim_cloud_fusion::FusionError::Ok);
  assert(smoother_result.error == glim_cloud_fusion::FusionError::Ok);
  assert(base_result.cloud.points.size() == smoother_result.cloud.points.size());
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

void test_equalize_dense_cloud_intensities() {
  glim_cloud_fusion::DensePointCloud cloud;
  cloud.points = {Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY()};
  cloud.intensities = {5.0, 100.0, 50.0};
  clc::equalize_dense_cloud_intensities(cloud);
  assert(cloud.intensities.size() == 3);
  assert(cloud.intensities[0] < cloud.intensities[1]);
  assert(cloud.intensities[1] < cloud.intensities[2]);
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
  test_colormap_jet_range_no_return_at_high_end();
  test_mask_rasterize();
  test_dataset_pickable_without_fusion();
  test_image_first_scan_time_validation();
  test_multi_scan_not_ready_until_fused();
  test_sync_preserves_in_memory_fusion_complete();
  test_dataset_not_ready_reasons();
  test_missing_fused_file_marks_not_ready();
  test_structured_scan_reference_ignores_single_scan_only();
  test_prune_placeholder_calibration_datasets();
  test_calibration_dataset_id_unique_ignores_placeholder_with_stale_id();
  test_sync_calibration_dataset_identity_clears_placeholder_id();
  test_global_mask_ops_session_round_trip();
  test_image_mask_session_round_trip();
  test_validate_alignment_image_masks();
  test_session_round_trip();
  test_extrinsic_available_default_identity();
  test_session_inspection_load_without_fusion_params();
  test_session_inspection_ignores_invalid_fusion_params();
  test_calibration_params_defaults_and_round_trip();
  test_fusion_k_neighbors_default();
  test_scan_geometry_empty_directory();
  test_fusion_single_scan_passthrough();
  test_equalize_intensities_for_nid();
  test_equalize_dense_cloud_intensities();
  test_dense_pcd_round_trip_non_timestamp_name();
  test_session_fusion_ivox_and_voxel_round_trip();
  test_fusion_ct_integrator_multi_scan();
  test_fusion_output_voxel_changes_point_count();
  test_fusion_integrator_ignores_slam_smoother_params();
  std::puts("clc_core_tests: ok");
  return 0;
}
