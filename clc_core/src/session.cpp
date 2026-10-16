// session.cpp — clc_core session, I/O, fusion, or calibration logic.

#include <clc_core/session.hpp>

#include <clc_core/dataset_util.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <thread>

#include <clc_core/range_image.hpp>

#include <nlohmann/json.hpp>

namespace clc {

namespace {

/// Writes `T` as a 16-element array: 4×4 matrix flattened in row-major order (row 0, then row 1, …).
void save_pose(nlohmann::json& j, const std::string& key, const Mat4d& T) {
  nlohmann::json row_major = nlohmann::json::array();
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      row_major.push_back(T(row, col));
    }
  }
  j[key] = std::move(row_major);
}

/// Loads a pose array: 16 elements (row-major 4×4) or legacy 7 elements (tx, ty, tz, qx, qy, qz, qw).
bool load_pose(const nlohmann::json& j, const std::string& key, Mat4d& T) {
  if (!j.contains(key)) {
    return false;
  }
  const auto& v = j[key];
  if (!v.is_array()) {
    return false;
  }
  if (v.size() == 16) {
    T = Mat4d::identity();
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        if (!v[row * 4 + col].is_number()) {
          return false;
        }
        T(row, col) = v[row * 4 + col].get<double>();
      }
    }
    return true;
  }
  if (v.size() == 7) {
    for (std::size_t i = 0; i < 7; ++i) {
      if (!v[i].is_number()) {
        return false;
      }
    }
    T = mat4dFromQuaternionTranslation(v[3], v[4], v[5], v[6], v[0], v[1], v[2]);
    return true;
  }
  return false;
}

nlohmann::json scan_time_to_json(const glim_cloud_fusion::ScanTimeParams& scan_time) {
  nlohmann::json j;
  j["time_unit"] = static_cast<int>(scan_time.time_unit);
  j["time_origin"] = static_cast<int>(scan_time.time_origin);
  j["scan_duration"] = *scan_time.scan_duration;
  return j;
}

nlohmann::json mask_polygon_op_to_json(const MaskPolygonOp& op) {
  nlohmann::json j;
  j["keep_inside"] = op.keep_inside;
  nlohmann::json vertices = nlohmann::json::array();
  for (const Vec2f& v : op.polygon.vertices) {
    vertices.push_back(nlohmann::json::array({v.x, v.y}));
  }
  j["vertices"] = std::move(vertices);
  return j;
}

bool mask_polygon_op_from_json(const nlohmann::json& j, MaskPolygonOp& op, std::string& error) {
  if (!j.is_object()) {
    error = "mask op must be an object";
    return false;
  }
  op.keep_inside = j.value("keep_inside", false);
  if (!j.contains("vertices") || !j["vertices"].is_array()) {
    error = "mask op missing vertices array";
    return false;
  }
  op.polygon.vertices.clear();
  for (const auto& pt : j["vertices"]) {
    if (!pt.is_array() || pt.size() < 2 || !pt[0].is_number() || !pt[1].is_number()) {
      error = "mask op vertex must be [x, y] numbers";
      return false;
    }
    op.polygon.vertices.emplace_back(pt[0].get<float>(), pt[1].get<float>());
  }
  if (op.polygon.vertices.size() < 3) {
    error = "mask op polygon needs at least 3 vertices";
    return false;
  }
  return true;
}

std::string resolve_global_mask_png_path(const GlobalMaskMeta& meta, const std::string& session_directory) {
  if (meta.path.empty()) {
    if (!session_directory.empty()) {
      const std::filesystem::path default_path =
        std::filesystem::path(session_directory) / "global_mask.png";
      if (std::filesystem::exists(default_path)) {
        return default_path.string();
      }
    }
    return {};
  }
  const std::filesystem::path stored(meta.path);
  if (std::filesystem::exists(stored)) {
    return stored.string();
  }
  if (!session_directory.empty()) {
    const std::filesystem::path beside =
      std::filesystem::path(session_directory) / stored.filename();
    if (std::filesystem::exists(beside)) {
      return beside.string();
    }
  }
  return {};
}

bool scan_time_from_json(const nlohmann::json& j, glim_cloud_fusion::ScanTimeParams& scan_time) {
  if (!j.is_object()) {
    return false;
  }
  scan_time.time_unit = static_cast<glim_cloud_fusion::TimeUnit>(
    j.value("time_unit", static_cast<int>(scan_time.time_unit)));
  scan_time.time_origin = static_cast<glim_cloud_fusion::TimeOrigin>(
    j.value("time_origin", static_cast<int>(scan_time.time_origin)));
  if (!j.contains("scan_duration") || !j["scan_duration"].is_number()) {
    return false;
  }
  const double duration = j["scan_duration"].get<double>();
  if (duration <= 0.0 || !std::isfinite(duration)) {
    return false;
  }
  scan_time.scan_duration = duration;
  return true;
}

const char* fusion_backend_to_string(glim_cloud_fusion::FusionBackend backend) {
  return backend == glim_cloud_fusion::FusionBackend::GlimSlam ? "glim_slam" : "ct_integrator";
}

bool fusion_backend_from_string(const std::string& s, glim_cloud_fusion::FusionBackend& backend) {
  if (s == "glim_slam") {
    backend = glim_cloud_fusion::FusionBackend::GlimSlam;
    return true;
  }
  if (s == "ct_integrator") {
    backend = glim_cloud_fusion::FusionBackend::CtIntegrator;
    return true;
  }
  return false;
}

nlohmann::json fusion_params_to_json(const glim_cloud_fusion::FusionParams& params) {
  nlohmann::json j;
  j["backend"] = fusion_backend_to_string(params.backend);
  j["voxel_resolution"] = params.voxel_resolution;
  j["num_threads"] = params.num_threads;
  j["max_correspondence_distance"] = params.max_correspondence_distance;
  j["k_neighbors"] = params.k_neighbors;
  j["ivox_resolution"] = params.ivox_resolution;
  j["ivox_min_points_dist"] = params.ivox_min_points_dist;
  j["ivox_lru_thresh"] = params.ivox_lru_thresh;
  j["location_consistency_inf_scale"] = params.location_consistency_inf_scale;
  j["constant_velocity_inf_scale"] = params.constant_velocity_inf_scale;
  j["lm_max_iterations"] = params.lm_max_iterations;
  j["smoother_lag"] = params.smoother_lag;
  j["use_isam2_dogleg"] = params.use_isam2_dogleg;
  j["isam2_relinearize_skip"] = params.isam2_relinearize_skip;
  j["isam2_relinearize_thresh"] = params.isam2_relinearize_thresh;
  j["compute_covs"] = params.compute_covs;
  j["enable_fixed_lag_smoother"] = params.enable_fixed_lag_smoother;
  j["target_num_points"] = params.target_num_points;
  j["min_distance"] = params.min_distance;
  j["integrator_ivox_resolution"] = params.integrator_ivox_resolution;
  j["integrator_ivox_min_points_dist"] = params.integrator_ivox_min_points_dist;
  j["integrator_ivox_lru_thresh"] = params.integrator_ivox_lru_thresh;
  j["integrator_lm_outer_loops"] = params.integrator_lm_outer_loops;
  j["integrator_debug"] = params.integrator_debug;
  j["scan_time"] = scan_time_to_json(params.scan_time);
  const auto& sm = params.slam_mapping;
  j["slam_mapping"] = {
    {"submap_max_num_keyframes", sm.submap_max_num_keyframes},
    {"keyframe_update_strategy", sm.keyframe_update_strategy},
    {"max_keyframe_overlap", sm.max_keyframe_overlap},
    {"keyframe_update_interval_trans", sm.keyframe_update_interval_trans},
    {"keyframe_update_interval_rot", sm.keyframe_update_interval_rot},
    {"submap_downsample_resolution", sm.submap_downsample_resolution},
    {"global_min_implicit_loop_overlap", sm.global_min_implicit_loop_overlap},
    {"global_max_implicit_loop_distance", sm.global_max_implicit_loop_distance},
    {"enable_odometry_smoother", sm.enable_odometry_smoother},
    {"odometry_smoother_lag_cap", sm.odometry_smoother_lag_cap},
    {"export_full_resolution", sm.export_full_resolution},
  };
  return j;
}

bool fusion_params_from_json(const nlohmann::json& j, glim_cloud_fusion::FusionParams& params) {
  if (!j.is_object()) {
    return false;
  }
  if (!j.contains("backend") || !j["backend"].is_string()) {
    return false;
  }
  if (!fusion_backend_from_string(j["backend"].get<std::string>(), params.backend)) {
    return false;
  }
  params.voxel_resolution = j.value("voxel_resolution", params.voxel_resolution);
  params.num_threads = j.value("num_threads", params.num_threads);
  params.max_correspondence_distance = j.value("max_correspondence_distance", params.max_correspondence_distance);
  params.k_neighbors = j.value("k_neighbors", params.k_neighbors);
  params.ivox_resolution = j.value("ivox_resolution", params.ivox_resolution);
  params.ivox_min_points_dist = j.value("ivox_min_points_dist", params.ivox_min_points_dist);
  params.ivox_lru_thresh = j.value("ivox_lru_thresh", params.ivox_lru_thresh);
  params.location_consistency_inf_scale =
    j.value("location_consistency_inf_scale", params.location_consistency_inf_scale);
  params.constant_velocity_inf_scale = j.value("constant_velocity_inf_scale", params.constant_velocity_inf_scale);
  params.lm_max_iterations = j.value("lm_max_iterations", params.lm_max_iterations);
  params.smoother_lag = j.value("smoother_lag", params.smoother_lag);
  params.use_isam2_dogleg = j.value("use_isam2_dogleg", params.use_isam2_dogleg);
  params.isam2_relinearize_skip = j.value("isam2_relinearize_skip", params.isam2_relinearize_skip);
  params.isam2_relinearize_thresh = j.value("isam2_relinearize_thresh", params.isam2_relinearize_thresh);
  params.compute_covs = j.value("compute_covs", params.compute_covs);
  params.enable_fixed_lag_smoother = j.value("enable_fixed_lag_smoother", params.enable_fixed_lag_smoother);
  if (!j.contains("scan_time") || !scan_time_from_json(j["scan_time"], params.scan_time)) {
    return false;
  }
  params.target_num_points = j.value("target_num_points", params.target_num_points);
  params.min_distance = j.value("min_distance", params.min_distance);
  params.integrator_ivox_resolution = j.value("integrator_ivox_resolution", params.integrator_ivox_resolution);
  params.integrator_ivox_min_points_dist =
    j.value("integrator_ivox_min_points_dist", params.integrator_ivox_min_points_dist);
  params.integrator_ivox_lru_thresh = j.value("integrator_ivox_lru_thresh", params.integrator_ivox_lru_thresh);
  params.integrator_lm_outer_loops = j.value("integrator_lm_outer_loops", params.integrator_lm_outer_loops);
  params.integrator_debug = j.value("integrator_debug", params.integrator_debug);
  if (j.contains("slam_mapping") && j["slam_mapping"].is_object()) {
    const auto& sm = j["slam_mapping"];
    auto& out = params.slam_mapping;
    out.submap_max_num_keyframes = sm.value("submap_max_num_keyframes", out.submap_max_num_keyframes);
    out.keyframe_update_strategy = sm.value("keyframe_update_strategy", out.keyframe_update_strategy);
    out.max_keyframe_overlap = sm.value("max_keyframe_overlap", out.max_keyframe_overlap);
    out.keyframe_update_interval_trans = sm.value("keyframe_update_interval_trans", out.keyframe_update_interval_trans);
    out.keyframe_update_interval_rot = sm.value("keyframe_update_interval_rot", out.keyframe_update_interval_rot);
    out.submap_downsample_resolution = sm.value("submap_downsample_resolution", out.submap_downsample_resolution);
    out.global_min_implicit_loop_overlap = sm.value("global_min_implicit_loop_overlap", out.global_min_implicit_loop_overlap);
    out.global_max_implicit_loop_distance = sm.value("global_max_implicit_loop_distance", out.global_max_implicit_loop_distance);
    out.enable_odometry_smoother = sm.value("enable_odometry_smoother", out.enable_odometry_smoother);
    out.odometry_smoother_lag_cap = sm.value("odometry_smoother_lag_cap", out.odometry_smoother_lag_cap);
    out.export_full_resolution = sm.value("export_full_resolution", out.export_full_resolution);
    out.odometry_target_num_points = sm.value("odometry_target_num_points", out.odometry_target_num_points);
  }
  return true;
}

/// JSON for a calibration dataset (fusion_params required on load).
nlohmann::json dataset_to_json(const DatasetEntry& d) {
  nlohmann::json j;
  j["id"] = d.id;
  j["image_path"] = d.image_path;
  j["cloud_directory"] = d.cloud_directory;
  j["fused_cloud_path"] = d.fused_cloud_path;
  j["pcd_paths"] = d.pcd_paths;
  j["fusion_complete"] = d.fusion_complete;
  j["geometry_error"] = d.geometry_error;
  j["fusion_params"] = fusion_params_to_json(d.fusion_params);
  j["correspondences"] = nlohmann::json::array();
  for (const auto& c : d.correspondences) {
    j["correspondences"].push_back({{"uv", {c.u, c.v}}, {"xyz", {c.x, c.y, c.z}}});
  }
  j["image_mask"]["path"] = d.image_mask.path;
  j["image_mask"]["width"] = d.image_mask.width;
  j["image_mask"]["height"] = d.image_mask.height;
  return j;
}

/// JSON for inspection data (image + scans only; no per-dataset fusion settings).
nlohmann::json inspection_dataset_to_json(const DatasetEntry& d) {
  nlohmann::json j;
  j["id"] = d.id;
  j["image_path"] = d.image_path;
  j["cloud_directory"] = d.cloud_directory;
  j["fused_cloud_path"] = d.fused_cloud_path;
  j["pcd_paths"] = d.pcd_paths;
  if (d.fusion_complete) {
    j["fusion_complete"] = true;
  }
  if (d.geometry_error) {
    j["geometry_error"] = true;
  }
  if (!d.correspondences.empty()) {
    j["correspondences"] = nlohmann::json::array();
    for (const auto& c : d.correspondences) {
      j["correspondences"].push_back({{"uv", {c.u, c.v}}, {"xyz", {c.x, c.y, c.z}}});
    }
  }
  return j;
}

void read_dataset_correspondences(const nlohmann::json& j, DatasetEntry& d) {
  if (!j.contains("correspondences") || !j["correspondences"].is_array()) {
    return;
  }
  for (const auto& c : j["correspondences"]) {
    PointCorrespondence corr;
    if (c.contains("uv") && c["uv"].is_array() && c["uv"].size() >= 2) {
      corr.u = c["uv"][0];
      corr.v = c["uv"][1];
    }
    if (c.contains("xyz") && c["xyz"].is_array() && c["xyz"].size() >= 3) {
      corr.x = c["xyz"][0];
      corr.y = c["xyz"][1];
      corr.z = c["xyz"][2];
    }
    d.correspondences.push_back(corr);
  }
}

bool dataset_from_json(const nlohmann::json& j, DatasetEntry& d, std::string& error) {
  error.clear();
  d = DatasetEntry{};
  d.id = j.value("id", "");
  d.image_path = j.value("image_path", "");
  d.cloud_directory = j.value("cloud_directory", "");
  d.fused_cloud_path = j.value("fused_cloud_path", "");
  d.pcd_paths = j.value("pcd_paths", std::vector<std::string>{});
  d.fusion_complete = j.value("fusion_complete", false);
  d.geometry_error = j.value("geometry_error", false);

  if (!j.contains("fusion_params") || !fusion_params_from_json(j["fusion_params"], d.fusion_params)) {
    error = "invalid or missing fusion_params";
    return false;
  }

  if (j.contains("scan_time")) {
    if (!scan_time_from_json(j["scan_time"], d.scan_time)) {
      error = "invalid scan_time";
      return false;
    }
    d.fusion_params.scan_time = d.scan_time;
  } else {
    d.scan_time = d.fusion_params.scan_time;
  }
  read_dataset_correspondences(j, d);
  if (j.contains("image_mask") && j["image_mask"].is_object()) {
    d.image_mask.path = j["image_mask"].value("path", "");
    d.image_mask.width = j["image_mask"].value("width", 0);
    d.image_mask.height = j["image_mask"].value("height", 0);
  }
  return true;
}

bool calibration_dataset_from_json(const nlohmann::json& j, DatasetEntry& d, std::string& error) {
  return dataset_from_json(j, d, error);
}

/// Inspection rows only need image + scan paths; ignore legacy fusion_params in JSON.
bool inspection_dataset_from_json(const nlohmann::json& j, DatasetEntry& d, std::string& error) {
  error.clear();
  d = DatasetEntry{};
  d.id = j.value("id", "");
  d.image_path = j.value("image_path", "");
  d.cloud_directory = j.value("cloud_directory", "");
  d.fused_cloud_path = j.value("fused_cloud_path", "");
  d.pcd_paths = j.value("pcd_paths", std::vector<std::string>{});
  d.fusion_complete = j.value("fusion_complete", false);
  d.geometry_error = j.value("geometry_error", false);
  apply_default_fusion_params(d.fusion_params);
  apply_default_scan_time_params(d.scan_time);
  apply_default_scan_time_params(d.fusion_params.scan_time);
  read_dataset_correspondences(j, d);
  return true;
}

}  // namespace

int default_fusion_num_threads() {
  const unsigned hw = std::thread::hardware_concurrency();
  return hw > 0 ? static_cast<int>(hw) : 4;
}

void apply_default_fusion_params(glim_cloud_fusion::FusionParams& params) {
  // CT integrator defaults: vlcal `DynamicPointCloudIntegratorParams` + `iVox(1.0, 0.05, 100)`.
  params.backend = glim_cloud_fusion::FusionBackend::CtIntegrator;
  params.voxel_resolution = 0.05;
  params.num_threads = 16;
  params.max_correspondence_distance = 1.0;
  params.k_neighbors = 20;
  params.ivox_resolution = 1.0;
  params.ivox_min_points_dist = 0.1;
  params.ivox_lru_thresh = 200;
  params.location_consistency_inf_scale = 1e-3;
  params.constant_velocity_inf_scale = 1e5;
  params.lm_max_iterations = 10;
  params.smoother_lag = 1.0;
  params.enable_fixed_lag_smoother = false;
  params.target_num_points = 10000;
  params.min_distance = 1.0;
  params.integrator_ivox_resolution = 1.0;
  params.integrator_ivox_min_points_dist = 0.05;
  params.integrator_ivox_lru_thresh = 100;
  params.integrator_lm_outer_loops = 3;
}

void apply_default_calibration_params(vlcal::VisualCameraCalibrationParams& params) {
  params.max_outer_iterations = 10;
  params.max_inner_iterations = 256;
  params.num_threads = 15;
  params.delta_trans_thresh = 0.005;
  params.delta_rot_thresh = 0.01 * M_PI / 180.0;
  params.disable_z_buffer_culling = false;
  params.nid_bins = 16;
  params.registration_type = vlcal::RegistrationType::NID_BFGS;
  params.nelder_mead_init_step = 1e-3;
  params.nelder_mead_convergence_criteria = 1e-8;
}

namespace {

nlohmann::json calibration_params_to_json(const vlcal::VisualCameraCalibrationParams& params) {
  nlohmann::json j;
  j["max_outer_iterations"] = params.max_outer_iterations;
  j["max_inner_iterations"] = params.max_inner_iterations;
  j["num_threads"] = params.num_threads;
  j["delta_trans_thresh"] = params.delta_trans_thresh;
  j["delta_rot_thresh"] = params.delta_rot_thresh;
  j["disable_z_buffer_culling"] = params.disable_z_buffer_culling;
  j["nid_bins"] = params.nid_bins;
  j["registration_type"] =
    (params.registration_type == vlcal::RegistrationType::NID_NELDER_MEAD) ? "nid_nelder_mead" : "nid_bfgs";
  j["nelder_mead_init_step"] = params.nelder_mead_init_step;
  j["nelder_mead_convergence_criteria"] = params.nelder_mead_convergence_criteria;
  return j;
}

void calibration_params_from_json(const nlohmann::json& j, vlcal::VisualCameraCalibrationParams& params) {
  if (!j.is_object()) {
    return;
  }
  params.max_outer_iterations = j.value("max_outer_iterations", params.max_outer_iterations);
  params.max_inner_iterations = j.value("max_inner_iterations", params.max_inner_iterations);
  params.num_threads = j.value("num_threads", params.num_threads);
  params.delta_trans_thresh = j.value("delta_trans_thresh", params.delta_trans_thresh);
  params.delta_rot_thresh = j.value("delta_rot_thresh", params.delta_rot_thresh);
  params.disable_z_buffer_culling = j.value("disable_z_buffer_culling", params.disable_z_buffer_culling);
  params.nid_bins = j.value("nid_bins", params.nid_bins);
  const std::string reg = j.value("registration_type", std::string("nid_bfgs"));
  if (reg == "nid_nelder_mead") {
    params.registration_type = vlcal::RegistrationType::NID_NELDER_MEAD;
  } else {
    params.registration_type = vlcal::RegistrationType::NID_BFGS;
  }
  params.nelder_mead_init_step = j.value("nelder_mead_init_step", params.nelder_mead_init_step);
  params.nelder_mead_convergence_criteria =
    j.value("nelder_mead_convergence_criteria", params.nelder_mead_convergence_criteria);
}

}  // namespace

void apply_default_scan_time_params(glim_cloud_fusion::ScanTimeParams& scan_time) {
  scan_time.time_unit = glim_cloud_fusion::TimeUnit::Nanoseconds;
  scan_time.time_origin = glim_cloud_fusion::TimeOrigin::ScanStart;
  scan_time.scan_duration.reset();
}

bool scan_time_duration_valid(const glim_cloud_fusion::ScanTimeParams& scan_time) {
  return scan_time.scan_duration.has_value() && *scan_time.scan_duration > 0.0 &&
         std::isfinite(*scan_time.scan_duration);
}

void SessionData::ensurePrimaryCalibration() {
  if (calibration_datasets.empty()) {
    calibration_datasets.emplace_back();
  }
}

void SessionData::ensurePrimaryInspection() {
  if (inspection_datasets.empty()) {
    inspection_datasets.emplace_back();
  }
}

bool save_session(const std::string& path, const SessionData& session_in) {
  SessionData session = session_in;
  prune_placeholder_calibration_datasets(session);

  if (!scan_time_duration_valid(session.scan_time)) {
    return false;
  }
  for (const auto& d : session.calibration_datasets) {
    if (!scan_time_duration_valid(d.fusion_params.scan_time)) {
      return false;
    }
  }

  nlohmann::json j;
  j["camera"]["model"] = session.camera.model;
  j["camera"]["intrinsics"] = session.camera.intrinsics;
  j["camera"]["distortion_coeffs"] = session.camera.distortion_coeffs;
  j["intrinsics_path"] = session.intrinsics_path;
  if (session.reference_scan_width > 0 && session.reference_scan_height > 0) {
    j["reference_scan"]["width"] = session.reference_scan_width;
    j["reference_scan"]["height"] = session.reference_scan_height;
  }
  j["global_mask"]["path"] = session.global_mask.path;
  j["global_mask"]["width"] = session.global_mask.width;
  j["global_mask"]["height"] = session.global_mask.height;
  j["global_mask"]["keep_inside"] = session.global_mask.keep_inside;
  j["global_mask"]["ops"] = nlohmann::json::array();
  for (const MaskPolygonOp& op : session.global_mask.ops) {
    j["global_mask"]["ops"].push_back(mask_polygon_op_to_json(op));
  }
  j["calibration_datasets"] = nlohmann::json::array();
  for (const auto& d : session.calibration_datasets) {
    j["calibration_datasets"].push_back(dataset_to_json(d));
  }
  j["inspection_datasets"] = nlohmann::json::array();
  for (const auto& d : session.inspection_datasets) {
    j["inspection_datasets"].push_back(inspection_dataset_to_json(d));
  }
  j["scan_time"] = scan_time_to_json(session.scan_time);
  save_pose(j, "T_camera_lidar", session.T_camera_lidar);
  save_pose(j, "T_camera_lidar_initial", session.T_camera_lidar_initial);
  j["extrinsic_available"] = session.extrinsic_available;
  j["calibration_params"] = calibration_params_to_json(session.calibration_params);
  std::ofstream ofs(path);
  if (!ofs) {
    return false;
  }
  ofs << j.dump(2);
  return true;
}

bool load_session(const std::string& path, SessionData& session, std::string& error) {
  error.clear();
  std::ifstream ifs(path);
  if (!ifs) {
    error = "cannot open file: " + path;
    return false;
  }
  nlohmann::json j;
  try {
    ifs >> j;
  } catch (const std::exception& e) {
    error = std::string("JSON parse error: ") + e.what();
    return false;
  }

  SessionData parsed;
  try {
    if (!j.contains("camera") || !j["camera"].is_object()) {
      error = "missing or invalid \"camera\" object";
      return false;
    }
    const auto& cam = j["camera"];
    if (!cam.contains("model") || !cam.contains("intrinsics") || !cam.contains("distortion_coeffs")) {
      error = "camera object missing model, intrinsics, or distortion_coeffs";
      return false;
    }
    parsed.camera.model = cam["model"].get<std::string>();
    parsed.camera.intrinsics = cam["intrinsics"].get<std::vector<double>>();
    parsed.camera.distortion_coeffs = cam["distortion_coeffs"].get<std::vector<double>>();
    parsed.intrinsics_path = j.value("intrinsics_path", "");
    if (j.contains("reference_scan")) {
      parsed.reference_scan_width = j["reference_scan"].value("width", 0);
      parsed.reference_scan_height = j["reference_scan"].value("height", 0);
    } else {
      parsed.reference_scan_width = 0;
      parsed.reference_scan_height = 0;
    }
    if (j.contains("global_mask")) {
      const auto& gm = j["global_mask"];
      if (!gm.is_object()) {
        error = "global_mask must be an object";
        return false;
      }
      parsed.global_mask.path = gm.value("path", "");
      parsed.global_mask.width = gm.value("width", 0);
      parsed.global_mask.height = gm.value("height", 0);
      parsed.global_mask.keep_inside = gm.value("keep_inside", false);
      parsed.global_mask.ops.clear();
      if (gm.contains("ops")) {
        if (!gm["ops"].is_array()) {
          error = "global_mask.ops must be an array";
          return false;
        }
        for (std::size_t i = 0; i < gm["ops"].size(); ++i) {
          MaskPolygonOp op;
          std::string op_error;
          if (!mask_polygon_op_from_json(gm["ops"][i], op, op_error)) {
            std::ostringstream oss;
            oss << "global_mask.ops[" << i << "]: " << op_error;
            error = oss.str();
            return false;
          }
          parsed.global_mask.ops.push_back(std::move(op));
        }
      }
    } else {
      parsed.global_mask = {};
    }
    parsed.calibration_datasets.clear();
    if (j.contains("calibration_datasets") && j["calibration_datasets"].is_array()) {
      const auto& arr = j["calibration_datasets"];
      for (std::size_t i = 0; i < arr.size(); ++i) {
        DatasetEntry entry;
        std::string ds_error;
        if (!calibration_dataset_from_json(arr[i], entry, ds_error)) {
          std::ostringstream oss;
          oss << "calibration_datasets[" << i << "]: " << ds_error;
          error = oss.str();
          return false;
        }
        parsed.calibration_datasets.push_back(std::move(entry));
      }
    }
    parsed.inspection_datasets.clear();
    if (j.contains("inspection_datasets") && j["inspection_datasets"].is_array()) {
      const auto& arr = j["inspection_datasets"];
      for (std::size_t i = 0; i < arr.size(); ++i) {
        DatasetEntry entry;
        std::string ds_error;
        if (!inspection_dataset_from_json(arr[i], entry, ds_error)) {
          std::ostringstream oss;
          oss << "inspection_datasets[" << i << "]: " << ds_error;
          error = oss.str();
          return false;
        }
        parsed.inspection_datasets.push_back(std::move(entry));
      }
    }
    prune_placeholder_calibration_datasets(parsed);
    apply_default_fusion_params(parsed.fusion_params);
    if (!j.contains("scan_time")) {
      error = "missing top-level scan_time";
      return false;
    }
    if (!scan_time_from_json(j["scan_time"], parsed.scan_time)) {
      error = "invalid top-level scan_time (need positive finite scan_duration)";
      return false;
    }
    load_pose(j, "T_camera_lidar", parsed.T_camera_lidar);
    load_pose(j, "T_camera_lidar_initial", parsed.T_camera_lidar_initial);
    if (j.contains("extrinsic_available")) {
      parsed.extrinsic_available = j["extrinsic_available"].get<bool>();
    } else {
      parsed.extrinsic_available = !mat4dNearRigidIdentity(parsed.T_camera_lidar) ||
                                     !mat4dNearRigidIdentity(parsed.T_camera_lidar_initial);
    }
    apply_default_calibration_params(parsed.calibration_params);
    if (j.contains("calibration_params")) {
      calibration_params_from_json(j["calibration_params"], parsed.calibration_params);
    }
    parsed.fusion_params.scan_time = parsed.scan_time;
    for (auto& d : parsed.calibration_datasets) {
      d.scan_time = d.fusion_params.scan_time;
      if (d.fusion_params.num_threads <= 0) {
        apply_default_fusion_params(d.fusion_params);
        d.fusion_params.scan_time = d.scan_time;
      }
      if (d.id.empty() && !d.pcd_paths.empty()) {
        d.id = dataset_id_from_first_scan(d.pcd_paths);
      }
      sync_dataset_fusion_status(d);
    }
    for (auto& d : parsed.inspection_datasets) {
      d.fusion_params.scan_time = parsed.scan_time;
      d.scan_time = parsed.scan_time;
      if (d.id.empty() && !d.pcd_paths.empty()) {
        d.id = dataset_id_from_first_scan(d.pcd_paths);
      }
      sync_dataset_fusion_status(d);
    }
  } catch (const std::exception& e) {
    error = std::string("session parse error: ") + e.what();
    return false;
  }

  session = std::move(parsed);
  return true;
}

bool rasterize_global_mask(
  const GlobalMaskMeta& meta,
  const std::string& session_directory,
  std::vector<std::uint8_t>& mask,
  std::string& error) {
  error.clear();
  mask.clear();
  if (meta.width <= 0 || meta.height <= 0) {
    error = "global mask dimensions are not set";
    return false;
  }
  if (!meta.ops.empty()) {
    mask = build_mask_from_ops(meta.width, meta.height, meta.ops);
    return true;
  }
  const std::string png_path = resolve_global_mask_png_path(meta, session_directory);
  if (png_path.empty()) {
    error = "global mask has no polygon ops and no PNG file on disk";
    return false;
  }
  return load_mask_png(png_path, meta.width, meta.height, mask, error);
}

bool fusion_params_from_json_file(const std::string& path, glim_cloud_fusion::FusionParams& params, std::string& error) {
  try {
    std::ifstream in(path);
    if (!in) {
      error = "cannot open file: " + path;
      return false;
    }
    nlohmann::json j;
    in >> j;
    if (j.contains("fusion_params") && j["fusion_params"].is_object() &&
        fusion_params_from_json(j["fusion_params"], params)) {
      return true;
    }
    if (j.contains("calibration_datasets") && j["calibration_datasets"].is_array() &&
        !j["calibration_datasets"].empty() && j["calibration_datasets"][0].contains("fusion_params") &&
        fusion_params_from_json(j["calibration_datasets"][0]["fusion_params"], params)) {
      return true;
    }
    if (j.is_object() && fusion_params_from_json(j, params)) {
      return true;
    }
    error = "no fusion_params object found in " + path;
    return false;
  } catch (const std::exception& e) {
    error = e.what();
    return false;
  }
}

void print_fusion_params(std::ostream& out, const glim_cloud_fusion::FusionParams& params) {
  const auto& st = params.scan_time;
  out << "  backend: " << fusion_backend_to_string(params.backend) << "\n"
      << "  voxel_resolution: " << params.voxel_resolution << "\n"
      << "  num_threads: " << params.num_threads << "\n"
      << "  max_correspondence_distance: " << params.max_correspondence_distance << "\n"
      << "  k_neighbors: " << params.k_neighbors << "\n"
      << "  ivox_resolution: " << params.ivox_resolution << "\n"
      << "  ivox_min_points_dist: " << params.ivox_min_points_dist << "\n"
      << "  ivox_lru_thresh: " << params.ivox_lru_thresh << "\n"
      << "  location_consistency_inf_scale: " << params.location_consistency_inf_scale << "\n"
      << "  constant_velocity_inf_scale: " << params.constant_velocity_inf_scale << "\n"
      << "  lm_max_iterations: " << params.lm_max_iterations << "\n"
      << "  smoother_lag: " << params.smoother_lag << "\n"
      << "  use_isam2_dogleg: " << (params.use_isam2_dogleg ? "true" : "false") << "\n"
      << "  isam2_relinearize_skip: " << params.isam2_relinearize_skip << "\n"
      << "  isam2_relinearize_thresh: " << params.isam2_relinearize_thresh << "\n"
      << "  compute_covs: " << (params.compute_covs ? "true" : "false") << "\n"
      << "  enable_fixed_lag_smoother: " << (params.enable_fixed_lag_smoother ? "true" : "false") << "\n"
      << "  target_num_points: " << params.target_num_points << "\n"
      << "  min_distance: " << params.min_distance << "\n"
      << "  integrator_ivox_resolution: " << params.integrator_ivox_resolution << "\n"
      << "  integrator_ivox_min_points_dist: " << params.integrator_ivox_min_points_dist << "\n"
      << "  integrator_ivox_lru_thresh: " << params.integrator_ivox_lru_thresh << "\n"
      << "  integrator_lm_outer_loops: " << params.integrator_lm_outer_loops << "\n"
      << "  integrator_debug: " << (params.integrator_debug ? "true" : "false") << "\n"
      << "  scan_time.time_unit: " << (st.time_unit == glim_cloud_fusion::TimeUnit::Nanoseconds ? "nanoseconds" : "seconds")
      << "\n"
      << "  scan_time.time_origin: "
      << (st.time_origin == glim_cloud_fusion::TimeOrigin::ScanStart ? "scan_start" : "first_point") << "\n";
  if (st.scan_duration.has_value()) {
    out << "  scan_time.scan_duration: " << *st.scan_duration << "\n";
  } else {
    out << "  scan_time.scan_duration: (required, not set)\n";
  }
  const auto& sm = params.slam_mapping;
  out << "  slam_mapping.submap_max_num_keyframes: " << sm.submap_max_num_keyframes << "\n"
      << "  slam_mapping.keyframe_update_strategy: " << sm.keyframe_update_strategy << "\n"
      << "  slam_mapping.max_keyframe_overlap: " << sm.max_keyframe_overlap << "\n"
      << "  slam_mapping.global_min_implicit_loop_overlap: " << sm.global_min_implicit_loop_overlap << "\n";
}

}  // namespace clc
