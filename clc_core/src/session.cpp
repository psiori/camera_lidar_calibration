#include <clc_core/session.hpp>

#include <clc_core/dataset_util.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <thread>

#include <nlohmann/json.hpp>

namespace clc {

namespace {

void save_pose(nlohmann::json& j, const std::string& key, const Eigen::Isometry3d& T) {
  const Eigen::Quaterniond q(T.linear());
  const Eigen::Vector3d t = T.translation();
  j[key] = {t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w()};
}

bool load_pose(const nlohmann::json& j, const std::string& key, Eigen::Isometry3d& T) {
  if (!j.contains(key)) {
    return false;
  }
  const auto& v = j[key];
  if (!v.is_array() || v.size() != 7) {
    return false;
  }
  T.translation() << v[0], v[1], v[2];
  T.linear() = Eigen::Quaterniond(v[6], v[3], v[4], v[5]).normalized().toRotationMatrix();
  return true;
}

nlohmann::json scan_time_to_json(const glim_cloud_fusion::ScanTimeParams& scan_time) {
  nlohmann::json j;
  j["time_unit"] = static_cast<int>(scan_time.time_unit);
  j["time_origin"] = static_cast<int>(scan_time.time_origin);
  if (scan_time.scan_duration.has_value()) {
    j["scan_duration"] = *scan_time.scan_duration;
  }
  return j;
}

void scan_time_from_json(const nlohmann::json& j, glim_cloud_fusion::ScanTimeParams& scan_time) {
  if (!j.is_object()) {
    return;
  }
  scan_time.time_unit = static_cast<glim_cloud_fusion::TimeUnit>(
    j.value("time_unit", static_cast<int>(scan_time.time_unit)));
  scan_time.time_origin = static_cast<glim_cloud_fusion::TimeOrigin>(
    j.value("time_origin", static_cast<int>(scan_time.time_origin)));
  if (j.contains("scan_duration")) {
    scan_time.scan_duration = j["scan_duration"].get<double>();
  } else {
    scan_time.scan_duration.reset();
  }
}

nlohmann::json fusion_params_to_json(const glim_cloud_fusion::FusionParams& params) {
  nlohmann::json j;
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
  j["scan_time"] = scan_time_to_json(params.scan_time);
  return j;
}

void fusion_params_from_json(const nlohmann::json& j, glim_cloud_fusion::FusionParams& params) {
  if (!j.is_object()) {
    return;
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
  if (j.contains("scan_time")) {
    scan_time_from_json(j["scan_time"], params.scan_time);
  }
}

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
  return j;
}

DatasetEntry dataset_from_json(const nlohmann::json& j) {
  DatasetEntry d;
  d.id = j.value("id", "");
  d.image_path = j.value("image_path", "");
  d.cloud_directory = j.value("cloud_directory", "");
  d.fused_cloud_path = j.value("fused_cloud_path", "");
  d.pcd_paths = j.value("pcd_paths", std::vector<std::string>{});
  d.fusion_complete = j.value("fusion_complete", false);
  d.geometry_error = j.value("geometry_error", false);
  if (j.contains("fusion_params")) {
    fusion_params_from_json(j["fusion_params"], d.fusion_params);
  }
  if (j.contains("scan_time")) {
    scan_time_from_json(j["scan_time"], d.scan_time);
    d.fusion_params.scan_time = d.scan_time;
  }
  if (j.contains("correspondences") && j["correspondences"].is_array()) {
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
  return d;
}

}  // namespace

int default_fusion_num_threads() {
  const unsigned hw = std::thread::hardware_concurrency();
  return hw > 0 ? static_cast<int>(hw) : 4;
}

void apply_default_fusion_params(glim_cloud_fusion::FusionParams& params) {
  params.voxel_resolution = 0.01;
  params.num_threads = 15;
  params.max_correspondence_distance = 0.5;
  params.k_neighbors = 40;
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

std::optional<double> infer_scan_duration_seconds(
  const glim_cloud_fusion::LidarScan& scan,
  const glim_cloud_fusion::ScanTimeParams& params) {
  if (scan.times.empty()) {
    return std::nullopt;
  }
  const double max_t = *std::max_element(scan.times.begin(), scan.times.end());
  const double scale = (params.time_unit == glim_cloud_fusion::TimeUnit::Nanoseconds) ? 1e-9 : 1.0;
  const double duration = max_t * scale;
  if (duration <= 0.0 || !std::isfinite(duration)) {
    return std::nullopt;
  }
  return duration;
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
  j["calibration_datasets"] = nlohmann::json::array();
  for (const auto& d : session.calibration_datasets) {
    j["calibration_datasets"].push_back(dataset_to_json(d));
  }
  j["inspection_datasets"] = nlohmann::json::array();
  for (const auto& d : session.inspection_datasets) {
    j["inspection_datasets"].push_back(dataset_to_json(d));
  }
  j["fusion"]["voxel_resolution"] = session.fusion_params.voxel_resolution;
  j["fusion"]["num_threads"] = session.fusion_params.num_threads;
  j["fusion"]["max_correspondence_distance"] = session.fusion_params.max_correspondence_distance;
  j["fusion"]["k_neighbors"] = session.fusion_params.k_neighbors;
  j["scan_time"]["time_unit"] = static_cast<int>(session.scan_time.time_unit);
  j["scan_time"]["time_origin"] = static_cast<int>(session.scan_time.time_origin);
  if (session.scan_time.scan_duration.has_value()) {
    j["scan_time"]["scan_duration"] = *session.scan_time.scan_duration;
  }
  save_pose(j, "T_camera_lidar", session.T_camera_lidar);
  save_pose(j, "T_camera_lidar_initial", session.T_camera_lidar_initial);
  j["calibration_params"] = calibration_params_to_json(session.calibration_params);
  std::ofstream ofs(path);
  if (!ofs) {
    return false;
  }
  ofs << j.dump(2);
  return true;
}

bool load_session(const std::string& path, SessionData& session) {
  std::ifstream ifs(path);
  if (!ifs) {
    return false;
  }
  nlohmann::json j;
  ifs >> j;
  session.camera.model = j["camera"]["model"];
  session.camera.intrinsics = j["camera"]["intrinsics"].get<std::vector<double>>();
  session.camera.distortion_coeffs = j["camera"]["distortion_coeffs"].get<std::vector<double>>();
  session.intrinsics_path = j.value("intrinsics_path", "");
  if (j.contains("reference_scan")) {
    session.reference_scan_width = j["reference_scan"].value("width", 0);
    session.reference_scan_height = j["reference_scan"].value("height", 0);
  } else {
    session.reference_scan_width = 0;
    session.reference_scan_height = 0;
  }
  if (j.contains("global_mask")) {
    session.global_mask.path = j["global_mask"].value("path", "");
    session.global_mask.width = j["global_mask"].value("width", 0);
    session.global_mask.height = j["global_mask"].value("height", 0);
    session.global_mask.keep_inside = j["global_mask"].value("keep_inside", false);
  } else {
    session.global_mask = {};
  }
  session.calibration_datasets.clear();
  if (j.contains("calibration_datasets") && j["calibration_datasets"].is_array()) {
    for (const auto& d : j["calibration_datasets"]) {
      session.calibration_datasets.push_back(dataset_from_json(d));
    }
  }
  session.inspection_datasets.clear();
  if (j.contains("inspection_datasets") && j["inspection_datasets"].is_array()) {
    for (const auto& d : j["inspection_datasets"]) {
      session.inspection_datasets.push_back(dataset_from_json(d));
    }
  }
  prune_placeholder_calibration_datasets(session);
  apply_default_fusion_params(session.fusion_params);
  if (j.contains("fusion")) {
    session.fusion_params.voxel_resolution = j["fusion"].value("voxel_resolution", session.fusion_params.voxel_resolution);
    session.fusion_params.num_threads = j["fusion"].value("num_threads", session.fusion_params.num_threads);
    session.fusion_params.max_correspondence_distance =
      j["fusion"].value("max_correspondence_distance", session.fusion_params.max_correspondence_distance);
    session.fusion_params.k_neighbors = j["fusion"].value("k_neighbors", session.fusion_params.k_neighbors);
  }
  session.scan_time.time_unit = static_cast<glim_cloud_fusion::TimeUnit>(
    j["scan_time"].value("time_unit", static_cast<int>(glim_cloud_fusion::TimeUnit::Nanoseconds)));
  session.scan_time.time_origin = static_cast<glim_cloud_fusion::TimeOrigin>(
    j["scan_time"].value("time_origin", static_cast<int>(glim_cloud_fusion::TimeOrigin::ScanStart)));
  if (j["scan_time"].contains("scan_duration")) {
    session.scan_time.scan_duration = j["scan_time"]["scan_duration"].get<double>();
  }
  load_pose(j, "T_camera_lidar", session.T_camera_lidar);
  load_pose(j, "T_camera_lidar_initial", session.T_camera_lidar_initial);
  apply_default_calibration_params(session.calibration_params);
  if (j.contains("calibration_params")) {
    calibration_params_from_json(j["calibration_params"], session.calibration_params);
  }
  session.fusion_params.scan_time = session.scan_time;
  for (auto& d : session.calibration_datasets) {
    if (d.fusion_params.num_threads <= 0) {
      d.fusion_params = session.fusion_params;
    }
    if (!d.fusion_params.scan_time.scan_duration.has_value() && session.scan_time.scan_duration.has_value()) {
      d.fusion_params.scan_time = session.scan_time;
    }
    if (d.scan_time.time_unit == glim_cloud_fusion::TimeUnit::Nanoseconds &&
        d.scan_time.time_origin == glim_cloud_fusion::TimeOrigin::ScanStart && !d.scan_time.scan_duration.has_value()) {
      d.scan_time = session.scan_time;
    }
    if (d.id.empty() && !d.pcd_paths.empty()) {
      d.id = dataset_id_from_first_scan(d.pcd_paths);
    }
    sync_dataset_fusion_status(d);
  }
  return true;
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
    if (j.contains("fusion_params") && j["fusion_params"].is_object()) {
      fusion_params_from_json(j["fusion_params"], params);
      return true;
    }
    if (j.contains("calibration_datasets") && j["calibration_datasets"].is_array() &&
        !j["calibration_datasets"].empty() && j["calibration_datasets"][0].contains("fusion_params")) {
      fusion_params_from_json(j["calibration_datasets"][0]["fusion_params"], params);
      return true;
    }
    if (j.is_object() && (j.contains("voxel_resolution") || j.contains("num_threads") || j.contains("scan_time"))) {
      fusion_params_from_json(j, params);
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
  out << "  voxel_resolution: " << params.voxel_resolution << "\n"
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
      << "  scan_time.time_unit: " << (st.time_unit == glim_cloud_fusion::TimeUnit::Nanoseconds ? "nanoseconds" : "seconds")
      << "\n"
      << "  scan_time.time_origin: "
      << (st.time_origin == glim_cloud_fusion::TimeOrigin::ScanStart ? "scan_start" : "first_point") << "\n";
  if (st.scan_duration.has_value()) {
    out << "  scan_time.scan_duration: " << *st.scan_duration << "\n";
  } else {
    out << "  scan_time.scan_duration: (infer from first scan)\n";
  }
}

}  // namespace clc
