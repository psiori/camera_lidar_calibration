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
  params.num_threads = default_fusion_num_threads();
  params.max_correspondence_distance = 0.5;
}

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
  j["scan_time"]["time_unit"] = static_cast<int>(session.scan_time.time_unit);
  j["scan_time"]["time_origin"] = static_cast<int>(session.scan_time.time_origin);
  if (session.scan_time.scan_duration.has_value()) {
    j["scan_time"]["scan_duration"] = *session.scan_time.scan_duration;
  }
  save_pose(j, "T_camera_lidar", session.T_camera_lidar);
  save_pose(j, "T_camera_lidar_initial", session.T_camera_lidar_initial);
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
  session.fusion_params.voxel_resolution = j["fusion"].value("voxel_resolution", 0.05);
  session.fusion_params.num_threads = j["fusion"].value("num_threads", default_fusion_num_threads());
  session.fusion_params.max_correspondence_distance = j["fusion"].value("max_correspondence_distance", 0.5);
  session.scan_time.time_unit = static_cast<glim_cloud_fusion::TimeUnit>(
    j["scan_time"].value("time_unit", static_cast<int>(glim_cloud_fusion::TimeUnit::Nanoseconds)));
  session.scan_time.time_origin = static_cast<glim_cloud_fusion::TimeOrigin>(
    j["scan_time"].value("time_origin", static_cast<int>(glim_cloud_fusion::TimeOrigin::ScanStart)));
  if (j["scan_time"].contains("scan_duration")) {
    session.scan_time.scan_duration = j["scan_time"]["scan_duration"].get<double>();
  }
  load_pose(j, "T_camera_lidar", session.T_camera_lidar);
  load_pose(j, "T_camera_lidar_initial", session.T_camera_lidar_initial);
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

}  // namespace clc
