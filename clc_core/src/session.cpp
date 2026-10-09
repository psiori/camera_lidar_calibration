#include <clc_core/session.hpp>

#include <fstream>

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

nlohmann::json dataset_to_json(const DatasetEntry& d) {
  nlohmann::json j;
  j["image_path"] = d.image_path;
  j["cloud_directory"] = d.cloud_directory;
  j["fused_cloud_path"] = d.fused_cloud_path;
  j["pcd_paths"] = d.pcd_paths;
  j["correspondences"] = nlohmann::json::array();
  for (const auto& c : d.correspondences) {
    j["correspondences"].push_back({{"uv", {c.u, c.v}}, {"xyz", {c.x, c.y, c.z}}});
  }
  return j;
}

DatasetEntry dataset_from_json(const nlohmann::json& j) {
  DatasetEntry d;
  d.image_path = j.value("image_path", "");
  d.cloud_directory = j.value("cloud_directory", "");
  d.fused_cloud_path = j.value("fused_cloud_path", "");
  d.pcd_paths = j.value("pcd_paths", std::vector<std::string>{});
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

void SessionData::syncPrimaryFromLegacy() {
  ensurePrimaryCalibration();
  auto& d = calibration_datasets.front();
  d.image_path = image_path;
  d.cloud_directory = cloud_directory;
  d.pcd_paths = pcd_paths;
}

void SessionData::syncLegacyFromPrimary() {
  ensurePrimaryCalibration();
  const auto& d = calibration_datasets.front();
  image_path = d.image_path;
  cloud_directory = d.cloud_directory;
  pcd_paths = d.pcd_paths;
}

void SessionData::syncInspectionPrimaryFromLegacy() {
  ensurePrimaryInspection();
  auto& d = inspection_datasets.front();
  d.image_path = image_path;
  d.cloud_directory = cloud_directory;
  d.pcd_paths = pcd_paths;
}

void SessionData::syncLegacyFromInspectionPrimary() {
  ensurePrimaryInspection();
  const auto& d = inspection_datasets.front();
  image_path = d.image_path;
  cloud_directory = d.cloud_directory;
  pcd_paths = d.pcd_paths;
}

bool save_session(const std::string& path, const SessionData& session_in) {
  SessionData session = session_in;
  session.syncPrimaryFromLegacy();

  nlohmann::json j;
  j["camera"]["model"] = session.camera.model;
  j["camera"]["intrinsics"] = session.camera.intrinsics;
  j["camera"]["distortion_coeffs"] = session.camera.distortion_coeffs;
  j["cloud_directory"] = session.cloud_directory;
  j["image_path"] = session.image_path;
  j["intrinsics_path"] = session.intrinsics_path;
  if (session.reference_scan_width > 0 && session.reference_scan_height > 0) {
    j["reference_scan"]["width"] = session.reference_scan_width;
    j["reference_scan"]["height"] = session.reference_scan_height;
  }
  j["pcd_paths"] = session.pcd_paths;
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
  session.cloud_directory = j.value("cloud_directory", "");
  session.image_path = j.value("image_path", "");
  session.intrinsics_path = j.value("intrinsics_path", "");
  if (j.contains("reference_scan")) {
    session.reference_scan_width = j["reference_scan"].value("width", 0);
    session.reference_scan_height = j["reference_scan"].value("height", 0);
  } else {
    session.reference_scan_width = 0;
    session.reference_scan_height = 0;
  }
  session.pcd_paths = j.value("pcd_paths", std::vector<std::string>{});
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
  if (session.calibration_datasets.empty()) {
    session.syncPrimaryFromLegacy();
  } else {
    session.syncLegacyFromPrimary();
  }
  session.fusion_params.voxel_resolution = j["fusion"].value("voxel_resolution", 0.05);
  session.fusion_params.num_threads = j["fusion"].value("num_threads", 4);
  session.scan_time.time_unit = static_cast<glim_cloud_fusion::TimeUnit>(j["scan_time"].value("time_unit", 0));
  session.scan_time.time_origin = static_cast<glim_cloud_fusion::TimeOrigin>(j["scan_time"].value("time_origin", 0));
  if (j["scan_time"].contains("scan_duration")) {
    session.scan_time.scan_duration = j["scan_time"]["scan_duration"].get<double>();
  }
  load_pose(j, "T_camera_lidar", session.T_camera_lidar);
  load_pose(j, "T_camera_lidar_initial", session.T_camera_lidar_initial);
  session.fusion_params.scan_time = session.scan_time;
  return true;
}

}  // namespace clc
