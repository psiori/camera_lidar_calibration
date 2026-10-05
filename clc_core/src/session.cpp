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

}  // namespace

bool save_session(const std::string& path, const SessionData& session) {
  nlohmann::json j;
  j["camera"]["model"] = session.camera.model;
  j["camera"]["intrinsics"] = session.camera.intrinsics;
  j["camera"]["distortion_coeffs"] = session.camera.distortion_coeffs;
  j["cloud_directory"] = session.cloud_directory;
  j["image_path"] = session.image_path;
  j["pcd_paths"] = session.pcd_paths;
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
  session.pcd_paths = j.value("pcd_paths", std::vector<std::string>{});
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
