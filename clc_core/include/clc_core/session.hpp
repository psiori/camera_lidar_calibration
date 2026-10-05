#pragma once

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <glim_cloud_fusion/fusion.hpp>
#include <vlcal/calib/visual_camera_calibration.hpp>

namespace clc {

struct CameraIntrinsics {
  std::string model = "plumb_bob";
  std::vector<double> intrinsics;
  std::vector<double> distortion_coeffs;
};

struct SessionData {
  CameraIntrinsics camera;
  vlcal::VisualCameraCalibrationParams calibration_params;
  glim_cloud_fusion::FusionParams fusion_params;
  glim_cloud_fusion::ScanTimeParams scan_time;
  Eigen::Isometry3d T_camera_lidar = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_camera_lidar_initial = Eigen::Isometry3d::Identity();
  std::string cloud_directory;
  std::string image_path;
  std::vector<std::string> pcd_paths;
};

bool save_session(const std::string& path, const SessionData& session);
bool load_session(const std::string& path, SessionData& session);

}  // namespace clc
