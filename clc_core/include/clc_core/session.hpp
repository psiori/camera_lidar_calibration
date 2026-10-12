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

struct PointCorrespondence {
  double u = 0.0;
  double v = 0.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct DatasetEntry {
  std::string image_path;
  std::string cloud_directory;
  std::string fused_cloud_path;
  std::vector<std::string> pcd_paths;
  std::vector<PointCorrespondence> correspondences;
};

struct GlobalMaskMeta {
  std::string path;
  int width = 0;
  int height = 0;
  bool keep_inside = false;
};

struct SessionData {
  CameraIntrinsics camera;
  vlcal::VisualCameraCalibrationParams calibration_params;
  glim_cloud_fusion::FusionParams fusion_params;
  glim_cloud_fusion::ScanTimeParams scan_time;
  Eigen::Isometry3d T_camera_lidar = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_camera_lidar_initial = Eigen::Isometry3d::Identity();
  std::string intrinsics_path;
  int reference_scan_width = 0;
  int reference_scan_height = 0;
  GlobalMaskMeta global_mask;
  std::vector<DatasetEntry> calibration_datasets;
  std::vector<DatasetEntry> inspection_datasets;

  void ensurePrimaryCalibration();
  void ensurePrimaryInspection();
};

bool save_session(const std::string& path, const SessionData& session);
bool load_session(const std::string& path, SessionData& session);

/// Default fusion thread count (hardware concurrency, minimum 1).
int default_fusion_num_threads();
void apply_default_fusion_params(glim_cloud_fusion::FusionParams& params);
void apply_default_scan_time_params(glim_cloud_fusion::ScanTimeParams& scan_time);

/// Sweep length in seconds from per-point timestamps (e.g. max time when origin is scan start).
std::optional<double> infer_scan_duration_seconds(
  const glim_cloud_fusion::LidarScan& scan,
  const glim_cloud_fusion::ScanTimeParams& params);

}  // namespace clc
