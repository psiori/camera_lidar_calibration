#pragma once

#include <iosfwd>
#include <optional>
#include <string>
#include <vector>

#include <clc/linalg.hpp>
#include <opencv2/core.hpp>

#include <glim_cloud_fusion/fusion.hpp>
#include <vlcal/calib/visual_camera_calibration.hpp>

namespace clc {

/// Camera model name and coefficient vectors as stored in session JSON.
struct CameraIntrinsics {
  std::string model = "plumb_bob";
  std::vector<double> intrinsics;
  std::vector<double> distortion_coeffs;
};

/// One matched pair: pixel (u,v) on the camera image and 3D point (x,y,z) in lidar frame.
struct PointCorrespondence {
  double u = 0.0;
  double v = 0.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

/// One calibration or inspection dataset: image, scans, fusion state, and manual correspondences.
struct DatasetEntry {
  std::string id;
  std::string image_path;
  std::string cloud_directory;
  std::string fused_cloud_path;
  std::vector<std::string> pcd_paths;
  std::vector<PointCorrespondence> correspondences;
  glim_cloud_fusion::FusionParams fusion_params{};
  glim_cloud_fusion::ScanTimeParams scan_time{};
  bool fusion_complete = false;
  bool geometry_error = false;
};

/// Global range-image mask file and raster dimensions shared across datasets.
struct GlobalMaskMeta {
  std::string path;
  int width = 0;
  int height = 0;
  bool keep_inside = false;
};

/// Full on-disk session: intrinsics, fusion/calibration params, poses, masks, and dataset lists.
struct SessionData {
  CameraIntrinsics camera;
  vlcal::VisualCameraCalibrationParams calibration_params;
  glim_cloud_fusion::FusionParams fusion_params;
  glim_cloud_fusion::ScanTimeParams scan_time;
  Mat4d T_camera_lidar = Mat4d::identity();
  Mat4d T_camera_lidar_initial = Mat4d::identity();
  std::string intrinsics_path;
  int reference_scan_width = 0;
  int reference_scan_height = 0;
  GlobalMaskMeta global_mask;
  std::vector<DatasetEntry> calibration_datasets;
  std::vector<DatasetEntry> inspection_datasets;

  /// Ensure at least one calibration dataset row exists.
  void ensurePrimaryCalibration();
  /// Ensure at least one inspection dataset row exists.
  void ensurePrimaryInspection();
};

bool save_session(const std::string& path, const SessionData& session);
/// Loads session state from `session.json`. On failure, `error` describes what went wrong.
bool load_session(const std::string& path, SessionData& session, std::string& error);

/// Default fusion thread count (hardware concurrency, minimum 1).
int default_fusion_num_threads();
void apply_default_fusion_params(glim_cloud_fusion::FusionParams& params);
void apply_default_scan_time_params(glim_cloud_fusion::ScanTimeParams& scan_time);
bool scan_time_duration_valid(const glim_cloud_fusion::ScanTimeParams& scan_time);
/// Parse a `fusion_params` object or a full `session.json` (uses top-level or first calibration dataset).
bool fusion_params_from_json_file(const std::string& path, glim_cloud_fusion::FusionParams& params, std::string& error);
void print_fusion_params(std::ostream& out, const glim_cloud_fusion::FusionParams& params);
/// Defaults aligned with vlcal `calibrate` / team preprocess CLI (NID + fusion voxel).
void apply_default_calibration_params(vlcal::VisualCameraCalibrationParams& params);

}  // namespace clc
