#pragma once

namespace psiviz {

/// Pinhole frustum drawn in the camera image frame (+X right, +Y down, +Z optical axis).
struct CameraFrustumParams {
  bool valid{false};
  float fx{0.f};
  float fy{0.f};
  float cx{0.f};
  float cy{0.f};
  float image_width{0.f};
  float image_height{0.f};
  /// Extent along +Z from the camera origin in meters.
  float depth_m{1.f};
};

/// Non-interactive calibration overlays composited with the point cloud.
struct CalibrationSceneOverlayState {
  bool camera_triad{false};
  bool lidar_triad{false};
  CameraFrustumParams frustum{};
};

}  // namespace psiviz
