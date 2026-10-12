#include <clc_core/calibration_service.hpp>

#include <stdexcept>

#include <vlcal/common/estimate_pose.hpp>
#include <vlcal/common/visual_lidar_data.hpp>

namespace clc {

vlcal::CalibrationResult run_alignment(const AlignmentInput& input, AlignmentProgressCallback on_progress) {
  if (!input.camera || input.images.empty() || input.clouds.empty()) {
    vlcal::CalibrationResult result;
    result.error = vlcal::CalibrationError::EmptyDataset;
    result.message = "alignment input incomplete";
    return result;
  }

  std::vector<vlcal::VisualLiDARData::ConstPtr> dataset;
  dataset.reserve(input.images.size());
  for (size_t i = 0; i < input.images.size(); ++i) {
    dataset.emplace_back(std::make_shared<vlcal::VisualLiDARData>(input.images[i], input.clouds[i]));
  }

  vlcal::VisualCameraCalibrationParams params = input.params;
  params.on_progress = on_progress;
  vlcal::VisualCameraCalibration calib(input.camera, dataset, params);
  return calib.calibrate(input.init_T_camera_lidar);
}

InitialGuessResult compute_manual_initial_guess(const ManualGuessInput& input) {
  InitialGuessResult result;
  if (!input.camera) {
    result.message = "camera model required";
    return result;
  }
  if (input.correspondences.size() < 3) {
    result.ok = true;
    result.T_camera_lidar = input.gizmo_T_camera_lidar;
    result.message = "using gizmo pose (need 3+ correspondences to estimate)";
    return result;
  }

  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>> pairs;
  pairs.reserve(input.correspondences.size());
  for (const auto& c : input.correspondences) {
    pairs.emplace_back(Eigen::Vector2d(c.u, c.v), Eigen::Vector4d(c.x, c.y, c.z, 1.0));
  }

  vlcal::PoseEstimationParams pose_params;
  pose_params.trust_correspondences = true;
  vlcal::PoseEstimation estimator(pose_params);
  try {
    result.T_camera_lidar = estimator.estimate(input.camera, pairs);
    result.ok = true;
    result.message = "estimated from correspondences";
  } catch (const std::exception& e) {
    result.ok = false;
    result.message = e.what();
  }
  return result;
}

InitialGuessResult compute_initial_guess(InitialGuessMethod method) {
  InitialGuessResult result;
  if (method == InitialGuessMethod::FeatureMatchStub) {
    result.ok = false;
    result.message = "ALIKED + LightGlue integration not implemented";
    return result;
  }
  result.ok = true;
  return result;
}

}  // namespace clc
