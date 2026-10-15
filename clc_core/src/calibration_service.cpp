// calibration_service.cpp — clc_core session, I/O, fusion, or calibration logic.

#include <clc_core/calibration_service.hpp>

#include <cstring>
#include <stdexcept>

#include <Eigen/Core>

#include "eigen_adapters.hpp"
#include <vlcal/common/estimate_pose.hpp>
#include <vlcal/common/visual_lidar_data.hpp>

namespace clc {

bool validate_alignment_image_masks(const AlignmentInput& input, std::string& error) {
  error.clear();
  if (input.image_masks.empty()) {
    return true;
  }
  if (input.image_masks.size() != input.images.size()) {
    error = "image mask count does not match image count";
    return false;
  }
  for (std::size_t i = 0; i < input.images.size(); ++i) {
    const auto& mask = input.image_masks[i];
    if (mask.empty()) {
      continue;
    }
    const std::size_t expected =
      static_cast<std::size_t>(input.images[i].cols) * static_cast<std::size_t>(input.images[i].rows);
    if (mask.size() != expected) {
      error = "image mask dimensions do not match dataset image";
      return false;
    }
  }
  return true;
}

vlcal::CalibrationResult run_alignment(
  const AlignmentInput& input,
  AlignmentProgressCallback on_progress,
  AlignmentCancellationCheck is_cancelled) {
  if (!input.camera || input.images.empty() || input.clouds.empty()) {
    vlcal::CalibrationResult result;
    result.error = vlcal::CalibrationError::EmptyDataset;
    result.message = "alignment input incomplete";
    return result;
  }

  std::string mask_error;
  if (!validate_alignment_image_masks(input, mask_error)) {
    vlcal::CalibrationResult result;
    result.error = vlcal::CalibrationError::InvalidArgument;
    result.message = mask_error;
    return result;
  }

  std::vector<vlcal::VisualLiDARData::ConstPtr> dataset;
  dataset.reserve(input.images.size());
  for (size_t i = 0; i < input.images.size(); ++i) {
    auto entry = std::make_shared<vlcal::VisualLiDARData>(input.images[i], input.clouds[i]);
    if (i < input.image_masks.size() && !input.image_masks[i].empty()) {
      cv::Mat mask(input.images[i].rows, input.images[i].cols, CV_8UC1);
      std::memcpy(mask.data, input.image_masks[i].data(), input.image_masks[i].size());
      entry->image_mask = mask;
    }
    dataset.emplace_back(std::move(entry));
  }

  vlcal::VisualCameraCalibrationParams params = input.params;
  params.on_progress = on_progress;
  params.is_cancelled = is_cancelled;
  vlcal::VisualCameraCalibration calib(input.camera, dataset, params);
  return calib.calibrate(detail::toIsometry(input.init_T_camera_lidar));
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
    result.T_camera_lidar = detail::fromIsometry(estimator.estimate(input.camera, pairs));
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
