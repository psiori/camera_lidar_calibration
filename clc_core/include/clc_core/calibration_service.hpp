#pragma once

#include <functional>
#include <vector>

#include <clc/linalg.hpp>
#include <clc_core/session.hpp>

#include <camera/generic_camera_base.hpp>
#include <opencv2/core.hpp>
#include <vlcal/calib/visual_camera_calibration.hpp>
#include <vlcal/common/frame_cpu.hpp>

namespace clc {

struct AlignmentInput {
  camera::GenericCameraBase::ConstPtr camera;
  std::vector<cv::Mat> images;
  std::vector<vlcal::FrameCPU::Ptr> clouds;
  /// Optional per-dataset masks (255 = include); empty vector or empty inner vector = no mask.
  std::vector<std::vector<std::uint8_t>> image_masks;
  Mat4d init_T_camera_lidar = Mat4d::identity();
  vlcal::VisualCameraCalibrationParams params;
};

/// Returns false and sets `error` when any mask size mismatches its paired image.
bool validate_alignment_image_masks(const AlignmentInput& input, std::string& error);

using AlignmentProgressCallback = std::function<void(const vlcal::CalibrationProgress&)>;
using AlignmentCancellationCheck = std::function<bool()>;

vlcal::CalibrationResult run_alignment(
  const AlignmentInput& input,
  AlignmentProgressCallback on_progress = nullptr,
  AlignmentCancellationCheck is_cancelled = nullptr);

enum class InitialGuessMethod { ManualCorrespondences, FeatureMatchStub };

struct InitialGuessResult {
  bool ok = false;
  std::string message;
  Mat4d T_camera_lidar = Mat4d::identity();
};

struct ManualGuessInput {
  camera::GenericCameraBase::ConstPtr camera;
  std::vector<PointCorrespondence> correspondences;
  Mat4d gizmo_T_camera_lidar = Mat4d::identity();
};

InitialGuessResult compute_manual_initial_guess(const ManualGuessInput& input);

InitialGuessResult compute_initial_guess(InitialGuessMethod method);

}  // namespace clc
