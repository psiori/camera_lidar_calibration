#pragma once

#include <functional>
#include <vector>

#include <camera/generic_camera_base.hpp>
#include <opencv2/core.hpp>
#include <vlcal/calib/visual_camera_calibration.hpp>
#include <vlcal/common/frame_cpu.hpp>

namespace clc {

struct AlignmentInput {
  camera::GenericCameraBase::ConstPtr camera;
  std::vector<cv::Mat> images;
  std::vector<vlcal::FrameCPU::Ptr> clouds;
  Eigen::Isometry3d init_T_camera_lidar = Eigen::Isometry3d::Identity();
  vlcal::VisualCameraCalibrationParams params;
};

using AlignmentProgressCallback = std::function<void(const vlcal::CalibrationProgress&)>;

vlcal::CalibrationResult run_alignment(const AlignmentInput& input, AlignmentProgressCallback on_progress = nullptr);

enum class InitialGuessMethod { ManualCorrespondences, FeatureMatchStub };

struct InitialGuessResult {
  bool ok = false;
  std::string message;
  Eigen::Isometry3d T_camera_lidar = Eigen::Isometry3d::Identity();
};

InitialGuessResult compute_initial_guess(InitialGuessMethod method);

}  // namespace clc
