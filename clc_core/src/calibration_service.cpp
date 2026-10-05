#include <clc_core/calibration_service.hpp>

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
