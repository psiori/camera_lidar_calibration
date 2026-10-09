#pragma once

#include <string>

#include <clc_core/session.hpp>

namespace clc {

/// Load camera model and parameters from an OpenCV YAML/XML file (cv::FileStorage).
bool load_camera_intrinsics_file(const std::string& path, CameraIntrinsics& camera, std::string& error);

}  // namespace clc
