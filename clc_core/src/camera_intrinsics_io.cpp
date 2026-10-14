// camera_intrinsics_io.cpp — clc_core session, I/O, fusion, or calibration logic.

#include <clc_core/camera_intrinsics_io.hpp>

#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>

#include <unordered_set>

namespace clc {

namespace {

/// Returns true if the string is a projection/distortion model name understood by vlcal.
bool is_vlcal_projection_model(const std::string& model) {
  static const std::unordered_set<std::string> k_models = {
      "plumb_bob",
      "pinhole",
      "opencv",
      "fisheye",
      "equidistant",
      "atan",
      "omnidir",
      "equirectangular",
      "rational_polynomial",
  };
  return k_models.count(model) > 0;
}

bool read_string_node(cv::FileNode node, std::string& out) {
  if (node.empty()) {
    return false;
  }
  if (node.isString()) {
    out = static_cast<std::string>(node);
    return true;
  }
  return false;
}

bool mat_to_intrinsics(const cv::Mat& k, std::vector<double>& intrinsics) {
  if (k.empty() || k.rows != 3 || k.cols != 3) {
    return false;
  }
  const double fx = k.at<double>(0, 0);
  const double fy = k.at<double>(1, 1);
  const double cx = k.at<double>(0, 2);
  const double cy = k.at<double>(1, 2);
  intrinsics = {fx, fy, cx, cy};
  return true;
}

bool node_to_distortion(const cv::FileNode& node, std::vector<double>& distortion) {
  if (node.empty()) {
    return false;
  }
  if (node.isSeq()) {
    distortion.clear();
    for (auto it = node.begin(); it != node.end(); ++it) {
      distortion.push_back(static_cast<double>(*it));
    }
    return !distortion.empty();
  }
  const cv::Mat d = node.mat();
  if (!d.empty()) {
    distortion.clear();
    for (int r = 0; r < d.rows; ++r) {
      for (int c = 0; c < d.cols; ++c) {
        distortion.push_back(d.at<double>(r, c));
      }
    }
    return !distortion.empty();
  }
  return false;
}

/// Map aliases to vlcal model names; unknown strings are not changed here.
std::string normalize_camera_model(std::string model) {
  if (model == "pinhole" || model == "opencv" || model.empty()) {
    return "plumb_bob";
  }
  return model;
}

/**
 * Choose the vlcal projection model from YAML fields.
 * `camera_model` is often a hardware label for documentation; `distortion_model` is the OpenCV model name.
 */
std::string resolve_projection_model(const std::string& camera_model_field,
                                     const std::string& distortion_model_field,
                                     const std::string& nested_camera_model_field) {
  if (!camera_model_field.empty() && is_vlcal_projection_model(camera_model_field)) {
    return normalize_camera_model(camera_model_field);
  }
  if (!distortion_model_field.empty()) {
    return normalize_camera_model(distortion_model_field);
  }
  if (!nested_camera_model_field.empty() && is_vlcal_projection_model(nested_camera_model_field)) {
    return normalize_camera_model(nested_camera_model_field);
  }
  return "plumb_bob";
}

}  // namespace

bool load_camera_intrinsics_file(const std::string& path, CameraIntrinsics& camera, std::string& error) {
  cv::FileStorage fs(path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    error = "failed to open intrinsics file: " + path;
    return false;
  }

  std::string camera_model_field;
  std::string distortion_model_field;
  std::string nested_camera_model_field;
  read_string_node(fs["camera_model"], camera_model_field);
  read_string_node(fs["distortion_model"], distortion_model_field);
  read_string_node(fs["camera"]["camera_model"], nested_camera_model_field);
  camera.model = resolve_projection_model(camera_model_field, distortion_model_field, nested_camera_model_field);

  cv::Mat k;
  if (!fs["camera_matrix"].empty()) {
    fs["camera_matrix"] >> k;
  } else if (!fs["K"].empty()) {
    fs["K"] >> k;
  } else if (!fs["camera"]["intrinsics"].empty()) {
    const auto& node = fs["camera"]["intrinsics"];
    if (node.isSeq()) {
      camera.intrinsics.clear();
      for (auto it = node.begin(); it != node.end(); ++it) {
        camera.intrinsics.push_back(static_cast<double>(*it));
      }
    }
  }

  if (!k.empty()) {
    if (!mat_to_intrinsics(k, camera.intrinsics)) {
      error = "invalid camera_matrix in: " + path;
      return false;
    }
  }

  if (camera.intrinsics.size() != 4) {
    error = "expected 4 intrinsic parameters (fx, fy, cx, cy) in: " + path;
    return false;
  }

  if (!node_to_distortion(fs["distortion_coefficients"], camera.distortion_coeffs)) {
    if (!node_to_distortion(fs["D"], camera.distortion_coeffs)) {
      if (!node_to_distortion(fs["distortion_coeffs"], camera.distortion_coeffs)) {
        if (!node_to_distortion(fs["camera"]["distortion_coeffs"], camera.distortion_coeffs)) {
          camera.distortion_coeffs = {0, 0, 0, 0, 0};
        }
      }
    }
  }

  return true;
}

}  // namespace clc
