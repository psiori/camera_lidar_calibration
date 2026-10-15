// dataset_util.cpp — clc_core session, I/O, fusion, or calibration logic.

#include <clc_core/dataset_util.hpp>

#include <clc_core/pcd_io.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>

namespace clc {

std::string dataset_id_from_first_scan(const std::vector<std::string>& sorted_pcd_paths) {
  if (sorted_pcd_paths.empty()) {
    return {};
  }
  return std::filesystem::path(sorted_pcd_paths.front()).stem().string();
}

bool dataset_pickable(const DatasetEntry& dataset) {
  if (dataset.image_path.empty() || dataset.geometry_error) {
    return false;
  }
  return !dataset.pcd_paths.empty() || !dataset.cloud_directory.empty();
}

bool dataset_is_single_scan(const DatasetEntry& dataset) {
  return dataset.pcd_paths.size() == 1;
}

bool session_has_multi_scan_dataset(const SessionData& session) {
  return std::any_of(session.calibration_datasets.begin(), session.calibration_datasets.end(),
                     [](const DatasetEntry& ds) { return ds.pcd_paths.size() > 1; });
}

void session_structured_scan_reference(const SessionData& session, int& width, int& height) {
  width = 0;
  height = 0;
  if (!session_has_multi_scan_dataset(session)) {
    return;
  }
  width = session.reference_scan_width;
  height = session.reference_scan_height;
}

void clear_session_structured_scan_reference_if_no_multi_scan(SessionData& session) {
  if (!session_has_multi_scan_dataset(session)) {
    reset_session_structured_scan_reference(session);
  }
}

void reset_session_structured_scan_reference(SessionData& session) {
  session.reference_scan_width = 0;
  session.reference_scan_height = 0;
  session.global_mask = {};
}

bool session_has_calibration_datasets(const SessionData& session) {
  return std::any_of(session.calibration_datasets.begin(), session.calibration_datasets.end(),
                     [](const DatasetEntry& ds) { return !calibration_dataset_is_placeholder(ds); });
}

void sync_dataset_fusion_status(DatasetEntry& dataset) {
  if (dataset.geometry_error) {
    return;
  }
  if (!dataset.fused_cloud_path.empty()) {
    dataset.fusion_complete = true;
    return;
  }
  if (dataset.pcd_paths.size() == 1) {
    dataset.fusion_complete = true;
  } else if (dataset.pcd_paths.size() > 1) {
    dataset.fusion_complete = false;
  }
}

bool validate_dataset_image_first_scan_time(const DatasetEntry& dataset, std::string& error) {
  error.clear();
  if (dataset.pcd_paths.size() <= 1) {
    return true;
  }
  if (dataset.image_path.empty()) {
    return true;
  }

  double first_scan_stamp = 0.0;
  const auto first_scan_name = std::filesystem::path(dataset.pcd_paths.front()).filename().string();
  if (!parse_pcd_filename_stamp(first_scan_name, first_scan_stamp)) {
    error = "cannot parse UTC timestamp from first scan filename: " + dataset.pcd_paths.front();
    return false;
  }

  double image_stamp = 0.0;
  const auto image_name = std::filesystem::path(dataset.image_path).filename().string();
  if (!parse_image_filename_stamp(image_name, image_stamp)) {
    error =
      "camera image filename does not match UTC timestamp pattern "
      "(YYYY-MM-DDTHH_MM_SS_mmmZ.<png|jpg|…>): " +
      dataset.image_path;
    return false;
  }

  const double delta_seconds = std::abs(image_stamp - first_scan_stamp);
  if (delta_seconds > k_dataset_image_first_scan_max_delta_seconds) {
    const int delta_ms = static_cast<int>(std::lround(delta_seconds * 1000.0));
    error = "camera image timestamp differs from first scan by " + std::to_string(delta_ms) +
            " ms (maximum " +
            std::to_string(static_cast<int>(k_dataset_image_first_scan_max_delta_seconds * 1000.0)) +
            " ms)";
    return false;
  }
  return true;
}

bool dataset_ready(const DatasetEntry& dataset) {
  if (dataset.image_path.empty() || dataset.cloud_directory.empty()) {
    return false;
  }
  if (dataset.geometry_error) {
    return false;
  }
  if (!dataset.fused_cloud_path.empty()) {
    return true;
  }
  return dataset.fusion_complete;
}

bool calibration_dataset_is_placeholder(const DatasetEntry& dataset) {
  return dataset.image_path.empty() && dataset.cloud_directory.empty() && dataset.pcd_paths.empty() &&
         dataset.fused_cloud_path.empty() && dataset.correspondences.empty();
}

void prune_placeholder_calibration_datasets(SessionData& session) {
  const bool has_real = std::any_of(session.calibration_datasets.begin(), session.calibration_datasets.end(),
                                    [](const DatasetEntry& d) { return !calibration_dataset_is_placeholder(d); });
  if (!has_real) {
    return;
  }
  session.calibration_datasets.erase(
    std::remove_if(session.calibration_datasets.begin(), session.calibration_datasets.end(),
                   [](const DatasetEntry& d) { return calibration_dataset_is_placeholder(d); }),
    session.calibration_datasets.end());
}

bool calibration_dataset_id_unique(const SessionData& session, const std::string& id, int skip_index) {
  if (id.empty()) {
    return true;
  }
  for (int i = 0; i < static_cast<int>(session.calibration_datasets.size()); ++i) {
    if (i == skip_index) {
      continue;
    }
    if (session.calibration_datasets[static_cast<std::size_t>(i)].id == id) {
      return false;
    }
  }
  return true;
}

}  // namespace clc
