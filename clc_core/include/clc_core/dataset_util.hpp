#pragma once

#include <string>

#include <clc_core/session.hpp>

namespace clc {

/// Dataset id from the first PCD filename (lexicographic / chronological scan order).
std::string dataset_id_from_first_scan(const std::vector<std::string>& sorted_pcd_paths);

/// Whether a calibration dataset is ready for initial alignment and NID.
bool dataset_ready(const DatasetEntry& dataset);

/// Image + scan catalog present; correspondence picking does not require fused cloud.
bool dataset_pickable(const DatasetEntry& dataset);

bool calibration_dataset_id_unique(const SessionData& session, const std::string& id, int skip_index = -1);

/// Empty slot created before any image/LiDAR paths are set (legacy session bootstrap).
bool calibration_dataset_is_placeholder(const DatasetEntry& dataset);

/// Drops placeholder rows when at least one real calibration dataset exists.
void prune_placeholder_calibration_datasets(SessionData& session);

}  // namespace clc
