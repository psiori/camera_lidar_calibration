#pragma once

#include <string>
#include <vector>

#include <clc_core/session.hpp>

namespace clc {

/// Dataset id from the first PCD filename (lexicographic / chronological scan order).
std::string dataset_id_from_first_scan(const std::vector<std::string>& sorted_pcd_paths);

/// True when the catalog is exactly one PCD (fusion passthrough is implicit).
bool dataset_is_single_scan(const DatasetEntry& dataset);

/// True when any calibration dataset has more than one scan (structured multi-scan fusion).
bool session_has_multi_scan_dataset(const SessionData& session);

/// Session reference WxH for validating structured scan folders; (0,0) if none apply (e.g. only single scans).
void session_structured_scan_reference(const SessionData& session, int& width, int& height);

/// Clears `reference_scan_*` when no multi-scan calibration datasets remain.
void clear_session_structured_scan_reference_if_no_multi_scan(SessionData& session);

/// Clears structured-scan session reference and global mask metadata (no datasets / fresh catalog).
void reset_session_structured_scan_reference(SessionData& session);

/// True when at least one non-placeholder calibration dataset exists.
bool session_has_calibration_datasets(const SessionData& session);

/// Updates `fusion_complete` from fused path / scan count (single scan counts as fused).
void sync_dataset_fusion_status(DatasetEntry& dataset);

/// True when `fused_cloud_path` is set and the file exists on disk.
bool dataset_fused_cloud_file_present(const DatasetEntry& dataset);

/// Whether a calibration dataset is ready for initial alignment and NID.
bool dataset_ready(const DatasetEntry& dataset);

/// Human-readable blockers when `dataset_ready` is false (empty when ready).
std::vector<std::string> dataset_not_ready_reasons(const DatasetEntry& dataset);

/// Max allowed |image time − first scan time| for multi-scan datasets (seconds). This is more of a sanity check
constexpr double k_dataset_image_first_scan_max_delta_seconds = 2;

/// For multi-scan datasets, require the image filename UTC time within
/// `k_dataset_image_first_scan_max_delta_seconds` of the first scan. Single-scan datasets skip this check.
bool validate_dataset_image_first_scan_time(const DatasetEntry& dataset, std::string& error);

/// Image + scan catalog present; correspondence picking does not require fused cloud.
bool dataset_pickable(const DatasetEntry& dataset);

bool calibration_dataset_id_unique(const SessionData& session, const std::string& id, int skip_index = -1);

/// Empty slot created before any image/LiDAR paths are set (legacy session bootstrap).
bool calibration_dataset_is_placeholder(const DatasetEntry& dataset);

/// Drops placeholder rows when at least one real calibration dataset exists.
void prune_placeholder_calibration_datasets(SessionData& session);

/// Removes all placeholder rows (e.g. after delete or before adding a real dataset).
void erase_placeholder_calibration_datasets(SessionData& session);

/// Clears `id` on placeholders; assigns id from the first scan when paths are present.
void sync_calibration_dataset_identity(DatasetEntry& dataset);

}  // namespace clc
