#include "app_controller.hpp"

#include <QMetaObject>
#include <QPointF>
#include <QVariantMap>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <optional>

#include <Eigen/Geometry>
#include <opencv2/imgcodecs.hpp>

#include <QFileDialog>
#include <QFileInfo>
#include <camera/create_camera.hpp>

#include <clc_core/calibration_providers.hpp>
#include <clc_core/calibration_service.hpp>
#include <clc_core/camera_intrinsics_io.hpp>
#include <clc_core/colormap.hpp>
#include <clc_core/dataset_util.hpp>
#include <clc_core/fusion_service.hpp>
#include <clc_core/nid_cloud_prep.hpp>
#include <clc_core/pcd_io.hpp>
#include <clc_core/point_color_model.hpp>
#include <clc_core/range_image.hpp>
#include <clc_core/scan_geometry.hpp>

#include <psiviz/point_cloud_item.hpp>

namespace {

std::optional<QPointF> variantToPoint(const QVariant &value) {
  if (value.canConvert<QPointF>()) {
    return value.toPointF();
  }
  const QVariantMap map = value.toMap();
  if (map.contains(QLatin1String("x")) && map.contains(QLatin1String("y"))) {
    return QPointF(map.value(QLatin1String("x")).toDouble(),
                   map.value(QLatin1String("y")).toDouble());
  }
  return std::nullopt;
}

clc::Polygon2f polygonFromVariantList(const QVariantList &native_vertices) {
  clc::Polygon2f polygon;
  for (const QVariant &v : native_vertices) {
    const auto point = variantToPoint(v);
    if (!point.has_value()) {
      continue;
    }
    polygon.vertices.emplace_back(static_cast<float>(point->x()),
                                  static_cast<float>(point->y()));
  }
  return polygon;
}

QVariantList isometryToPoseMatrix(const Eigen::Isometry3d &T) {
  QVariantList out;
  const Eigen::Matrix4d m = T.matrix();
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      out.append(m(r, c));
    }
  }
  return out;
}

QImage rangeImageToDisplay(const clc::RangeImage &image,
                           clc::RangeImageChannel channel, float min_v,
                           float max_v, double y_stretch) {
  if (image.width <= 0 || image.height <= 0) {
    return {};
  }
  const std::vector<float> &values =
      channel == clc::RangeImageChannel::Range ? image.range : image.intensity;
  if (values.empty()) {
    return {};
  }
  std::vector<std::uint8_t> rgb;
  clc::colormap_jet(values, min_v, max_v, rgb);
  QImage out(image.width, image.height, QImage::Format_RGB888);
  if (static_cast<int>(rgb.size()) == image.width * image.height * 3) {
    std::memcpy(out.bits(), rgb.data(), rgb.size());
  }
  if (y_stretch > 0.0 && !qFuzzyCompare(y_stretch, 1.0)) {
    const int out_h =
        std::max(1, static_cast<int>(std::lround(
                        static_cast<double>(image.height) * y_stretch)));
    out = out.scaled(image.width, out_h, Qt::IgnoreAspectRatio,
                     Qt::FastTransformation);
  }
  return out;
}

float autoMaxValue(const std::vector<float> &values) {
  float max_v = 0.f;
  for (float v : values) {
    if (std::isfinite(v)) {
      max_v = std::max(max_v, v);
    }
  }
  return max_v > 0.f ? max_v : 1.f;
}

QString normalizedDirectoryPath(const QString &path) {
  if (path.isEmpty()) {
    return path;
  }
  QFileInfo info(path);
  if (info.isDir()) {
    return info.absoluteFilePath();
  }
  return info.absolutePath();
}

QString normalizedPcdPath(const QString &path) {
  if (path.isEmpty()) {
    return path;
  }
  const QFileInfo info(path);
  if (info.isFile()) {
    return info.absoluteFilePath();
  }
  if (info.isDir()) {
    return info.absoluteFilePath();
  }
  if (path.endsWith(QStringLiteral(".pcd"), Qt::CaseInsensitive)) {
    return info.absoluteFilePath();
  }
  return info.absoluteFilePath();
}

QString pickPcdFileDialog(const QString &title, const QString &start_path) {
  const QFileInfo start_info(start_path);
  QString dialog_start = normalizedDirectoryPath(start_path);
  if (start_info.isFile()) {
    dialog_start = start_info.absoluteFilePath();
  }
  return QFileDialog::getOpenFileName(
      nullptr, title, dialog_start,
      QStringLiteral("PCD files (*.pcd);;All files (*)"));
}

QString pickPcdFolderDialog(const QString &title, const QString &start_path) {
  return QFileDialog::getExistingDirectory(nullptr, title,
                                           normalizedDirectoryPath(start_path));
}

} // namespace

AppController::AppController(QObject *parent) : QObject(parent) {
  clc::apply_default_fusion_params(session_.fusion_params);
  clc::apply_default_scan_time_params(session_.scan_time);
  clc::apply_default_calibration_params(session_.calibration_params);
  session_.fusion_params.scan_time = session_.scan_time;
  viewer_colormap_debounce_.setSingleShot(true);
  viewer_colormap_debounce_.setInterval(50);
  connect(&viewer_colormap_debounce_, &QTimer::timeout, this,
          &AppController::onViewerColormapDebounce);
  range_colormap_debounce_.setSingleShot(true);
  range_colormap_debounce_.setInterval(50);
  connect(&range_colormap_debounce_, &QTimer::timeout, this,
          &AppController::onRangeColormapDebounce);
  connect(this, &AppController::correspondencesChanged, this,
          &AppController::syncCorrespondenceMarkers);
}

AppController::~AppController() { joinWorker(); }

void AppController::joinWorker() {
  if (worker_ && worker_->joinable()) {
    worker_->join();
  }
  worker_.reset();
}

void AppController::setWorkerRunning(bool running) {
  const bool was = worker_running_.exchange(running);
  if (was != running) {
    emit busyWorkingChanged();
    refreshViewerGizmoState();
  }
}

double AppController::scanDuration() const {
  if (main_work_tab_ == 3) {
    return session_.scan_time.scan_duration.value_or(0.0);
  }
  return activeDatasetEntry().scan_time.scan_duration.value_or(0.0);
}

int AppController::timeOrigin() const {
  if (main_work_tab_ == 3) {
    return static_cast<int>(session_.scan_time.time_origin);
  }
  return static_cast<int>(activeDatasetEntry().scan_time.time_origin);
}

int AppController::timeUnit() const {
  if (main_work_tab_ == 3) {
    return static_cast<int>(session_.scan_time.time_unit);
  }
  return static_cast<int>(activeDatasetEntry().scan_time.time_unit);
}

void AppController::setScanDuration(double seconds) {
  if (main_work_tab_ == 3) {
    session_.scan_time.scan_duration = seconds;
    session_.fusion_params.scan_time = session_.scan_time;
  } else if (session_.calibration_datasets.empty()) {
    session_.scan_time.scan_duration = seconds;
    session_.fusion_params.scan_time = session_.scan_time;
  } else {
    auto &ds = activeDatasetEntry();
    ds.scan_time.scan_duration = seconds;
    ds.fusion_params.scan_time = ds.scan_time;
    session_.scan_time = ds.scan_time;
    session_.fusion_params.scan_time = ds.scan_time;
  }
  emit scanTimeChanged();
}

void AppController::setTimeOrigin(int origin) {
  if (main_work_tab_ == 3) {
    session_.scan_time.time_origin =
        static_cast<glim_cloud_fusion::TimeOrigin>(origin);
    session_.fusion_params.scan_time = session_.scan_time;
  } else if (session_.calibration_datasets.empty()) {
    session_.scan_time.time_origin =
        static_cast<glim_cloud_fusion::TimeOrigin>(origin);
    session_.fusion_params.scan_time = session_.scan_time;
  } else {
    auto &ds = activeDatasetEntry();
    ds.scan_time.time_origin =
        static_cast<glim_cloud_fusion::TimeOrigin>(origin);
    ds.fusion_params.scan_time = ds.scan_time;
    session_.scan_time = ds.scan_time;
    session_.fusion_params.scan_time = ds.scan_time;
  }
  emit scanTimeChanged();
}

void AppController::setTimeUnit(int unit) {
  if (main_work_tab_ == 3) {
    session_.scan_time.time_unit =
        static_cast<glim_cloud_fusion::TimeUnit>(unit);
    session_.fusion_params.scan_time = session_.scan_time;
  } else if (session_.calibration_datasets.empty()) {
    session_.scan_time.time_unit =
        static_cast<glim_cloud_fusion::TimeUnit>(unit);
    session_.fusion_params.scan_time = session_.scan_time;
  } else {
    auto &ds = activeDatasetEntry();
    ds.scan_time.time_unit = static_cast<glim_cloud_fusion::TimeUnit>(unit);
    ds.fusion_params.scan_time = ds.scan_time;
    session_.scan_time = ds.scan_time;
    session_.fusion_params.scan_time = ds.scan_time;
  }
  emit scanTimeChanged();
}

void AppController::setStatus(const QString &message) {
  status_message_ = message;
  emit statusMessageChanged();
}

void AppController::setPointCount(int count) {
  point_count_ = count;
  emit pointCountChanged();
}

void AppController::ensureActiveCalibrationDataset() {
  if (session_.calibration_datasets.empty()) {
    session_.calibration_datasets.emplace_back();
    active_calibration_dataset_index_ = 0;
    emit calibrationDatasetsChanged();
  }
}

clc::DatasetEntry &AppController::activeDatasetEntry() {
  if (main_work_tab_ == 3) {
    session_.ensurePrimaryInspection();
    return session_.inspection_datasets.front();
  }
  if (session_.calibration_datasets.empty()) {
    return empty_calibration_dataset_scratch_;
  }
  if (active_calibration_dataset_index_ < 0 ||
      active_calibration_dataset_index_ >=
          static_cast<int>(session_.calibration_datasets.size())) {
    active_calibration_dataset_index_ = 0;
  }
  return session_.calibration_datasets[static_cast<std::size_t>(
      active_calibration_dataset_index_)];
}

const clc::DatasetEntry &AppController::activeDatasetEntry() const {
  if (main_work_tab_ == 3) {
    if (session_.inspection_datasets.empty()) {
      static const clc::DatasetEntry kEmpty;
      return kEmpty;
    }
    return session_.inspection_datasets.front();
  }
  if (session_.calibration_datasets.empty()) {
    static const clc::DatasetEntry kEmpty;
    return kEmpty;
  }
  const int idx =
      std::clamp(active_calibration_dataset_index_, 0,
                 static_cast<int>(session_.calibration_datasets.size()) - 1);
  return session_.calibration_datasets[static_cast<std::size_t>(idx)];
}

int AppController::pcdFileCount() const {
  return static_cast<int>(activeDatasetEntry().pcd_paths.size());
}

double AppController::fusionVoxelResolution() const {
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    return activeDatasetEntry().fusion_params.voxel_resolution;
  }
  return session_.fusion_params.voxel_resolution;
}

int AppController::fusionNumThreads() const {
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    return activeDatasetEntry().fusion_params.num_threads;
  }
  return session_.fusion_params.num_threads;
}

double AppController::fusionMaxCorrespondence() const {
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    return activeDatasetEntry().fusion_params.max_correspondence_distance;
  }
  return session_.fusion_params.max_correspondence_distance;
}

int AppController::fusionKNeighbors() const {
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    return activeDatasetEntry().fusion_params.k_neighbors;
  }
  return session_.fusion_params.k_neighbors;
}

int AppController::fusionBackend() const {
  const auto backend =
      (main_work_tab_ != 3 && !session_.calibration_datasets.empty())
          ? activeDatasetEntry().fusion_params.backend
          : session_.fusion_params.backend;
  return backend == glim_cloud_fusion::FusionBackend::GlimSlam ? 1 : 0;
}

bool AppController::fusionBackendIsIntegrator() const {
  return fusionBackend() == 0;
}

int AppController::fusionTargetNumPoints() const {
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    return activeDatasetEntry().fusion_params.target_num_points;
  }
  return session_.fusion_params.target_num_points;
}

double AppController::fusionMinDistance() const {
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    return activeDatasetEntry().fusion_params.min_distance;
  }
  return session_.fusion_params.min_distance;
}

double AppController::fusionIntegratorIvoxResolution() const {
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    return activeDatasetEntry().fusion_params.integrator_ivox_resolution;
  }
  return session_.fusion_params.integrator_ivox_resolution;
}

double AppController::fusionIntegratorIvoxMinPointsDist() const {
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    return activeDatasetEntry().fusion_params.integrator_ivox_min_points_dist;
  }
  return session_.fusion_params.integrator_ivox_min_points_dist;
}

double AppController::fusionIvoxResolution() const {
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    return activeDatasetEntry().fusion_params.ivox_resolution;
  }
  return session_.fusion_params.ivox_resolution;
}

int AppController::fusionLmMaxIterations() const {
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    return activeDatasetEntry().fusion_params.lm_max_iterations;
  }
  return session_.fusion_params.lm_max_iterations;
}

int AppController::correspondenceCount() const {
  if (main_work_tab_ == 3 || session_.calibration_datasets.empty()) {
    return 0;
  }
  int total = 0;
  for (const auto &ds : session_.calibration_datasets) {
    total += static_cast<int>(ds.correspondences.size());
  }
  return total;
}

QString AppController::imagePath() const {
  return QString::fromStdString(activeDatasetEntry().image_path);
}

QString AppController::pcdDirectory() const {
  return QString::fromStdString(activeDatasetEntry().cloud_directory);
}

QString AppController::intrinsicsPath() const {
  return QString::fromStdString(session_.intrinsics_path);
}

void AppController::setSessionDirectory(const QString &directory) {
  const QString normalized = normalizedDirectoryPath(directory);
  if (session_directory_ == normalized) {
    return;
  }
  session_directory_ = normalized;
  emit sessionPathsChanged();
}

void AppController::setImagePath(const QString &path) {
  const QString normalized = QFileInfo(path).absoluteFilePath();
  if (imagePath() == normalized) {
    return;
  }
  if (main_work_tab_ == 3) {
    session_.ensurePrimaryInspection();
  } else {
    ensureActiveCalibrationDataset();
  }
  activeDatasetEntry().image_path = normalized.toStdString();
  invalidateProjectionImageCache();
  refreshCameraPreview();
  if (!viewer_positions_.isEmpty()) {
    reprojectViewerCameraRgb();
  }
  emit sessionPathsChanged();
}

void AppController::setPcdDirectory(const QString &path) {
  const QString normalized = normalizedPcdPath(path);
  if (pcdDirectory() == normalized) {
    return;
  }
  if (main_work_tab_ == 3) {
    session_.ensurePrimaryInspection();
  } else {
    ensureActiveCalibrationDataset();
  }
  activeDatasetEntry().cloud_directory = normalized.toStdString();
  refreshPcdCatalog();
  emit sessionPathsChanged();
}

void AppController::setInspectionMode(bool inspection) {
  setMainWorkTab(inspection ? 3 : 0);
}

void AppController::setMainWorkTab(int tab) {
  const int clamped = std::clamp(tab, 0, 3);
  if (main_work_tab_ == clamped) {
    return;
  }
  const int previous_tab = main_work_tab_;
  main_work_tab_ = clamped;
  if (previous_tab == 1 && clamped != 1) {
    pending_correspondence_3d_.reset();
    pending_correspondence_2d_.reset();
  }
  selected_correspondence_row_ = -1;
  emit mainWorkTabChanged();
  emit inspectionModeChanged();
  emit correspondencesChanged();
  emit sessionPathsChanged();
  emit scanTimeChanged();
  emit fusionParamsChanged();
  emit pcdCatalogChanged();
  if (main_work_tab_ != 3) {
    refreshActiveDatasetVisuals();
  } else {
    loadActiveDatasetCloudToViewer();
  }
  refreshViewerGizmoState();
  syncCorrespondenceMarkers();
}

void AppController::setActiveCalibrationDatasetIndex(int index) {
  if (session_.calibration_datasets.empty()) {
    return;
  }
  const int clamped = std::clamp(
      index, 0, static_cast<int>(session_.calibration_datasets.size()) - 1);
  if (active_calibration_dataset_index_ == clamped) {
    return;
  }
  active_calibration_dataset_index_ = clamped;
  pending_correspondence_3d_.reset();
  selected_correspondence_row_ = -1;
  syncSessionFusionToActiveDataset();
  emit calibrationDatasetsChanged();
  emit sessionPathsChanged();
  emit scanTimeChanged();
  emit fusionParamsChanged();
  emit pcdCatalogChanged();
  refreshActiveDatasetVisuals();
  refreshRangeImage();
  emit correspondencesChanged();
}

QStringList AppController::calibrationDatasetIds() const {
  QStringList ids;
  for (const auto &ds : session_.calibration_datasets) {
    ids.append(QString::fromStdString(
        ds.id.empty() ? QStringLiteral("(unnamed)").toStdString() : ds.id));
  }
  return ids;
}

bool AppController::activeDatasetReady() const {
  return isCalibrationDatasetReady(active_calibration_dataset_index_);
}

bool AppController::canPickCorrespondences() const {
  if (session_.calibration_datasets.empty() ||
      active_calibration_dataset_index_ < 0 ||
      active_calibration_dataset_index_ >=
          static_cast<int>(session_.calibration_datasets.size())) {
    return false;
  }
  return clc::dataset_pickable(
      session_.calibration_datasets[static_cast<std::size_t>(
          active_calibration_dataset_index_)]);
}

bool AppController::isCalibrationDatasetReady(int index) const {
  if (index < 0 ||
      index >= static_cast<int>(session_.calibration_datasets.size())) {
    return false;
  }
  return clc::dataset_ready(
      session_.calibration_datasets[static_cast<std::size_t>(index)]);
}

QString AppController::calibrationDatasetTooltip(int index) const {
  if (isCalibrationDatasetReady(index)) {
    return QStringLiteral("Ready for initial alignment and calibration");
  }
  return QStringLiteral(
      "Load image and LiDAR; fuse multi-scan datasets before NID");
}

bool AppController::manualInitialGuessSupportsMultiDataset() const {
  return clc::InitialGuessTraits<
      clc::ManualGuessProvider>::supports_multi_dataset;
}

void AppController::setInitialAlignmentMethod(int method) {
  if (initial_alignment_method_ == method) {
    return;
  }
  initial_alignment_method_ = method;
  emit initialAlignmentMethodChanged();
}

void AppController::setMaskPreviewDatasetIndex(int index) {
  if (session_.calibration_datasets.empty()) {
    return;
  }
  mask_preview_dataset_index_ = std::clamp(
      index, 0, static_cast<int>(session_.calibration_datasets.size()) - 1);

  const int prev = active_calibration_dataset_index_;
  active_calibration_dataset_index_ = mask_preview_dataset_index_;

  refreshRangeImage();
  active_calibration_dataset_index_ = prev;
}

bool AppController::activeDatasetIsSingleScan() const {
  return clc::dataset_is_single_scan(activeDatasetEntry());
}

QString AppController::addDatasetDefaultImagePath() const {
#ifdef CLC_DEV_ADD_DATASET_DEFAULTS
  return QStringLiteral(
      "/Users/rasmus/Downloads/pbb-muc-255-1-fasssauna-high-freq-approach/"
      "sequence1/2026-10-01T18_42_13_624Z.png");
#else
  return {};
#endif
}

QString AppController::addDatasetDefaultLidarPath() const {
#ifdef CLC_DEV_ADD_DATASET_DEFAULTS
  return QStringLiteral(
      "/Users/rasmus/Downloads/pbb-muc-255-1-fasssauna-high-freq-approach/"
      "sequence1.pcd");
#else
  return {};
#endif
}

QString AppController::addDatasetDefaultIntrinsicsPath() const {
#ifdef CLC_DEV_ADD_DATASET_DEFAULTS
  return QStringLiteral("/Users/rasmus/Documents/Projects/autocrane-core/"
                        "config/pbb/muc/shared/calibration/"
                        "cam_bridge_narrow_rotated_upright.yml");
#else
  return {};
#endif
}

QString AppController::pickAddDatasetImage(const QString &initial_dir) {
  const QString start =
      initial_dir.isEmpty() ? addDatasetDefaultImagePath() : initial_dir;
  return QFileDialog::getOpenFileName(
      nullptr, QStringLiteral("Camera image for new dataset"), start,
      QStringLiteral("Images (*.png *.jpg *.jpeg *.bmp *.tif *.tiff)"));
}

QString AppController::pickAddDatasetLidarFile(const QString &initial_dir) {
  QString start = initial_dir;
  if (start.isEmpty()) {
    start = addDatasetDefaultLidarPath();
  }
  return pickPcdFileDialog(QStringLiteral("LiDAR scan file for new dataset"),
                           start);
}

QString AppController::pickAddDatasetLidarFolder(const QString &initial_dir) {
  QString start = initial_dir;
  if (start.isEmpty()) {
    start = addDatasetDefaultLidarPath();
  }
  return pickPcdFolderDialog(
      QStringLiteral("LiDAR scan folder for new dataset"), start);
}

QString AppController::pickAddDatasetIntrinsics(const QString &initial_dir) {
  const QString start =
      initial_dir.isEmpty() ? addDatasetDefaultIntrinsicsPath() : initial_dir;
  return QFileDialog::getOpenFileName(
      nullptr, QStringLiteral("Camera intrinsics for session"), start,
      QStringLiteral("YAML (*.yaml *.yml);;OpenCV (*.xml)"));
}

bool AppController::addCalibrationDatasetFromPaths(
    const QString &image_path, const QString &lidar_path,
    const QString &intrinsics_path) {
  if (image_path.isEmpty()) {
    setStatus(QStringLiteral("Camera image path is required"));
    return false;
  }
  if (lidar_path.isEmpty()) {
    setStatus(
        QStringLiteral("LiDAR path (directory or .pcd file) is required"));
    return false;
  }

  const auto scan = clc::scan_pcd_source(lidar_path.toStdString(),
                                         session_.reference_scan_width,
                                         session_.reference_scan_height);
  if (!scan.error.empty()) {
    setStatus(QString::fromStdString(scan.error));
    return false;
  }
  const std::string id = clc::dataset_id_from_first_scan(scan.paths);
  if (!clc::calibration_dataset_id_unique(session_, id)) {
    setStatus(
        QStringLiteral(
            "Dataset id '%1' already exists (duplicate first-scan timestamp)")
            .arg(QString::fromStdString(id)));
    return false;
  }

  if (!intrinsics_path.isEmpty()) {
    if (!loadIntrinsicsFile(intrinsics_path)) {
      return false;
    }
  }

  clc::DatasetEntry entry;
  entry.id = id;
  entry.image_path = QFileInfo(image_path).absoluteFilePath().toStdString();
  entry.cloud_directory = normalizedPcdPath(lidar_path).toStdString();
  entry.pcd_paths = scan.paths;
  entry.geometry_error = false;
  clc::sync_dataset_fusion_status(entry);
  if (!session_.calibration_datasets.empty()) {
    entry.fusion_params = session_.calibration_datasets.front().fusion_params;
    entry.scan_time = session_.calibration_datasets.front().scan_time;
  } else {
    entry.fusion_params = session_.fusion_params;
    entry.scan_time = session_.scan_time;
    entry.fusion_params.scan_time = entry.scan_time;
  }
  if (session_.reference_scan_width <= 0) {
    session_.reference_scan_width = scan.reference_width;
    session_.reference_scan_height = scan.reference_height;
  } else if (scan.reference_width != session_.reference_scan_width ||
             scan.reference_height != session_.reference_scan_height) {
    setStatus(
        QStringLiteral("Scan geometry does not match session reference (%1×%2)")
            .arg(session_.reference_scan_width)
            .arg(session_.reference_scan_height));
    return false;
  }

  session_.calibration_datasets.erase(
      std::remove_if(session_.calibration_datasets.begin(),
                     session_.calibration_datasets.end(),
                     clc::calibration_dataset_is_placeholder),
      session_.calibration_datasets.end());
  session_.calibration_datasets.push_back(entry);
  active_calibration_dataset_index_ =
      static_cast<int>(session_.calibration_datasets.size()) - 1;
  fusion_blocked_ = false;
  fused_cloud_ = {};
  fused_cloud_dataset_index_.reset();
  emit calibrationDatasetsChanged();
  emit pcdCatalogChanged();
  emit sessionPathsChanged();
  refreshRangeImage();
  refreshActiveDatasetVisuals();
  setStatus(QStringLiteral("Added calibration dataset %1")
                .arg(QString::fromStdString(id)));
  return true;
}

void AppController::removeActiveCalibrationDataset() {
  if (session_.calibration_datasets.empty()) {
    return;
  }
  session_.calibration_datasets.erase(session_.calibration_datasets.begin() +
                                      active_calibration_dataset_index_);
  fused_cloud_ = {};
  fused_cloud_dataset_index_.reset();
  pending_correspondence_3d_.reset();
  pending_correspondence_2d_.reset();
  selected_correspondence_row_ = -1;
  if (session_.calibration_datasets.empty()) {
    active_calibration_dataset_index_ = 0;
    empty_calibration_dataset_scratch_ = {};
    fusion_blocked_ = false;
    emit calibrationDatasetsChanged();
    emit pcdCatalogChanged();
    emit sessionPathsChanged();
    emit fusionParamsChanged();
    emit scanTimeChanged();
    clearCalibrationDatasetVisuals();
    emit correspondencesChanged();
    setStatus(QStringLiteral("Calibration dataset removed"));
    return;
  }
  active_calibration_dataset_index_ =
      std::min(active_calibration_dataset_index_,
               static_cast<int>(session_.calibration_datasets.size()) - 1);
  emit calibrationDatasetsChanged();
  emit pcdCatalogChanged();
  emit sessionPathsChanged();
  refreshActiveDatasetVisuals();
  emit correspondencesChanged();
}

void AppController::syncActiveDatasetFusionToSession() {
  if (main_work_tab_ == 3 || session_.calibration_datasets.empty()) {
    return;
  }
  const auto &ds = activeDatasetEntry();
  session_.fusion_params = ds.fusion_params;
  session_.scan_time = ds.scan_time;
  session_.fusion_params.scan_time = ds.scan_time;
}

void AppController::syncSessionFusionToActiveDataset() {
  if (main_work_tab_ == 3 || session_.calibration_datasets.empty()) {
    return;
  }
  auto &ds = activeDatasetEntry();
  ds.fusion_params = session_.fusion_params;
  ds.scan_time = session_.scan_time;
  ds.fusion_params.scan_time = ds.scan_time;
}

QVariantList AppController::correspondenceTable() const {
  QVariantList rows;
  if (main_work_tab_ == 3) {
    return rows;
  }
  int global = 0;
  for (int di = 0; di < static_cast<int>(session_.calibration_datasets.size());
       ++di) {
    const auto &ds =
        session_.calibration_datasets[static_cast<std::size_t>(di)];
    const bool editable = (di == active_calibration_dataset_index_);
    for (int ri = 0; ri < static_cast<int>(ds.correspondences.size()); ++ri) {
      const auto &c = ds.correspondences[static_cast<std::size_t>(ri)];
      QVariantMap row;
      row.insert(QStringLiteral("globalIndex"), global);
      row.insert(QStringLiteral("datasetIndex"), di);
      row.insert(QStringLiteral("rowIndex"), ri);
      row.insert(QStringLiteral("datasetId"), QString::fromStdString(ds.id));
      row.insert(QStringLiteral("u"), c.u);
      row.insert(QStringLiteral("v"), c.v);
      row.insert(QStringLiteral("x"), c.x);
      row.insert(QStringLiteral("y"), c.y);
      row.insert(QStringLiteral("z"), c.z);
      row.insert(QStringLiteral("editable"), editable);
      row.insert(QStringLiteral("selected"),
                 global == selected_correspondence_row_);
      rows.append(row);
      ++global;
    }
  }
  if (main_work_tab_ == 1 && active_calibration_dataset_index_ >= 0 &&
      active_calibration_dataset_index_ <
          static_cast<int>(session_.calibration_datasets.size())) {
    const auto &ds = session_.calibration_datasets[static_cast<std::size_t>(
        active_calibration_dataset_index_)];
    if (pending_correspondence_3d_.has_value()) {
      QVariantMap row;
      row.insert(QStringLiteral("globalIndex"), -2);
      row.insert(QStringLiteral("datasetIndex"),
                 active_calibration_dataset_index_);
      row.insert(QStringLiteral("rowIndex"), -1);
      row.insert(QStringLiteral("datasetId"), QString::fromStdString(ds.id));
      row.insert(QStringLiteral("x"), pending_correspondence_3d_->x());
      row.insert(QStringLiteral("y"), pending_correspondence_3d_->y());
      row.insert(QStringLiteral("z"), pending_correspondence_3d_->z());
      row.insert(QStringLiteral("editable"), true);
      row.insert(QStringLiteral("selected"), false);
      row.insert(QStringLiteral("pendingPick"), true);
      row.insert(QStringLiteral("pendingKind"), QStringLiteral("3d"));
      rows.append(row);
    }
    if (pending_correspondence_2d_.has_value()) {
      QVariantMap row;
      row.insert(QStringLiteral("globalIndex"), -1);
      row.insert(QStringLiteral("datasetIndex"),
                 active_calibration_dataset_index_);
      row.insert(QStringLiteral("rowIndex"), -1);
      row.insert(QStringLiteral("datasetId"), QString::fromStdString(ds.id));
      row.insert(QStringLiteral("u"), pending_correspondence_2d_->first);
      row.insert(QStringLiteral("v"), pending_correspondence_2d_->second);
      row.insert(QStringLiteral("editable"), true);
      row.insert(QStringLiteral("selected"), false);
      row.insert(QStringLiteral("pendingPick"), true);
      row.insert(QStringLiteral("pendingKind"), QStringLiteral("image"));
      rows.append(row);
    }
  }
  return rows;
}

bool AppController::canComputeInitialAlignment() const {
  if (session_.camera.intrinsics.empty()) {
    return false;
  }
  return correspondenceCount() >= 3;
}

void AppController::setSelectedCorrespondenceRow(int global_row) {
  selected_correspondence_row_ = global_row;
  emit correspondencesChanged();
}

void AppController::deleteCorrespondenceRow(int global_row) {
  if (main_work_tab_ != 1) {
    return;
  }
  if (global_row == -1) {
    pending_correspondence_2d_.reset();
    selected_correspondence_row_ = -1;
    emit correspondencesChanged();
    setStatus(QStringLiteral("Removed pending image pick"));
    return;
  }
  if (global_row == -2) {
    pending_correspondence_3d_.reset();
    selected_correspondence_row_ = -1;
    emit correspondencesChanged();
    setStatus(QStringLiteral("Removed pending 3D pick"));
    return;
  }
  if (global_row < 0) {
    return;
  }
  int cursor = 0;
  for (int di = 0; di < static_cast<int>(session_.calibration_datasets.size());
       ++di) {
    auto &ds = session_.calibration_datasets[static_cast<std::size_t>(di)];
    const int count = static_cast<int>(ds.correspondences.size());
    if (global_row >= cursor && global_row < cursor + count) {
      const int local = global_row - cursor;
      if (di != active_calibration_dataset_index_) {
        setStatus(QStringLiteral(
            "Switch to that dataset to delete its correspondences"));
        return;
      }
      ds.correspondences.erase(ds.correspondences.begin() + local);
      selected_correspondence_row_ = -1;
      pending_correspondence_3d_.reset();
      pending_correspondence_2d_.reset();
      emit correspondencesChanged();
      setStatus(QStringLiteral("Correspondence removed (%1 remaining)")
                    .arg(correspondenceCount()));
      return;
    }
    cursor += count;
  }
  setStatus(QStringLiteral("Could not delete correspondence (row not found)"));
}

void AppController::refreshCameraPreview() {
  const QString path = imagePath();
  if (path.isEmpty()) {
    camera_preview_ = {};
    emit cameraPreviewChanged();
    return;
  }
  QImage img(path);
  if (img.isNull()) {
    camera_preview_ = {};
  } else {
    camera_preview_ = img.convertToFormat(QImage::Format_RGB888);
  }
  emit cameraPreviewChanged();
}

void AppController::setIntrinsicsPath(const QString &path) {
  const QString normalized = QFileInfo(path).absoluteFilePath();
  if (QString::fromStdString(session_.intrinsics_path) == normalized) {
    return;
  }
  session_.intrinsics_path = normalized.toStdString();
  emit sessionPathsChanged();
}

bool AppController::repopulateActiveDatasetPcdCatalogIfEmpty() {
  auto &dataset = activeDatasetEntry();
  if (!dataset.pcd_paths.empty()) {
    return true;
  }
  if (dataset.cloud_directory.empty()) {
    return false;
  }

  const auto scan = clc::scan_pcd_source(dataset.cloud_directory,
                                         session_.reference_scan_width,
                                         session_.reference_scan_height);
  if (!scan.error.empty()) {
    dataset.pcd_paths.clear();
    dataset.geometry_error = true;
    fusion_blocked_ = true;
    setStatus(QString::fromStdString(scan.error));
    emit pcdCatalogChanged();
    emit calibrationDatasetsChanged();
    return false;
  }

  dataset.geometry_error = false;
  dataset.pcd_paths = scan.paths;
  if (dataset.id.empty() && !dataset.pcd_paths.empty()) {
    dataset.id = clc::dataset_id_from_first_scan(dataset.pcd_paths);
  }
  clc::sync_dataset_fusion_status(dataset);
  session_.reference_scan_width = scan.reference_width;
  session_.reference_scan_height = scan.reference_height;
  emit pcdCatalogChanged();
  emit calibrationDatasetsChanged();
  return !dataset.pcd_paths.empty();
}

void AppController::refreshPcdCatalog() {
  fusion_blocked_ = false;
  if (session_.calibration_datasets.empty()) {
    emit pcdCatalogChanged();
    return;
  }
  auto &dataset = activeDatasetEntry();
  if (dataset.cloud_directory.empty()) {
    dataset.pcd_paths.clear();
    emit pcdCatalogChanged();
    return;
  }

  dataset.pcd_paths.clear();
  if (!repopulateActiveDatasetPcdCatalogIfEmpty()) {
    return;
  }

  fused_cloud_ = {};
  fused_cloud_dataset_index_.reset();
  setStatus(QStringLiteral("Found %1 PCD files (%2×%3)")
                .arg(static_cast<int>(dataset.pcd_paths.size()))
                .arg(session_.reference_scan_width)
                .arg(session_.reference_scan_height));
  refreshRangeImage();
  refreshPreviewInViewer();
}

void AppController::refreshPreviewInViewer() {
  const bool fused_cache_for_active =
      fused_cloud_dataset_index_.has_value() &&
      *fused_cloud_dataset_index_ == active_calibration_dataset_index_ &&
      !fused_cloud_.points.empty();
  if (fused_cache_for_active) {
    uploadFusedCloudToViewer();
  } else {
    uploadPreviewScanToViewer();
  }
}

int AppController::maskWidth() const { return maskRasterWidth(); }

int AppController::maskHeight() const { return maskRasterHeight(); }

QByteArray AppController::maskPreview() const {
  const int w = maskRasterWidth();
  const int h = maskRasterHeight();
  if (w <= 0 || h <= 0) {
    return {};
  }
  const std::size_t n = static_cast<std::size_t>(w * h);
  std::vector<std::uint8_t> preview;
  if (global_mask_pixels_.size() == n) {
    preview = global_mask_pixels_;
  } else {
    preview.assign(n, 255);
  }
  if (mask_polygon_pending_ && pending_mask_polygon_.vertices.size() >= 3) {
    const std::vector<std::uint8_t> pending_interior =
        clc::rasterize_mask(w, h, {pending_mask_polygon_}, false);
    if (pending_interior.size() == n) {
      for (std::size_t i = 0; i < n; ++i) {
        if (pending_interior[i] == 0) {
          preview[i] = 0;
        }
      }
    }
  }
  return QByteArray(reinterpret_cast<const char *>(preview.data()),
                    static_cast<int>(preview.size()));
}

void AppController::setRangeColormapMin(double v) {
  if (qFuzzyCompare(range_colormap_min_, v)) {
    return;
  }
  range_colormap_min_ = v;
  refreshRangeImage();
}

void AppController::setRangeColormapMax(double v) {
  if (qFuzzyCompare(range_colormap_max_, v)) {
    return;
  }
  range_colormap_max_ = v;
  refreshRangeImage();
}

void AppController::setRangeChannel(int channel) {
  if (range_channel_ == channel) {
    return;
  }
  range_channel_ = channel;
  refreshRangeImage();
}

void AppController::setRangeYStretch(double stretch) {
  stretch = std::clamp(stretch, 1.0, 8.0);
  if (qFuzzyCompare(range_y_stretch_, stretch)) {
    return;
  }
  range_y_stretch_ = stretch;
  refreshRangeImage();
  emit rangeYStretchChanged();
}

void AppController::setViewerColormapMin(double v) {
  previewViewerColormapRange(v, viewer_colormap_max_);
}

void AppController::setViewerColormapMax(double v) {
  previewViewerColormapRange(viewer_colormap_min_, v);
}

void AppController::applyViewerScalarRangeToItem() {
  if (auto *item = qobject_cast<psiviz::PointCloudItem *>(viewer_)) {
    item->setScalarMin(viewer_colormap_min_);
    item->setScalarMax(viewer_colormap_max_);
  }
}

void AppController::previewViewerColormapRange(double min_v, double max_v) {
  if (max_v < min_v) {
    std::swap(min_v, max_v);
  }
  viewer_colormap_min_ = min_v;
  viewer_colormap_max_ = max_v;
  emit viewerColormapChanged();
  applyViewerScalarRangeToItem();
}

void AppController::commitViewerColormapRange(double min_v, double max_v) {
  viewer_colormap_debounce_.stop();
  previewViewerColormapRange(min_v, max_v);
}

void AppController::onViewerColormapDebounce() {
  applyViewerScalarRangeToItem();
}

void AppController::previewRangeColormapRange(double min_v, double max_v) {
  if (max_v < min_v) {
    std::swap(min_v, max_v);
  }
  range_colormap_min_ = min_v;
  range_colormap_max_ = max_v;
  emit rangeImageChanged();
  refreshRangeImageDisplay();
  range_colormap_debounce_.start();
}

void AppController::commitRangeColormapRange(double min_v, double max_v) {
  range_colormap_debounce_.stop();
  previewRangeColormapRange(min_v, max_v);
}

void AppController::onRangeColormapDebounce() { refreshRangeImageDisplay(); }

void AppController::setViewerScalarChannel(int channel) {
  if (viewer_scalar_channel_ == channel) {
    return;
  }
  viewer_scalar_channel_ = channel;
  if (!fused_cloud_.points.empty()) {
    refreshViewerColormapBounds();
  }
  emit viewerColormapChanged();
  if (!fused_cloud_.points.empty()) {
    uploadFusedCloudToViewer();
  } else {
    uploadPreviewScanToViewer();
  }
}

void AppController::setFusionVoxelResolution(double v) {
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    auto &ds = activeDatasetEntry();
    if (qFuzzyCompare(ds.fusion_params.voxel_resolution, v)) {
      return;
    }
    ds.fusion_params.voxel_resolution = v;
    session_.fusion_params = ds.fusion_params;
  } else if (qFuzzyCompare(session_.fusion_params.voxel_resolution, v)) {
    return;
  } else {
    session_.fusion_params.voxel_resolution = v;
  }
  emit fusionParamsChanged();
}

void AppController::setFusionNumThreads(int n) {
  const int clamped = std::max(1, n);
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    auto &ds = activeDatasetEntry();
    if (ds.fusion_params.num_threads == clamped) {
      return;
    }
    ds.fusion_params.num_threads = clamped;
    session_.fusion_params = ds.fusion_params;
  } else if (session_.fusion_params.num_threads == clamped) {
    return;
  } else {
    session_.fusion_params.num_threads = clamped;
  }
  emit fusionParamsChanged();
}

void AppController::setFusionMaxCorrespondence(double v) {
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    auto &ds = activeDatasetEntry();
    if (qFuzzyCompare(ds.fusion_params.max_correspondence_distance, v)) {
      return;
    }
    ds.fusion_params.max_correspondence_distance = v;
    session_.fusion_params = ds.fusion_params;
  } else if (qFuzzyCompare(session_.fusion_params.max_correspondence_distance,
                           v)) {
    return;
  } else {
    session_.fusion_params.max_correspondence_distance = v;
  }
  emit fusionParamsChanged();
}

void AppController::setFusionKNeighbors(int n) {
  const int clamped = std::max(2, n);
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    auto &ds = activeDatasetEntry();
    if (ds.fusion_params.k_neighbors == clamped) {
      return;
    }
    ds.fusion_params.k_neighbors = clamped;
    session_.fusion_params = ds.fusion_params;
  } else if (session_.fusion_params.k_neighbors == clamped) {
    return;
  } else {
    session_.fusion_params.k_neighbors = clamped;
  }
  emit fusionParamsChanged();
}

void AppController::setFusionBackend(int backend) {
  const auto value = backend == 1
                         ? glim_cloud_fusion::FusionBackend::GlimSlam
                         : glim_cloud_fusion::FusionBackend::CtIntegrator;
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    auto &ds = activeDatasetEntry();
    if (ds.fusion_params.backend == value) {
      return;
    }
    ds.fusion_params.backend = value;
    session_.fusion_params = ds.fusion_params;
  } else if (session_.fusion_params.backend == value) {
    return;
  } else {
    session_.fusion_params.backend = value;
  }
  emit fusionParamsChanged();
}

void AppController::setFusionTargetNumPoints(int n) {
  const int clamped = std::max(100, n);
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    auto &ds = activeDatasetEntry();
    if (ds.fusion_params.target_num_points == clamped) {
      return;
    }
    ds.fusion_params.target_num_points = clamped;
    session_.fusion_params = ds.fusion_params;
  } else if (session_.fusion_params.target_num_points == clamped) {
    return;
  } else {
    session_.fusion_params.target_num_points = clamped;
  }
  emit fusionParamsChanged();
}

void AppController::setFusionMinDistance(double v) {
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    auto &ds = activeDatasetEntry();
    if (qFuzzyCompare(ds.fusion_params.min_distance, v)) {
      return;
    }
    ds.fusion_params.min_distance = v;
    session_.fusion_params = ds.fusion_params;
  } else if (qFuzzyCompare(session_.fusion_params.min_distance, v)) {
    return;
  } else {
    session_.fusion_params.min_distance = v;
  }
  emit fusionParamsChanged();
}

void AppController::setFusionIntegratorIvoxResolution(double v) {
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    auto &ds = activeDatasetEntry();
    if (qFuzzyCompare(ds.fusion_params.integrator_ivox_resolution, v)) {
      return;
    }
    ds.fusion_params.integrator_ivox_resolution = v;
    session_.fusion_params = ds.fusion_params;
  } else if (qFuzzyCompare(session_.fusion_params.integrator_ivox_resolution,
                           v)) {
    return;
  } else {
    session_.fusion_params.integrator_ivox_resolution = v;
  }
  emit fusionParamsChanged();
}

void AppController::setFusionIntegratorIvoxMinPointsDist(double v) {
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    auto &ds = activeDatasetEntry();
    if (qFuzzyCompare(ds.fusion_params.integrator_ivox_min_points_dist, v)) {
      return;
    }
    ds.fusion_params.integrator_ivox_min_points_dist = v;
    session_.fusion_params = ds.fusion_params;
  } else if (qFuzzyCompare(
                 session_.fusion_params.integrator_ivox_min_points_dist, v)) {
    return;
  } else {
    session_.fusion_params.integrator_ivox_min_points_dist = v;
  }
  emit fusionParamsChanged();
}

void AppController::setFusionIvoxResolution(double v) {
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    auto &ds = activeDatasetEntry();
    if (qFuzzyCompare(ds.fusion_params.ivox_resolution, v)) {
      return;
    }
    ds.fusion_params.ivox_resolution = v;
    session_.fusion_params = ds.fusion_params;
  } else if (qFuzzyCompare(session_.fusion_params.ivox_resolution, v)) {
    return;
  } else {
    session_.fusion_params.ivox_resolution = v;
  }
  emit fusionParamsChanged();
}

void AppController::setFusionLmMaxIterations(int n) {
  const int clamped = std::max(1, n);
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    auto &ds = activeDatasetEntry();
    if (ds.fusion_params.lm_max_iterations == clamped) {
      return;
    }
    ds.fusion_params.lm_max_iterations = clamped;
    session_.fusion_params = ds.fusion_params;
  } else if (session_.fusion_params.lm_max_iterations == clamped) {
    return;
  } else {
    session_.fusion_params.lm_max_iterations = clamped;
  }
  emit fusionParamsChanged();
}

int AppController::calibrationMaxOuterIterations() const {
  return session_.calibration_params.max_outer_iterations;
}

int AppController::calibrationMaxInnerIterations() const {
  return session_.calibration_params.max_inner_iterations;
}

int AppController::calibrationNumThreads() const {
  return session_.calibration_params.num_threads;
}

double AppController::calibrationDeltaTransThresh() const {
  return session_.calibration_params.delta_trans_thresh;
}

double AppController::calibrationDeltaRotDeg() const {
  return session_.calibration_params.delta_rot_thresh * 180.0 / M_PI;
}

bool AppController::calibrationDisableZBufferCulling() const {
  return session_.calibration_params.disable_z_buffer_culling;
}

int AppController::calibrationNidBins() const {
  return session_.calibration_params.nid_bins;
}

int AppController::calibrationRegistrationType() const {
  return session_.calibration_params.registration_type ==
                 vlcal::RegistrationType::NID_NELDER_MEAD
             ? 1
             : 0;
}

double AppController::calibrationNelderMeadInitStep() const {
  return session_.calibration_params.nelder_mead_init_step;
}

double AppController::calibrationNelderMeadConvergence() const {
  return session_.calibration_params.nelder_mead_convergence_criteria;
}

void AppController::setCalibrationMaxOuterIterations(int v) {
  const int clamped = std::max(1, v);
  if (session_.calibration_params.max_outer_iterations == clamped) {
    return;
  }
  session_.calibration_params.max_outer_iterations = clamped;
  emit calibrationParamsChanged();
}

void AppController::setCalibrationMaxInnerIterations(int v) {
  const int clamped = std::max(1, v);
  if (session_.calibration_params.max_inner_iterations == clamped) {
    return;
  }
  session_.calibration_params.max_inner_iterations = clamped;
  emit calibrationParamsChanged();
}

void AppController::setCalibrationNumThreads(int v) {
  const int clamped = std::max(1, v);
  if (session_.calibration_params.num_threads == clamped) {
    return;
  }
  session_.calibration_params.num_threads = clamped;
  emit calibrationParamsChanged();
}

void AppController::setCalibrationDeltaTransThresh(double v) {
  if (qFuzzyCompare(session_.calibration_params.delta_trans_thresh, v)) {
    return;
  }
  session_.calibration_params.delta_trans_thresh = v;
  emit calibrationParamsChanged();
}

void AppController::setCalibrationDeltaRotDeg(double v) {
  const double rad = v * M_PI / 180.0;
  if (qFuzzyCompare(session_.calibration_params.delta_rot_thresh, rad)) {
    return;
  }
  session_.calibration_params.delta_rot_thresh = rad;
  emit calibrationParamsChanged();
}

void AppController::setCalibrationDisableZBufferCulling(bool v) {
  if (session_.calibration_params.disable_z_buffer_culling == v) {
    return;
  }
  session_.calibration_params.disable_z_buffer_culling = v;
  emit calibrationParamsChanged();
}

void AppController::setCalibrationNidBins(int v) {
  const int clamped = std::max(2, v);
  if (session_.calibration_params.nid_bins == clamped) {
    return;
  }
  session_.calibration_params.nid_bins = clamped;
  emit calibrationParamsChanged();
}

void AppController::setCalibrationRegistrationType(int v) {
  const auto reg = (v == 1) ? vlcal::RegistrationType::NID_NELDER_MEAD
                            : vlcal::RegistrationType::NID_BFGS;
  if (session_.calibration_params.registration_type == reg) {
    return;
  }
  session_.calibration_params.registration_type = reg;
  emit calibrationParamsChanged();
}

void AppController::setCalibrationNelderMeadInitStep(double v) {
  if (qFuzzyCompare(session_.calibration_params.nelder_mead_init_step, v)) {
    return;
  }
  session_.calibration_params.nelder_mead_init_step = v;
  emit calibrationParamsChanged();
}

void AppController::setCalibrationNelderMeadConvergence(double v) {
  if (qFuzzyCompare(
          session_.calibration_params.nelder_mead_convergence_criteria, v)) {
    return;
  }
  session_.calibration_params.nelder_mead_convergence_criteria = v;
  emit calibrationParamsChanged();
}

void AppController::refreshViewerColors() { applyViewerScalarRangeToItem(); }

void AppController::invalidateProjectionImageCache() {
  projection_image_.release();
  projection_image_path_.clear();
}

bool AppController::ensureProjectionImageForActiveDataset() {
  if (session_.calibration_datasets.empty()) {
    invalidateProjectionImageCache();
    return false;
  }
  const std::string image_path = activeDatasetEntry().image_path;
  if (image_path.empty()) {
    invalidateProjectionImageCache();
    return false;
  }
  if (projection_image_path_ == image_path && !projection_image_.empty()) {
    return true;
  }
  projection_image_.release();
  projection_image_ = cv::imread(image_path, cv::IMREAD_COLOR);
  projection_image_path_ = image_path;
  return !projection_image_.empty();
}

void AppController::clearCalibrationDatasetVisuals() {
  invalidateProjectionImageCache();
  fused_cloud_ = {};
  fused_cloud_dataset_index_.reset();
  range_image_ = {};
  range_image_display_ = {};
  emit rangeImageChanged();
  viewer_positions_.clear();
  viewer_scalars_.clear();
  viewer_rgb_.clear();
  setPointCount(0);
  if (auto *item = qobject_cast<psiviz::PointCloudItem *>(viewer_)) {
    item->setPointCloudData({}, {}, {});
  }
  refreshCameraPreview();
  refreshViewerGizmoState();
}

void AppController::refreshActiveDatasetVisuals() {
  if (main_work_tab_ == 3) {
    return;
  }
  if (session_.calibration_datasets.empty()) {
    clearCalibrationDatasetVisuals();
    return;
  }
  invalidateProjectionImageCache();
  if (!fused_cloud_dataset_index_.has_value() ||
      *fused_cloud_dataset_index_ != active_calibration_dataset_index_) {
    fused_cloud_ = {};
    fused_cloud_dataset_index_.reset();
  }
  refreshCameraPreview();
  loadActiveDatasetCloudToViewer();
}

void AppController::syncColorModelFromSession() {
  const std::string image_path = activeDatasetEntry().image_path;
  if (session_.camera.intrinsics.empty() || image_path.empty() ||
      viewer_positions_.isEmpty()) {
    return;
  }
  if (!ensureProjectionImageForActiveDataset()) {
    return;
  }
  const auto camera =
      camera::create_camera(session_.camera.model, session_.camera.intrinsics,
                            session_.camera.distortion_coeffs);
  color_model_.setCamera(camera);
  color_model_.setImage(projection_image_);

  const int n = viewer_positions_.size() / 3;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      points;
  std::vector<double> intensities;
  points.reserve(n);
  intensities.reserve(n);
  for (int i = 0; i < n; ++i) {
    points.emplace_back(viewer_positions_[i * 3], viewer_positions_[i * 3 + 1],
                        viewer_positions_[i * 3 + 2]);
    intensities.push_back(i < viewer_scalars_.size()
                              ? static_cast<double>(viewer_scalars_[i])
                              : 0.0);
  }
  color_model_.setPoints(points, intensities);
}

void AppController::reprojectViewerCameraRgb() {
  auto *item = qobject_cast<psiviz::PointCloudItem *>(viewer_);
  if (!item || viewer_positions_.isEmpty()) {
    return;
  }
  if (!color_model_.ready()) {
    syncColorModelFromSession();
  }
  if (!color_model_.ready()) {
    viewer_rgb_.fill(0.f, viewer_positions_.size());
  } else {
    std::vector<float> rgb;
    color_model_.projectCameraRgb(session_.T_camera_lidar, rgb);
    viewer_rgb_ = QVector<float>(static_cast<int>(rgb.size()));
    if (!rgb.empty()) {
      std::memcpy(viewer_rgb_.data(), rgb.data(), rgb.size() * sizeof(float));
    }
  }
  item->setScalarMin(viewer_colormap_min_);
  item->setScalarMax(viewer_colormap_max_);
  const int point_count = viewer_positions_.size() / 3;
  const bool same_geometry =
      item->hasPointCloudContent() &&
      item->pointCloudPointCount() == static_cast<std::uint64_t>(point_count);
  if (same_geometry) {
    item->updatePointCloudScalarsAndColors(viewer_scalars_, viewer_rgb_);
  } else {
    item->setPointCloudData(viewer_positions_, viewer_scalars_, viewer_rgb_);
  }
  syncCorrespondenceMarkers();
}

bool AppController::buildAlignmentInput(clc::AlignmentInput &input,
                                        std::string &error) {
  static_assert(
      clc::OptimizerTraits<clc::NidOptimizerProvider>::supports_multi_dataset);
  if (session_.camera.intrinsics.empty()) {
    error = "camera intrinsics required";
    return false;
  }
  input.camera =
      camera::create_camera(session_.camera.model, session_.camera.intrinsics,
                            session_.camera.distortion_coeffs);
  input.init_T_camera_lidar = session_.T_camera_lidar_initial;
  input.params = session_.calibration_params;

  auto load_fused_for_dataset =
      [&](const clc::DatasetEntry &ds,
          glim_cloud_fusion::DensePointCloud &out) -> bool {
    if (!ds.fused_cloud_path.empty()) {
      std::string err;
      if (clc::load_dense_pcd(ds.fused_cloud_path, out, err)) {
        return !out.points.empty();
      }
    }
    if (&ds == &activeDatasetEntry() && !fused_cloud_.points.empty() &&
        fused_cloud_dataset_index_.has_value() &&
        *fused_cloud_dataset_index_ == active_calibration_dataset_index_) {
      out = fused_cloud_;
      return true;
    }
    if (ds.pcd_paths.size() == 1) {
      clc::PointXYZITCloud cloud;
      std::string err;
      if (clc::load_pcd_xyzit(ds.pcd_paths.front(), cloud, err, false)) {
        out.points.clear();
        out.intensities.clear();
        for (const auto &p : cloud.scan.points) {
          out.points.emplace_back(p.x(), p.y(), p.z());
        }
        out.intensities = cloud.scan.intensities;
        return !out.points.empty();
      }
    }
    return false;
  };

  for (const auto &ds : session_.calibration_datasets) {
    if (!clc::dataset_ready(ds)) {
      continue;
    }
    if (ds.image_path.empty()) {
      continue;
    }
    cv::Mat img = cv::imread(ds.image_path, cv::IMREAD_COLOR);
    if (img.empty()) {
      error = "failed to load dataset image: " + ds.image_path;
      return false;
    }
    glim_cloud_fusion::DensePointCloud dense;
    if (!load_fused_for_dataset(ds, dense)) {
      error = "fused cloud missing for dataset: " + ds.id;
      return false;
    }
    input.images.push_back(img);
    input.clouds.push_back(clc::dense_cloud_to_nid_frame(dense));
  }
  if (input.images.empty()) {
    error = "no ready calibration datasets for alignment";
    return false;
  }
  return true;
}

QString AppController::exportFusedPcd() {
  if (fused_cloud_.points.empty()) {
    setStatus(QStringLiteral("No fused cloud to export"));
    return {};
  }
  const QString path =
      QFileDialog::getSaveFileName(nullptr, QStringLiteral("Export fused PCD"),
                                   QString(), QStringLiteral("PCD (*.pcd)"));
  if (path.isEmpty()) {
    return {};
  }
  reprojectViewerCameraRgb();
  std::string err;
  std::vector<float> rgb(static_cast<size_t>(viewer_rgb_.size()));
  for (int i = 0; i < viewer_rgb_.size(); ++i) {
    rgb[i] = viewer_rgb_[i];
  }
  if (!clc::save_dense_pcd_rgb(path.toStdString(), fused_cloud_, rgb, err)) {
    setStatus(QString::fromStdString(err));
    return {};
  }
  setStatus(QStringLiteral("Exported %1").arg(path));
  return path;
}

void AppController::refreshViewerColormapBounds(bool reset_colormap_range) {
  if (fused_cloud_.points.empty()) {
    return;
  }
  float min_v = std::numeric_limits<float>::infinity();
  float max_v = -std::numeric_limits<float>::infinity();
  const int n = static_cast<int>(fused_cloud_.points.size());
  for (int i = 0; i < n; ++i) {
    float s = 0.f;
    switch (viewer_scalar_channel_) {
    case 1:
      s = fused_cloud_.points[i].x();
      break;
    case 2:
      s = fused_cloud_.points[i].y();
      break;
    case 3:
      s = fused_cloud_.points[i].z();
      break;
    default:
      s = i < static_cast<int>(fused_cloud_.intensities.size())
              ? static_cast<float>(fused_cloud_.intensities[i])
              : 0.f;
      break;
    }
    if (!std::isfinite(s)) {
      continue;
    }
    min_v = std::min(min_v, s);
    max_v = std::max(max_v, s);
  }
  if (!std::isfinite(min_v) || !std::isfinite(max_v) || max_v <= min_v) {
    min_v = 0.f;
    max_v = 1.f;
  }
  viewer_data_min_ = min_v;
  viewer_data_max_ = max_v;
  if (reset_colormap_range) {
    viewer_colormap_min_ = min_v;
    viewer_colormap_max_ = max_v;
    viewer_scalar_bounds_valid_ = true;
  }
}

void AppController::setMaskPolygonPending(bool pending) {
  if (mask_polygon_pending_ == pending) {
    return;
  }
  mask_polygon_pending_ = pending;
  emit maskPolygonPendingChanged();
  emit maskChanged();
}

QVariantList AppController::pendingMaskPolygonVertices() const {
  QVariantList out;
  if (!mask_polygon_pending_) {
    return out;
  }
  for (const auto &v : pending_mask_polygon_.vertices) {
    out.append(QVariant::fromValue(QPointF(v.x(), v.y())));
  }
  return out;
}

std::vector<QPointF> AppController::pendingMaskPolygonNativePoints() const {
  std::vector<QPointF> out;
  if (!mask_polygon_pending_ || pending_mask_polygon_.vertices.size() < 3) {
    return out;
  }
  out.reserve(pending_mask_polygon_.vertices.size());
  for (const auto &v : pending_mask_polygon_.vertices) {
    out.emplace_back(v.x(), v.y());
  }
  return out;
}

int AppController::maskRasterWidth() const {
  if (session_.reference_scan_width > 0) {
    return session_.reference_scan_width;
  }
  return range_image_.width;
}

int AppController::maskRasterHeight() const {
  if (session_.reference_scan_height > 0) {
    return session_.reference_scan_height;
  }
  return range_image_.height;
}

void AppController::rebuildMaskFromOps() {
  const int w = maskRasterWidth();
  const int h = maskRasterHeight();
  if (w <= 0 || h <= 0) {
    emit maskChanged();
    return;
  }
  if (mask_ops_.empty()) {
    global_mask_pixels_.clear();
    emit maskChanged();
    refreshPreviewCloudFromMask(true);
    return;
  }
  global_mask_pixels_ = clc::build_mask_from_ops(w, h, mask_ops_);
  session_.global_mask.width = w;
  session_.global_mask.height = h;
  emit maskChanged();
  refreshPreviewCloudFromMask(true);
}

void AppController::commitMaskPolygonNative(
    const std::vector<QPointF> &nativeVertices) {
  if (nativeVertices.size() < 3) {
    return;
  }
  const int mask_w = maskRasterWidth();
  const int mask_h = maskRasterHeight();
  const int range_w = range_image_.width;
  const int range_h = range_image_.height;
  const qreal sx =
      (mask_w > 0 && range_w > 0 && mask_w != range_w)
          ? static_cast<qreal>(mask_w) / static_cast<qreal>(range_w)
          : 1.0;
  const qreal sy =
      (mask_h > 0 && range_h > 0 && mask_h != range_h)
          ? static_cast<qreal>(mask_h) / static_cast<qreal>(range_h)
          : 1.0;

  pending_mask_polygon_ = {};
  for (const QPointF &p : nativeVertices) {
    pending_mask_polygon_.vertices.emplace_back(static_cast<float>(p.x() * sx),
                                                static_cast<float>(p.y() * sy));
  }
  setMaskPolygonPending(true);
  setStatus(QStringLiteral("Polygon closed — choose Keep or Remove to apply"));
}

void AppController::commitMaskPolygon(const QVariantList &nativeVertices) {
  pending_mask_polygon_ = polygonFromVariantList(nativeVertices);
  if (pending_mask_polygon_.vertices.size() < 3) {
    return;
  }
  setMaskPolygonPending(true);
  setStatus(QStringLiteral("Polygon closed — choose Keep or Remove to apply"));
}

void AppController::applyPendingMaskKeep() {
  if (!mask_polygon_pending_) {
    return;
  }
  mask_ops_.push_back({pending_mask_polygon_, true});
  pending_mask_polygon_ = {};
  setMaskPolygonPending(false);
  rebuildMaskFromOps();
  setStatus(QStringLiteral("Mask updated (keep region, %1 ops)")
                .arg(static_cast<int>(mask_ops_.size())));
}

void AppController::applyPendingMaskRemove() {
  if (!mask_polygon_pending_) {
    return;
  }
  mask_ops_.push_back({pending_mask_polygon_, false});
  pending_mask_polygon_ = {};
  setMaskPolygonPending(false);
  rebuildMaskFromOps();
  setStatus(QStringLiteral("Mask updated (remove region, %1 ops)")
                .arg(static_cast<int>(mask_ops_.size())));
}

#ifdef CLC_APP_TESTING
void AppController::installSyntheticMaskEditorScan(int native_width,
                                                   int native_height,
                                                   double y_stretch) {
  if (native_width < 3 || native_height < 3) {
    return;
  }
  fusion_blocked_ = false;
  session_.reference_scan_width = native_width;
  session_.reference_scan_height = native_height;
  range_image_.width = native_width;
  range_image_.height = native_height;
  const std::size_t n = static_cast<std::size_t>(native_width * native_height);
  range_image_.range.assign(n, 12.f);
  range_image_.intensity.assign(n, 0.5f);
  range_data_min_ = 0.0;
  range_data_max_ = 50.0;
  range_colormap_min_ = 0.0;
  range_colormap_max_ = 50.0;
  range_y_stretch_ = std::max(1.0, y_stretch);
  mask_polygon_pending_ = false;
  pending_mask_polygon_ = {};
  refreshRangeImageDisplay();
  emit maskPolygonPendingChanged();
  emit maskChanged();
}

void AppController::setReferenceScanSizeForTest(int width, int height) {
  session_.reference_scan_width = width;
  session_.reference_scan_height = height;
  emit maskChanged();
}
#endif

void AppController::discardPendingMaskPolygon() {
  if (!mask_polygon_pending_) {
    return;
  }
  pending_mask_polygon_ = {};
  setMaskPolygonPending(false);
  emit maskChanged();
  setStatus(QStringLiteral("Pending polygon discarded"));
}

void AppController::clearMask() {
  mask_ops_.clear();
  pending_mask_polygon_ = {};
  setMaskPolygonPending(false);
  global_mask_pixels_.clear();
  session_.global_mask.path.clear();
  session_.global_mask.width = 0;
  session_.global_mask.height = 0;
  emit maskChanged();
  refreshPreviewCloudFromMask(true);
  setStatus(QStringLiteral("Mask cleared"));
}

bool AppController::saveMask() {
  const int w = session_.reference_scan_width;
  const int h = session_.reference_scan_height;
  if (w <= 0 || h <= 0 ||
      global_mask_pixels_.size() != static_cast<std::size_t>(w * h)) {
    setStatus(
        QStringLiteral("No mask to save (draw polygons or load a mask first)"));
    return false;
  }
  QString dir = session_directory_;
  if (dir.isEmpty()) {
    dir = QFileDialog::getExistingDirectory(
        nullptr, QStringLiteral("Save mask to session folder"));
    if (dir.isEmpty()) {
      return false;
    }
    setSessionDirectory(dir);
  }
  const QString path = dir + QStringLiteral("/global_mask.png");
  if (!clc::save_mask_png(path.toStdString(), w, h, global_mask_pixels_)) {
    setStatus(QStringLiteral("Failed to write mask"));
    return false;
  }
  session_.global_mask.path = path.toStdString();
  session_.global_mask.width = w;
  session_.global_mask.height = h;
  emit maskChanged();
  setStatus(QStringLiteral("Mask saved to %1").arg(path));
  return true;
}

bool AppController::loadMask() {
  const int w = session_.reference_scan_width;
  const int h = session_.reference_scan_height;
  if (w <= 0 || h <= 0) {
    setStatus(QStringLiteral("Load structured PCDs before loading a mask"));
    return false;
  }
  QString path = maskPath();
  if (path.isEmpty()) {
    path = browseMaskFile();
    if (path.isEmpty()) {
      return false;
    }
  }
  std::string error;
  if (!clc::load_mask_png(path.toStdString(), w, h, global_mask_pixels_,
                          error)) {
    setStatus(QString::fromStdString(error));
    return false;
  }
  mask_ops_.clear();
  setMaskPolygonPending(false);
  session_.global_mask.path = path.toStdString();
  session_.global_mask.width = w;
  session_.global_mask.height = h;
  emit maskChanged();
  refreshPreviewCloudFromMask(true);
  setStatus(QStringLiteral("Mask loaded"));
  return true;
}

QString AppController::browseMaskFile() {
  const QString file = QFileDialog::getOpenFileName(
      nullptr, QStringLiteral("Load mask PNG"), session_directory_,
      QStringLiteral("PNG mask (*.png)"));
  if (file.isEmpty()) {
    return QString();
  }
  const int w = session_.reference_scan_width;
  const int h = session_.reference_scan_height;
  if (w <= 0 || h <= 0) {
    setStatus(QStringLiteral("Load structured PCDs before loading a mask"));
    return QString();
  }
  std::string error;
  if (!clc::load_mask_png(file.toStdString(), w, h, global_mask_pixels_,
                          error)) {
    setStatus(QString::fromStdString(error));
    return QString();
  }
  mask_ops_.clear();
  setMaskPolygonPending(false);
  session_.global_mask.path = file.toStdString();
  session_.global_mask.width = w;
  session_.global_mask.height = h;
  emit maskChanged();
  refreshPreviewCloudFromMask(true);
  setStatus(QStringLiteral("Mask loaded"));
  return file;
}

bool AppController::loadFirstReadableScan(clc::PointXYZITCloud &cloud,
                                          std::string &err) {
  const bool check_filename_stamp =
      activeDatasetEntry().pcd_paths.size() > 1;
  for (const auto &path : activeDatasetEntry().pcd_paths) {
    if (clc::load_pcd_xyzit(path, cloud, err, check_filename_stamp)) {
      return true;
    }
  }
  if (err.empty()) {
    err = "no readable PCD in catalog";
  }
  return false;
}

void AppController::refreshRangeImage() {
  const int previous_width = range_image_.width;
  const int previous_height = range_image_.height;
  range_image_ = {};
  range_image_display_ = {};
  if (activeDatasetEntry().pcd_paths.empty() || fusion_blocked_) {
    emit rangeImageChanged();
    return;
  }
  clc::PointXYZITCloud cloud;
  std::string err;
  if (!loadFirstReadableScan(cloud, err)) {
    setStatus(QString::fromStdString(err));
    emit rangeImageChanged();
    return;
  }
  range_image_ = clc::build_range_image(cloud);
  if (previous_width > 0 && previous_height > 0 &&
      (range_image_.width != previous_width ||
       range_image_.height != previous_height)) {
    discardPendingMaskPolygon();
  }
  const auto channel = range_channel_ == 0 ? clc::RangeImageChannel::Range
                                           : clc::RangeImageChannel::Intensity;
  const std::vector<float> &values = channel == clc::RangeImageChannel::Range
                                         ? range_image_.range
                                         : range_image_.intensity;
  if (!values.empty()) {
    const auto [min_it, max_it] =
        std::minmax_element(values.begin(), values.end());
    range_data_min_ = static_cast<double>(*min_it);
    range_data_max_ = static_cast<double>(autoMaxValue(values));
  }
  if (range_colormap_max_ <= range_colormap_min_) {
    range_colormap_min_ = range_data_min_;
    range_colormap_max_ = range_data_max_;
  }
  refreshRangeImageDisplay();
  emit rangeImageChanged();
}

void AppController::refreshRangeImageDisplay() {
  if (range_image_.width <= 0 || range_image_.height <= 0) {
    range_image_display_ = {};
    emit rangeImageChanged();
    return;
  }
  const auto channel = range_channel_ == 0 ? clc::RangeImageChannel::Range
                                           : clc::RangeImageChannel::Intensity;
  range_image_display_ = rangeImageToDisplay(
      range_image_, channel, static_cast<float>(range_colormap_min_),
      static_cast<float>(range_colormap_max_), range_y_stretch_);
  emit rangeImageChanged();
}

void AppController::applySessionToViewer() {
  auto *item = qobject_cast<psiviz::PointCloudItem *>(viewer_);
  if (!item) {
    return;
  }
  item->setPoseMatrix(isometryToPoseMatrix(session_.T_camera_lidar));
}

void AppController::refreshViewerGizmoState() {
  auto *item = qobject_cast<psiviz::PointCloudItem *>(viewer_);
  if (!item) {
    return;
  }
  const bool show_gizmo =
      (main_work_tab_ == 1 || main_work_tab_ == 2) && activeDatasetReady();
  const bool manipulable =
      show_gizmo && main_work_tab_ == 1 && !worker_running_;
  item->setGizmoEnabled(show_gizmo);
  item->setGizmoManipulable(manipulable);
  item->setPickingEnabled(pickingAllowed() && main_work_tab_ == 1);
}

void AppController::loadActiveDatasetCloudToViewer() {
  if (main_work_tab_ != 3 && session_.calibration_datasets.empty()) {
    refreshPreviewInViewer();
    return;
  }
  if (main_work_tab_ == 3) {
    if (!fused_cloud_.points.empty()) {
      uploadFusedCloudToViewer();
    } else {
      refreshPreviewInViewer();
    }
    return;
  }
  auto &ds = activeDatasetEntry();
  const bool fused_cache_for_active =
      fused_cloud_dataset_index_.has_value() &&
      *fused_cloud_dataset_index_ == active_calibration_dataset_index_ &&
      !fused_cloud_.points.empty();
  if (fused_cache_for_active && ds.fusion_complete) {
    uploadFusedCloudToViewer();
    reprojectViewerCameraRgb();
    return;
  }
  if (!ds.fused_cloud_path.empty()) {
    glim_cloud_fusion::DensePointCloud dense;
    std::string err;
    if (clc::load_dense_pcd(ds.fused_cloud_path, dense, err)) {
      fused_cloud_ = dense;
      fused_cloud_dataset_index_ = active_calibration_dataset_index_;
      setPointCount(static_cast<int>(dense.points.size()));
      uploadFusedCloudToViewer();
      return;
    }
  }
  if (!ds.pcd_paths.empty() && !fusion_blocked_) {
    refreshPreviewInViewer();
  } else {
    setPointCount(0);
    if (auto *item = qobject_cast<psiviz::PointCloudItem *>(viewer_)) {
      item->setPointCloudData({}, {}, {});
    }
  }
  refreshViewerGizmoState();
}

void AppController::emitSessionUiState() {
  emit scanTimeChanged();
  emit fusionParamsChanged();
  emit calibrationParamsChanged();
  emit sessionPathsChanged();
  emit pcdCatalogChanged();
  emit sessionLoaded();
  applySessionToViewer();
  if (!activeDatasetEntry().pcd_paths.empty() && !fusion_blocked_) {
    refreshRangeImage();
    if (fused_cloud_.points.empty()) {
      uploadPreviewScanToViewer();
    }
  }
}

bool AppController::loadSession(const QString &directory) {
  const QString dir = normalizedDirectoryPath(directory);
  if (dir.isEmpty()) {
    setStatus(QStringLiteral("Session directory is empty"));
    return false;
  }
  const QString path = dir + QStringLiteral("/session.json");
  if (!clc::load_session(path.toStdString(), session_)) {
    setStatus(QStringLiteral("Failed to load session from %1").arg(path));
    return false;
  }
  clc::prune_placeholder_calibration_datasets(session_);
  invalidateProjectionImageCache();
  fused_cloud_ = {};
  fused_cloud_dataset_index_.reset();
  if (session_.calibration_datasets.empty()) {
    active_calibration_dataset_index_ = 0;
  } else {
    active_calibration_dataset_index_ =
        std::clamp(active_calibration_dataset_index_, 0,
                   static_cast<int>(session_.calibration_datasets.size()) - 1);
  }
  session_.fusion_params.scan_time = session_.scan_time;
  setSessionDirectory(dir);
  if (!session_.calibration_datasets.empty() &&
      !activeDatasetEntry().cloud_directory.empty()) {
    refreshPcdCatalog();
  } else if (!activeDatasetEntry().pcd_paths.empty()) {
    emit pcdCatalogChanged();
  }
  if (!session_.intrinsics_path.empty()) {
    std::string error;
    clc::load_camera_intrinsics_file(session_.intrinsics_path, session_.camera,
                                     error);
  }
  mask_ops_.clear();
  pending_mask_polygon_ = {};
  setMaskPolygonPending(false);
  global_mask_pixels_.clear();
  if (!session_.global_mask.path.empty() && session_.global_mask.width > 0 &&
      session_.global_mask.height > 0) {
    std::string error;
    clc::load_mask_png(session_.global_mask.path, session_.global_mask.width,
                       session_.global_mask.height, global_mask_pixels_, error);
  }
  refreshCameraPreview();
  setStatus(QStringLiteral("Session loaded"));
  emit maskChanged();
  active_calibration_dataset_index_ = 0;
  emit calibrationDatasetsChanged();
  emitSessionUiState();
  emit correspondencesChanged();
  applySessionToViewer();
  refreshViewerGizmoState();
  refreshActiveDatasetVisuals();
  return true;
}

bool AppController::saveSession(const QString &directory) {
  const QString dir = normalizedDirectoryPath(
      directory.isEmpty() ? session_directory_ : directory);
  if (dir.isEmpty()) {
    setStatus(QStringLiteral("Session directory is empty"));
    return false;
  }
  setSessionDirectory(dir);
  clc::prune_placeholder_calibration_datasets(session_);
  session_.fusion_params.scan_time = session_.scan_time;
  if (!global_mask_pixels_.empty() && session_.reference_scan_width > 0 &&
      session_.reference_scan_height > 0 &&
      global_mask_pixels_.size() ==
          static_cast<std::size_t>(session_.reference_scan_width *
                                   session_.reference_scan_height)) {
    const QString mask_path = dir + QStringLiteral("/global_mask.png");
    clc::save_mask_png(mask_path.toStdString(), session_.reference_scan_width,
                       session_.reference_scan_height, global_mask_pixels_);
    session_.global_mask.path = mask_path.toStdString();
    session_.global_mask.width = session_.reference_scan_width;
    session_.global_mask.height = session_.reference_scan_height;
  }
  const QString path = dir + QStringLiteral("/session.json");
  if (!clc::save_session(path.toStdString(), session_)) {
    setStatus(QStringLiteral("Failed to save session to %1").arg(path));
    return false;
  }
  setStatus(QStringLiteral("Session saved"));
  return true;
}

QString AppController::browseSessionDirectory() {
  const QString dir = QFileDialog::getExistingDirectory(
      nullptr, QStringLiteral("Session directory"), session_directory_);
  if (!dir.isEmpty()) {
    setSessionDirectory(dir);
  }
  return session_directory_;
}

QString AppController::browsePcdFile() {
  const QString path =
      pickPcdFileDialog(QStringLiteral("LiDAR scan file"), pcdDirectory());
  if (!path.isEmpty()) {
    setPcdDirectory(path);
  }
  return pcdDirectory();
}

QString AppController::browsePcdFolder() {
  const QString path =
      pickPcdFolderDialog(QStringLiteral("LiDAR scan folder"), pcdDirectory());
  if (!path.isEmpty()) {
    setPcdDirectory(path);
  }
  return pcdDirectory();
}

QString AppController::browseImageFile() {
  const QString file = QFileDialog::getOpenFileName(
      nullptr, QStringLiteral("Camera image"), imagePath(),
      QStringLiteral("Images (*.png *.jpg *.jpeg *.bmp *.tif *.tiff)"));
  if (!file.isEmpty()) {
    setImagePath(file);
  }
  return imagePath();
}

bool AppController::loadIntrinsicsFile(const QString &path) {
  const QString normalized = QFileInfo(path).absoluteFilePath();
  if (normalized.isEmpty()) {
    setStatus(QStringLiteral("Intrinsics path is empty"));
    return false;
  }
  std::string error;
  if (!clc::load_camera_intrinsics_file(normalized.toStdString(),
                                        session_.camera, error)) {
    setStatus(QString::fromStdString(error));
    return false;
  }
  session_.intrinsics_path = normalized.toStdString();
  setStatus(QStringLiteral("Intrinsics loaded (%1)")
                .arg(QString::fromStdString(session_.camera.model)));
  emit sessionPathsChanged();
  return true;
}

void AppController::setViewer(QObject *viewer) {
  if (viewer_ == viewer) {
    return;
  }
  if (viewer_) {
    disconnect(viewer_, nullptr, this, nullptr);
  }
  viewer_ = viewer;
  auto *item = qobject_cast<psiviz::PointCloudItem *>(viewer_);
  if (!item) {
    setStatus(QStringLiteral(
        "3D viewer type mismatch; point cloud preview disabled"));
    return;
  }
  connect(item, &psiviz::PointCloudItem::pointPicked, this,
          &AppController::onPointPicked);
  connect(item, &psiviz::PointCloudItem::poseChanged, this,
          &AppController::onViewerPoseChanged);
  connect(this, &AppController::mainWorkTabChanged, this,
          &AppController::refreshViewerGizmoState);
  syncCorrespondenceMarkers();
  refreshViewerGizmoState();
  applySessionToViewer();
  if (!viewer_positions_.isEmpty()) {
    reprojectViewerCameraRgb();
  } else if (!fused_cloud_.points.empty()) {
    uploadFusedCloudToViewer();
  } else if (preview_upload_pending_ ||
             !activeDatasetEntry().pcd_paths.empty()) {
    uploadPreviewScanToViewer();
  }
}

void AppController::refreshPreviewCloudFromMask(bool preserve_view_state) {
  if (!fused_cloud_.points.empty()) {
    return;
  }
  uploadPreviewScanToViewer(preserve_view_state);
}

void AppController::uploadLidarScanToViewer(const clc::PointXYZITCloud &cloud,
                                            bool preserve_view_state) {
  auto *item = qobject_cast<psiviz::PointCloudItem *>(viewer_);
  if (!item || cloud.scan.points.empty()) {
    return;
  }
  const int n = static_cast<int>(cloud.scan.points.size());
  QVector<float> positions;
  QVector<float> scalars;
  QVector<float> rgb;
  positions.reserve(n * 3);
  scalars.reserve(n);
  rgb.reserve(n * 3);
  float min_v = std::numeric_limits<float>::infinity();
  float max_v = -std::numeric_limits<float>::infinity();
  for (int i = 0; i < n; ++i) {
    const auto &p = cloud.scan.points[i];
    if (!cloud.scan.keep.empty() &&
        i < static_cast<int>(cloud.scan.keep.size()) && !cloud.scan.keep[i]) {
      continue;
    }
    const float x = static_cast<float>(p.x());
    const float y = static_cast<float>(p.y());
    const float z = static_cast<float>(p.z());
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    positions << x << y << z;
    float scalar = 0.f;
    switch (viewer_scalar_channel_) {
    case 1:
      scalar = x;
      break;
    case 2:
      scalar = y;
      break;
    case 3:
      scalar = z;
      break;
    default:
      scalar = i < static_cast<int>(cloud.scan.intensities.size())
                   ? static_cast<float>(cloud.scan.intensities[i])
                   : 0.f;
      break;
    }
    scalars << scalar;
    rgb << 0.f << 0.f << 0.f;
    min_v = std::min(min_v, scalar);
    max_v = std::max(max_v, scalar);
  }
  if (positions.isEmpty()) {
    setPointCount(0);
    setStatus(QStringLiteral(
        "No finite points to show in 3D preview (check mask / PCD)"));
    return;
  }
  if (!preserve_view_state || !viewer_scalar_bounds_valid_) {
    if (std::isfinite(min_v) && std::isfinite(max_v) && max_v > min_v) {
      viewer_data_min_ = min_v;
      viewer_data_max_ = max_v;
      viewer_colormap_min_ = min_v;
      viewer_colormap_max_ = max_v;
    } else if (viewer_scalar_channel_ == 0) {
      viewer_data_min_ = 0.0;
      viewer_data_max_ = 255.0;
      viewer_colormap_min_ = 0.0;
      viewer_colormap_max_ = 255.0;
    } else {
      viewer_data_min_ = 0.0;
      viewer_data_max_ = 1.0;
      viewer_colormap_min_ = 0.0;
      viewer_colormap_max_ = 1.0;
    }
    viewer_scalar_bounds_valid_ = true;
    emit viewerColormapChanged();
  } else if (std::isfinite(min_v) && std::isfinite(max_v) && max_v > min_v) {
    const double new_data_min = static_cast<double>(min_v);
    const double new_data_max = static_cast<double>(max_v);
    if (new_data_min < viewer_data_min_ || new_data_max > viewer_data_max_) {
      viewer_data_min_ = std::min(viewer_data_min_, new_data_min);
      viewer_data_max_ = std::max(viewer_data_max_, new_data_max);
      emit viewerColormapChanged();
    }
  }
  item->setScalarMin(viewer_colormap_min_);
  item->setScalarMax(viewer_colormap_max_);
  viewer_positions_ = positions;
  viewer_scalars_ = scalars;
  viewer_rgb_ = rgb;
  syncColorModelFromSession();
  reprojectViewerCameraRgb();
  const int shown = static_cast<int>(viewer_positions_.size() / 3);
  setPointCount(shown);
  setStatus(QStringLiteral("3D preview: %1 points (intensity)").arg(shown));
}

void AppController::uploadPreviewScanToViewer(bool preserve_view_state) {
  if (fusion_blocked_ || activeDatasetEntry().pcd_paths.empty()) {
    preview_upload_pending_ = false;
    return;
  }
  if (!viewer_) {
    preview_upload_pending_ = true;
    return;
  }
  preview_upload_pending_ = false;
  clc::PointXYZITCloud cloud;
  std::string err;
  if (!loadFirstReadableScan(cloud, err)) {
    setStatus(QString::fromStdString(err.empty() ? "failed to load preview scan"
                                                 : err));
    setPointCount(0);
    return;
  }
  cloud.scan = clc::to_lidar_scan_masked(cloud, global_mask_pixels_);
  uploadLidarScanToViewer(cloud, preserve_view_state);
}

void AppController::uploadDenseCloudToViewer(
    const glim_cloud_fusion::DensePointCloud &cloud, bool preserve_view_state) {
  auto *item = qobject_cast<psiviz::PointCloudItem *>(viewer_);
  if (!item || cloud.points.empty()) {
    return;
  }
  QVector<float> positions;
  QVector<float> scalars;
  QVector<float> rgb;
  const int n = static_cast<int>(cloud.points.size());
  positions.reserve(n * 3);
  scalars.reserve(n);
  rgb.reserve(n * 3);
  float min_v = std::numeric_limits<float>::infinity();
  float max_v = -std::numeric_limits<float>::infinity();
  for (int i = 0; i < n; ++i) {
    const auto &p = cloud.points[i];
    positions << static_cast<float>(p.x()) << static_cast<float>(p.y())
              << static_cast<float>(p.z());
    float scalar = 0.f;
    switch (viewer_scalar_channel_) {
    case 1:
      scalar = static_cast<float>(p.x());
      break;
    case 2:
      scalar = static_cast<float>(p.y());
      break;
    case 3:
      scalar = static_cast<float>(p.z());
      break;
    default:
      scalar = i < static_cast<int>(cloud.intensities.size())
                   ? static_cast<float>(cloud.intensities[i])
                   : 0.f;
      break;
    }
    scalars << scalar;
    rgb << 0.f << 0.f << 0.f;
    if (std::isfinite(scalar)) {
      min_v = std::min(min_v, scalar);
      max_v = std::max(max_v, scalar);
    }
  }
  if (!preserve_view_state || !viewer_scalar_bounds_valid_) {
    if (std::isfinite(min_v) && std::isfinite(max_v) && max_v > min_v) {
      viewer_data_min_ = min_v;
      viewer_data_max_ = max_v;
      viewer_colormap_min_ = min_v;
      viewer_colormap_max_ = max_v;
    } else if (viewer_scalar_channel_ == 0) {
      viewer_data_min_ = 0.0;
      viewer_data_max_ = 255.0;
      viewer_colormap_min_ = 0.0;
      viewer_colormap_max_ = 255.0;
    }
    viewer_scalar_bounds_valid_ = true;
    emit viewerColormapChanged();
  } else if (std::isfinite(min_v) && std::isfinite(max_v) && max_v > min_v) {
    const double new_data_min = static_cast<double>(min_v);
    const double new_data_max = static_cast<double>(max_v);
    if (new_data_min < viewer_data_min_ || new_data_max > viewer_data_max_) {
      viewer_data_min_ = std::min(viewer_data_min_, new_data_min);
      viewer_data_max_ = std::max(viewer_data_max_, new_data_max);
      emit viewerColormapChanged();
    }
  }
  item->setScalarMin(viewer_colormap_min_);
  item->setScalarMax(viewer_colormap_max_);
  viewer_positions_ = positions;
  viewer_scalars_ = scalars;
  viewer_rgb_ = rgb;
  syncColorModelFromSession();
  reprojectViewerCameraRgb();
  setPointCount(static_cast<int>(cloud.points.size()));
}

void AppController::uploadFusedCloudToViewer() {
  uploadDenseCloudToViewer(fused_cloud_, viewer_scalar_bounds_valid_);
}

QString AppController::exportInspectionColoredPcd() {
  if (main_work_tab_ != 3) {
    setStatus(QStringLiteral(
        "Switch to the Inspection tab to export inspection clouds"));
    return {};
  }
  return exportFusedPcd();
}

void AppController::syncCorrespondenceMarkers() {
  auto *item = qobject_cast<psiviz::PointCloudItem *>(viewer_);
  if (!item) {
    return;
  }
  if (main_work_tab_ == 3) {
    item->clearSphereMarkers();
    return;
  }
  const double marker_radius = 0.05;

  item->setMarkerSphereRadius(marker_radius);
  QVector<double> positions;
  for (const auto &ds : session_.calibration_datasets) {
    for (const auto &c : ds.correspondences) {
      positions << c.x << c.y << c.z;
    }
  }
  item->setSphereMarkerPositions(positions);
  if (pending_correspondence_3d_) {
    item->setPendingSphereMarker((*pending_correspondence_3d_).x(),
                                 (*pending_correspondence_3d_).y(),
                                 (*pending_correspondence_3d_).z());
  } else {
    item->clearPendingSphereMarker();
  }
}

void AppController::finalizePendingCorrespondenceIfReady() {
  if (!pending_correspondence_3d_.has_value() ||
      !pending_correspondence_2d_.has_value()) {
    return;
  }
  ensureActiveCalibrationDataset();
  auto &ds = activeDatasetEntry();
  clc::PointCorrespondence c;
  c.u = pending_correspondence_2d_->first;
  c.v = pending_correspondence_2d_->second;
  c.x = pending_correspondence_3d_->x();
  c.y = pending_correspondence_3d_->y();
  c.z = pending_correspondence_3d_->z();
  ds.correspondences.push_back(c);
  pending_correspondence_3d_.reset();
  pending_correspondence_2d_.reset();
  selected_correspondence_row_ = -1;
  emit correspondencesChanged();
  setStatus(QStringLiteral("Correspondence added (%1 total)")
                .arg(correspondenceCount()));
}

void AppController::onPointPicked(double x, double y, double z) {
  qInfo().noquote() << QStringLiteral("3D correspondence pick: x=%1 y=%2 z=%3")
                           .arg(x, 0, 'f', 4)
                           .arg(y, 0, 'f', 4)
                           .arg(z, 0, 'f', 4);
  if (main_work_tab_ == 3) {
    setStatus(QStringLiteral(
        "Correspondence picking is only available outside Inspection"));
    return;
  }
  if (main_work_tab_ != 1) {
    return;
  }
  if (!pickingAllowed()) {
    return;
  }
  if (selected_correspondence_row_ >= 0) {
    int cursor = 0;
    for (int di = 0;
         di < static_cast<int>(session_.calibration_datasets.size()); ++di) {
      auto &row_ds =
          session_.calibration_datasets[static_cast<std::size_t>(di)];
      const int count = static_cast<int>(row_ds.correspondences.size());
      if (selected_correspondence_row_ >= cursor &&
          selected_correspondence_row_ < cursor + count) {
        if (di != active_calibration_dataset_index_) {
          setStatus(QStringLiteral(
              "Switch to that dataset to edit its correspondences"));
          return;
        }
        auto &c = row_ds.correspondences[static_cast<std::size_t>(
            selected_correspondence_row_ - cursor)];
        c.x = x;
        c.y = y;
        c.z = z;
        pending_correspondence_3d_.reset();
        pending_correspondence_2d_.reset();
        emit correspondencesChanged();
        setStatus(QStringLiteral("Updated 3D point for selected correspondence "
                                 "— Shift+click image to update pixel"));
        return;
      }
      cursor += count;
    }
  }
  pending_correspondence_3d_ = Eigen::Vector3d(x, y, z);
  if (pending_correspondence_2d_.has_value()) {
    finalizePendingCorrespondenceIfReady();
    return;
  }
  emit correspondencesChanged();
  setStatus(QStringLiteral("3D point picked — Shift+click the matching "
                           "location on the camera image"));
}

void AppController::pickImageCorrespondence(double u, double v) {
  if (main_work_tab_ != 1 || !pickingAllowed()) {
    return;
  }
  ensureActiveCalibrationDataset();
  auto &ds = activeDatasetEntry();
  if (selected_correspondence_row_ >= 0) {
    int cursor = 0;
    for (int di = 0;
         di < static_cast<int>(session_.calibration_datasets.size()); ++di) {
      auto &row_ds =
          session_.calibration_datasets[static_cast<std::size_t>(di)];
      const int count = static_cast<int>(row_ds.correspondences.size());
      if (selected_correspondence_row_ >= cursor &&
          selected_correspondence_row_ < cursor + count) {
        if (di != active_calibration_dataset_index_) {
          setStatus(QStringLiteral(
              "Switch to that dataset to edit its correspondences"));
          return;
        }
        auto &c = row_ds.correspondences[static_cast<std::size_t>(
            selected_correspondence_row_ - cursor)];
        c.u = u;
        c.v = v;
        if (pending_correspondence_3d_) {
          c.x = (*pending_correspondence_3d_).x();
          c.y = (*pending_correspondence_3d_).y();
          c.z = (*pending_correspondence_3d_).z();
          pending_correspondence_3d_.reset();
        }
        pending_correspondence_2d_.reset();
        emit correspondencesChanged();
        return;
      }
      cursor += count;
    }
  }
  pending_correspondence_2d_ = std::make_pair(u, v);
  if (pending_correspondence_3d_.has_value()) {
    finalizePendingCorrespondenceIfReady();
    return;
  }
  emit correspondencesChanged();
  setStatus(QStringLiteral(
      "Image point picked — Shift+click the matching 3D point in the viewer"));
}

void AppController::clearCorrespondences() {
  ensureActiveCalibrationDataset();
  activeDatasetEntry().correspondences.clear();
  pending_correspondence_3d_.reset();
  pending_correspondence_2d_.reset();
  selected_correspondence_row_ = -1;
  emit correspondencesChanged();
  setStatus(QStringLiteral("Correspondences cleared for active dataset"));
}

void AppController::estimateInitialGuessFromCorrespondences() {
  if (session_.camera.intrinsics.empty()) {
    setStatus(QStringLiteral(
        "Load camera intrinsics before estimating initial guess"));
    return;
  }
  clc::ManualGuessInput input;
  input.camera =
      camera::create_camera(session_.camera.model, session_.camera.intrinsics,
                            session_.camera.distortion_coeffs);
  for (const auto &ds : session_.calibration_datasets) {
    for (const auto &c : ds.correspondences) {
      input.correspondences.push_back(c);
    }
  }
  input.gizmo_T_camera_lidar = session_.T_camera_lidar;
  clc::InitialGuessResult result;
  try {
    result = clc::compute_manual_initial_guess(input);
  } catch (const std::exception &e) {
    setStatus(QString::fromUtf8(e.what()));
    return;
  }
  if (!result.ok) {
    setStatus(QString::fromStdString(result.message));
    return;
  }
  session_.T_camera_lidar_initial = result.T_camera_lidar;
  session_.T_camera_lidar = result.T_camera_lidar;
  applySessionToViewer();
  reprojectViewerCameraRgb();
  setStatus(QString::fromStdString(result.message));
}

void AppController::onViewerPoseChanged() {
  auto *item = qobject_cast<psiviz::PointCloudItem *>(viewer_);
  if (!item) {
    return;
  }
  const QVariantList m = item->poseMatrix();
  if (m.size() != 16) {
    return;
  }
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  int k = 0;
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      T(r, c) = m[k++].toDouble();
    }
  }
  session_.T_camera_lidar_initial = Eigen::Isometry3d(T);
  session_.T_camera_lidar = session_.T_camera_lidar_initial;
  reprojectViewerCameraRgb();
}

void AppController::runFusion() {
  if (worker_running_) {
    return;
  }
  if (clc::dataset_is_single_scan(activeDatasetEntry())) {
    setStatus(QStringLiteral("Fusion is not used for single-scan datasets"));
    return;
  }
  if (fusion_blocked_) {
    setStatus(QStringLiteral("Fusion blocked: fix PCD scan geometry"));
    return;
  }
  if (!repopulateActiveDatasetPcdCatalogIfEmpty()) {
    if (activeDatasetEntry().pcd_paths.empty()) {
      setStatus(QStringLiteral("No PCD files loaded"));
    }
    return;
  }
  const auto &catalog = activeDatasetEntry().pcd_paths;
  if (!clc::dataset_is_single_scan(activeDatasetEntry()) &&
      !clc::scan_time_duration_valid(
          activeDatasetEntry().fusion_params.scan_time)) {
    setStatus(QStringLiteral(
        "Set scan duration (seconds) in Scan time before fusion"));
    return;
  }
  setWorkerRunning(true);
  setStatus(QStringLiteral("Fusion running..."));
  joinWorker();
  const int ref_w = session_.reference_scan_width;
  const int ref_h = session_.reference_scan_height;
  syncActiveDatasetFusionToSession();
  const auto fusion_params = activeDatasetEntry().fusion_params;
  const std::vector<std::uint8_t> mask_pixels = global_mask_pixels_;
  worker_ = std::make_unique<std::thread>([this, catalog, ref_w, ref_h,
                                           fusion_params, mask_pixels]() {
    auto params = fusion_params;
    params.scan_time = fusion_params.scan_time;
    const int total_scans = static_cast<int>(catalog.size());
    const bool check_filename_stamp = catalog.size() > 1;

    std::vector<glim_cloud_fusion::LidarScan> scans;
    for (const auto &path : catalog) {
      clc::PointXYZITCloud cloud;
      std::string err;
      if (!clc::load_pcd_xyzit(path, cloud, err, check_filename_stamp)) {
        QMetaObject::invokeMethod(
            this,
            [this, err]() {
              setWorkerRunning(false);
              setStatus(QString::fromStdString(err));
              emit fusionFinished(false, QString::fromStdString(err));
            },
            Qt::QueuedConnection);
        return;
      }
      if (ref_w > 0 && (cloud.width != ref_w || cloud.height != ref_h)) {
        const std::string dim_err = "scan dimension mismatch in " + path;
        QMetaObject::invokeMethod(
            this,
            [this, dim_err]() {
              setWorkerRunning(false);
              setStatus(QString::fromStdString(dim_err));
              emit fusionFinished(false, QString::fromStdString(dim_err));
            },
            Qt::QueuedConnection);
        return;
      }
      scans.push_back(clc::to_lidar_scan_masked(cloud, mask_pixels));
    }

    const bool integrator_debug = params.integrator_debug;
    const auto result = clc::run_fusion(
        scans, params,
        [this, integrator_debug](const glim_cloud_fusion::FusionProgress &p) {
          if (integrator_debug && p.integrator.has_value()) {
            glim_cloud_fusion::print_fusion_integrator_diagnostics(
                std::cerr, p.scan_index, *p.integrator);
          }
          QMetaObject::invokeMethod(
              this,
              [this, p]() {
                setStatus(QStringLiteral("Fusion %1/%2: %3")
                              .arg(p.scan_index)
                              .arg(p.total_scans)
                              .arg(QString::fromStdString(p.stage)));
              },
              Qt::QueuedConnection);
        });

    QMetaObject::invokeMethod(
        this,
        [this, result]() {
          setWorkerRunning(false);
          if (result.error != glim_cloud_fusion::FusionError::Ok) {
            setStatus(QString::fromStdString(result.message));
            emit fusionFinished(false, QString::fromStdString(result.message));
            return;
          }
          fused_cloud_ = result.cloud;
          if (!result.scan_poses.empty()) {
            clc::transform_dense_cloud_from_world(fused_cloud_,
                                                  result.scan_poses.front());
          }
          fused_cloud_dataset_index_ = active_calibration_dataset_index_;
          auto &ds = activeDatasetEntry();
          ds.fusion_complete = true;
          if (!session_directory_.isEmpty() && !ds.id.empty()) {
            const QString fused_path =
                session_directory_ + QStringLiteral("/fused_") +
                QString::fromStdString(ds.id) + QStringLiteral(".pcd");
            std::string save_err;
            if (clc::save_dense_pcd(fused_path.toStdString(), fused_cloud_,
                                    save_err)) {
              ds.fused_cloud_path = fused_path.toStdString();
            }
          }
          setPointCount(static_cast<int>(result.cloud.points.size()));
          refreshViewerColormapBounds(false);
          emit viewerColormapChanged();
          emit calibrationDatasetsChanged();
          setStatus(QStringLiteral("Fusion complete"));
          uploadFusedCloudToViewer();
          refreshViewerGizmoState();
          emit fusionFinished(true, QStringLiteral("Fusion complete"));
        },
        Qt::QueuedConnection);
  });
}

void AppController::runAlignment() {
  if (worker_running_) {
    return;
  }
  if (session_.camera.intrinsics.empty()) {
    setStatus(QStringLiteral("Load camera intrinsics before alignment"));
    return;
  }
  bool any_ready = false;
  for (const auto &ds : session_.calibration_datasets) {
    if (clc::dataset_ready(ds)) {
      any_ready = true;
      break;
    }
  }
  if (!any_ready) {
    setStatus(QStringLiteral("No ready calibration datasets (fuse multi-scan "
                             "sets or complete initial alignment)"));
    return;
  }
  setWorkerRunning(true);
  setStatus(QStringLiteral("Alignment running..."));
  refreshViewerGizmoState();
  joinWorker();
  worker_ = std::make_unique<std::thread>([this]() {
    clc::AlignmentInput input;
    std::string build_err;
    if (!buildAlignmentInput(input, build_err)) {
      QMetaObject::invokeMethod(
          this,
          [this, build_err]() {
            setWorkerRunning(false);
            refreshViewerGizmoState();
            setStatus(QString::fromStdString(build_err));
            emit alignmentFinished(false, QString::fromStdString(build_err));
          },
          Qt::QueuedConnection);
      return;
    }

    const auto result =
        clc::run_alignment(input, [this](const vlcal::CalibrationProgress &p) {
          QMetaObject::invokeMethod(
              this,
              [this, p]() {
                session_.T_camera_lidar = p.T_camera_lidar;
                applySessionToViewer();
                reprojectViewerCameraRgb();
                setStatus(QStringLiteral("cost=%1 outer=%2 inner=%3")
                              .arg(p.cost, 0, 'g', 6)
                              .arg(p.outer_iteration)
                              .arg(p.inner_iteration));
              },
              Qt::QueuedConnection);
        });

    QMetaObject::invokeMethod(
        this,
        [this, result]() {
          setWorkerRunning(false);
          refreshViewerGizmoState();
          if (result.error != vlcal::CalibrationError::Ok) {
            setStatus(QString::fromStdString(result.message));
            emit alignmentFinished(false,
                                   QString::fromStdString(result.message));
            return;
          }
          session_.T_camera_lidar = result.T_camera_lidar;
          applySessionToViewer();
          reprojectViewerCameraRgb();
          setStatus(QStringLiteral("Alignment complete"));
          emit alignmentFinished(true, QStringLiteral("Alignment complete"));
        },
        Qt::QueuedConnection);
  });
}
