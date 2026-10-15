#include "app_controller.hpp"
#include "app_logging.hpp"

#include <QMetaObject>
#include <QPointF>
#include <QVariantMap>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <filesystem>
#include <optional>

#include <clc/linalg.hpp>

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

// File-local helpers for QVariant/QML geometry and display images (not exposed to QML).

namespace {

/// Parse a QPointF from QVariant (point type or {x,y} map).
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

/// Build a clc polygon from QML vertex list (skips invalid entries).
clc::Polygon2f polygonFromVariantList(const QVariantList &native_vertices) {
  clc::Polygon2f polygon;
  for (const QVariant &v : native_vertices) {
    const auto point = variantToPoint(v);
    if (!point.has_value()) {
      continue;
    }
    polygon.vertices.push_back(
        {static_cast<float>(point->x()), static_cast<float>(point->y())});
  }
  return polygon;
}

QVariantList mat4dToPoseMatrix(const clc::Mat4d& T) {
  QVariantList out;
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      out.append(T(r, c));
    }
  }
  return out;
}

clc::Mat4f mat4dToMat4f(const clc::Mat4d& T) {
  clc::Mat4f m;
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      m(r, c) = static_cast<float>(T(r, c));
    }
  }
  return m;
}

/// Session `T_camera_lidar` maps lidar points into the camera frame; draw pose is its inverse.
QVariantList cameraPoseInLidarFrameMatrix(const clc::Mat4d& T_camera_lidar) {
  return mat4dToPoseMatrix(clc::inverseRigid(T_camera_lidar));
}

/// Apply JET colormap and optional vertical stretch for the range-image QML item.
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

/// Largest finite value in a channel, or 1.0 if empty (default colormap max).
float autoMaxValue(const std::vector<float> &values) {
  float max_v = 0.f;
  for (float v : values) {
    if (std::isfinite(v)) {
      max_v = std::max(max_v, v);
    }
  }
  return max_v > 0.f ? max_v : 1.f;
}

/// Return absolute directory path (if `path` is a file, use its parent folder).
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

/// Normalize a user-selected path to an absolute file or directory for PCD catalog logic.
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

/// Native file dialog for choosing one .pcd scan.
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

/// Native folder dialog for a directory of structured scans.
QString pickPcdFolderDialog(const QString &title, const QString &start_path) {
  return QFileDialog::getExistingDirectory(nullptr, title,
                                           normalizedDirectoryPath(start_path));
}

QString registrationTypeLabel(vlcal::RegistrationType type) {
  if (type == vlcal::RegistrationType::NID_NELDER_MEAD) {
    return QStringLiteral("NID_NELDER_MEAD");
  }
  if (type == vlcal::RegistrationType::NID_BFGS) {
    return QStringLiteral("NID_BFGS");
  }
  return QStringLiteral("unknown");
}

/// Emit one structured line describing a calibration dataset row.
void logDatasetCatalogEntry(int index, const clc::DatasetEntry &ds,
                            const QString &action) {
  qCInfo(lcClcDataset).noquote()
      << QStringLiteral("%1 dataset[%2] id=%3 ready=%4 fusion_complete=%5 "
                        "scans=%6 image=%7 lidar=%8 fused_pcd=%9 "
                        "correspondences=%10")
             .arg(action)
             .arg(index)
             .arg(QString::fromStdString(ds.id))
             .arg(clc::dataset_ready(ds) ? QStringLiteral("yes")
                                         : QStringLiteral("no"))
             .arg(ds.fusion_complete ? QStringLiteral("yes")
                                     : QStringLiteral("no"))
             .arg(static_cast<int>(ds.pcd_paths.size()))
             .arg(QString::fromStdString(ds.image_path))
             .arg(QString::fromStdString(ds.cloud_directory))
             .arg(QString::fromStdString(ds.fused_cloud_path))
             .arg(static_cast<int>(ds.correspondences.size()));
}

/// Summarize frames passed into NID after `buildAlignmentInput` succeeds.
void logAlignmentOptimizationInput(const clc::AlignmentInput &input,
                                   int skipped_not_ready) {
  qCInfo(lcClcOptimization).noquote()
      << QStringLiteral(
             "alignment input: frames=%1 skipped_not_ready=%2 camera_model=%3 "
             "registration=%4 init_T_camera_lidar=[%5,%6,%7]")
             .arg(static_cast<int>(input.images.size()))
             .arg(skipped_not_ready)
             .arg(input.camera ? QStringLiteral("yes") : QStringLiteral("no"))
             .arg(registrationTypeLabel(input.params.registration_type))
             .arg(input.init_T_camera_lidar(0, 3), 0, 'g', 6)
             .arg(input.init_T_camera_lidar(1, 3), 0, 'g', 6)
             .arg(input.init_T_camera_lidar(2, 3), 0, 'g', 6);
  for (std::size_t i = 0; i < input.images.size(); ++i) {
    const cv::Mat &img = input.images[i];
    const auto &cloud = input.clouds[i];
    const std::size_t points =
        cloud ? static_cast<std::size_t>(cloud->size()) : 0U;
    qCInfo(lcClcOptimization).noquote()
        << QStringLiteral("  frame[%1] image=%2x%3 cloud_points=%4")
               .arg(static_cast<int>(i))
               .arg(img.cols)
               .arg(img.rows)
               .arg(static_cast<qulonglong>(points));
  }
}

} // namespace

// --- AppController (QML-facing session, fusion, calibration, viewer) ---

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

AppController::~AppController() {
  worker_cancel_requested_ = true;
  joinWorker();
}

/// Wait for the background worker thread to finish.
void AppController::joinWorker() {
  if (worker_ && worker_->joinable()) {
    worker_->join();
  }
  worker_.reset();
}

/// Update busy flag and gizmo interaction state.
void AppController::setWorkerRunning(bool running) {
  const bool was = worker_running_.exchange(running);
  if (was != running) {
    emit busyWorkingChanged();
    emit activeDatasetSelectionLockChanged();
    refreshViewerAnnotationState();
  }
  if (!running) {
    worker_cancel_requested_ = false;
  }
}

/// Request cancellation of the active fusion or alignment job.
void AppController::cancelWorker() {
  if (!worker_running_.load()) {
    return;
  }
  worker_cancel_requested_ = true;
  setStatus(QStringLiteral("Cancelling..."));
}

/// AppController::scanDuration implementation.
double AppController::scanDuration() const {
  if (main_work_tab_ == 3) {
    return session_.scan_time.scan_duration.value_or(0.0);
  }
  return activeDatasetEntry().scan_time.scan_duration.value_or(0.0);
}

/// AppController::timeOrigin implementation.
int AppController::timeOrigin() const {
  if (main_work_tab_ == 3) {
    return static_cast<int>(session_.scan_time.time_origin);
  }
  return static_cast<int>(activeDatasetEntry().scan_time.time_origin);
}

/// AppController::timeUnit implementation.
int AppController::timeUnit() const {
  if (main_work_tab_ == 3) {
    return static_cast<int>(session_.scan_time.time_unit);
  }
  return static_cast<int>(activeDatasetEntry().scan_time.time_unit);
}

/// Update Scan Duration (QML property backing).
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

/// Update Time Origin (QML property backing).
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

/// Update Time Unit (QML property backing).
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

/// Update Status (QML property backing).
void AppController::setStatus(const QString &message) {
  status_message_ = message;
  emit statusMessageChanged();
}

/// Update Point Count (QML property backing).
void AppController::setPointCount(int count) {
  point_count_ = count;
  emit pointCountChanged();
}

/// Create or validate Active Calibration Dataset before continuing.
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

/// AppController::pcdFileCount implementation.
int AppController::pcdFileCount() const {
  return static_cast<int>(activeDatasetEntry().pcd_paths.size());
}

/// AppController::fusionVoxelResolution implementation.
double AppController::fusionVoxelResolution() const {
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    return activeDatasetEntry().fusion_params.voxel_resolution;
  }
  return session_.fusion_params.voxel_resolution;
}

/// AppController::fusionNumThreads implementation.
int AppController::fusionNumThreads() const {
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    return activeDatasetEntry().fusion_params.num_threads;
  }
  return session_.fusion_params.num_threads;
}

/// AppController::fusionMaxCorrespondence implementation.
double AppController::fusionMaxCorrespondence() const {
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    return activeDatasetEntry().fusion_params.max_correspondence_distance;
  }
  return session_.fusion_params.max_correspondence_distance;
}

/// AppController::fusionKNeighbors implementation.
int AppController::fusionKNeighbors() const {
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    return activeDatasetEntry().fusion_params.k_neighbors;
  }
  return session_.fusion_params.k_neighbors;
}

/// AppController::fusionBackend implementation.
int AppController::fusionBackend() const {
  const auto backend =
      (main_work_tab_ != 3 && !session_.calibration_datasets.empty())
          ? activeDatasetEntry().fusion_params.backend
          : session_.fusion_params.backend;
  return backend == glim_cloud_fusion::FusionBackend::GlimSlam ? 1 : 0;
}

/// AppController::fusionBackendIsIntegrator implementation.
bool AppController::fusionBackendIsIntegrator() const {
  return fusionBackend() == 0;
}

/// AppController::fusionTargetNumPoints implementation.
int AppController::fusionTargetNumPoints() const {
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    return activeDatasetEntry().fusion_params.target_num_points;
  }
  return session_.fusion_params.target_num_points;
}

/// AppController::fusionMinDistance implementation.
double AppController::fusionMinDistance() const {
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    return activeDatasetEntry().fusion_params.min_distance;
  }
  return session_.fusion_params.min_distance;
}

/// AppController::fusionIntegratorIvoxResolution implementation.
double AppController::fusionIntegratorIvoxResolution() const {
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    return activeDatasetEntry().fusion_params.integrator_ivox_resolution;
  }
  return session_.fusion_params.integrator_ivox_resolution;
}

/// AppController::fusionIntegratorIvoxMinPointsDist implementation.
double AppController::fusionIntegratorIvoxMinPointsDist() const {
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    return activeDatasetEntry().fusion_params.integrator_ivox_min_points_dist;
  }
  return session_.fusion_params.integrator_ivox_min_points_dist;
}

/// AppController::fusionIvoxResolution implementation.
double AppController::fusionIvoxResolution() const {
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    return activeDatasetEntry().fusion_params.ivox_resolution;
  }
  return session_.fusion_params.ivox_resolution;
}

/// AppController::fusionLmMaxIterations implementation.
int AppController::fusionLmMaxIterations() const {
  if (main_work_tab_ != 3 && !session_.calibration_datasets.empty()) {
    return activeDatasetEntry().fusion_params.lm_max_iterations;
  }
  return session_.fusion_params.lm_max_iterations;
}

/// AppController::correspondenceCount implementation.
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

/// AppController::imagePath implementation.
QString AppController::imagePath() const {
  return QString::fromStdString(activeDatasetEntry().image_path);
}

/// AppController::pcdDirectory implementation.
QString AppController::pcdDirectory() const {
  return QString::fromStdString(activeDatasetEntry().cloud_directory);
}

/// AppController::intrinsicsPath implementation.
QString AppController::intrinsicsPath() const {
  return QString::fromStdString(session_.intrinsics_path);
}

/// Update Session Directory (QML property backing).
void AppController::setSessionDirectory(const QString &directory) {
  const QString normalized = normalizedDirectoryPath(directory);
  if (session_directory_ == normalized) {
    return;
  }
  session_directory_ = normalized;
  emit sessionPathsChanged();
}

/// Update Image Path (QML property backing).
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
  auto &dataset = activeDatasetEntry();
  clc::DatasetEntry with_image = dataset;
  with_image.image_path = normalized.toStdString();
  std::string image_time_err;
  if (!clc::validate_dataset_image_first_scan_time(with_image, image_time_err)) {
    setStatus(QString::fromStdString(image_time_err));
    return;
  }
  dataset.image_path = normalized.toStdString();
  if (!dataset.pcd_paths.empty()) {
    dataset.geometry_error = false;
    fusion_blocked_ = false;
  }
  qCInfo(lcClcDataset).noquote()
      << QStringLiteral("active dataset image set: %1").arg(normalized);
  invalidateProjectionImageCache();
  refreshCameraPreview();
  if (!viewer_positions_.isEmpty()) {
    reprojectViewerCameraRgb();
  }
  emit sessionPathsChanged();
}

/// Update Pcd Directory (QML property backing).
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
  qCInfo(lcClcDataset).noquote()
      << QStringLiteral("active dataset lidar catalog path set: %1")
             .arg(normalized);
  refreshPcdCatalog();
  if (main_work_tab_ == 3) {
    loadActiveDatasetCloudToViewer();
  }
  emit sessionPathsChanged();
}

/// Update Inspection Mode (QML property backing).
void AppController::setInspectionMode(bool inspection) {
  setMainWorkTab(inspection ? 3 : 0);
}

/// Update Main Work Tab (QML property backing).
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
  refreshViewerAnnotationState();
  syncCorrespondenceMarkers();
}

/// Update Active Calibration Dataset Index (QML property backing).
void AppController::setActiveCalibrationDatasetIndex(int index) {
  if (!canChangeActiveCalibrationDataset()) {
    return;
  }
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
  // Fused vs preview intensities use different scales; recompute colormap limits per dataset.
  viewer_scalar_bounds_valid_ = false;
  syncActiveDatasetFusionToSession();
  emit calibrationDatasetsChanged();
  emit sessionPathsChanged();
  emit scanTimeChanged();
  emit fusionParamsChanged();
  emit pcdCatalogChanged();
  refreshActiveDatasetVisuals();
  if (!mask_editor_open_) {
    refreshRangeImage();
  }
  emit imageMaskChanged();
  emit correspondencesChanged();
}

/// AppController::canChangeActiveCalibrationDataset implementation.
bool AppController::canChangeActiveCalibrationDataset() const {
  return !worker_running_ && !mask_editor_open_ && !image_mask_editor_open_;
}

/// AppController::calibrationDatasetIds implementation.
QStringList AppController::calibrationDatasetIds() const {
  QStringList ids;
  for (const auto &ds : session_.calibration_datasets) {
    ids.append(QString::fromStdString(
        ds.id.empty() ? QStringLiteral("(unnamed)").toStdString() : ds.id));
  }
  return ids;
}

/// AppController::activeDatasetReady implementation.
bool AppController::activeDatasetReady() const {
  return isCalibrationDatasetReady(active_calibration_dataset_index_);
}

/// AppController::canPickCorrespondences implementation.
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

/// AppController::isCalibrationDatasetReady implementation.
bool AppController::isCalibrationDatasetReady(int index) const {
  if (index < 0 ||
      index >= static_cast<int>(session_.calibration_datasets.size())) {
    return false;
  }
  return clc::dataset_ready(
      session_.calibration_datasets[static_cast<std::size_t>(index)]);
}

/// AppController::calibrationDatasetTooltip implementation.
QString AppController::calibrationDatasetTooltip(int index) const {
  if (isCalibrationDatasetReady(index)) {
    return QStringLiteral("Ready for initial alignment and calibration");
  }
  return QStringLiteral(
      "Load image and LiDAR; fuse multi-scan datasets before NID");
}

/// AppController::manualInitialGuessSupportsMultiDataset implementation.
bool AppController::manualInitialGuessSupportsMultiDataset() const {
  return clc::InitialGuessTraits<
      clc::ManualGuessProvider>::supports_multi_dataset;
}

/// Update Initial Alignment Method (QML property backing).
void AppController::setInitialAlignmentMethod(int method) {
  if (initial_alignment_method_ == method) {
    return;
  }
  initial_alignment_method_ = method;
  emit initialAlignmentMethodChanged();
}

/// Update Mask Preview Dataset Index (QML property backing).
void AppController::setMaskPreviewDatasetIndex(int index) {
  if (session_.calibration_datasets.empty()) {
    return;
  }
  const int clamped = std::clamp(
      index, 0, static_cast<int>(session_.calibration_datasets.size()) - 1);
  if (mask_preview_dataset_index_ == clamped) {
    return;
  }
  mask_preview_dataset_index_ = clamped;
  emit maskPreviewDatasetIndexChanged();
  if (mask_editor_open_) {
    refreshRangeImage();
  }
}

/// AppController::setMaskEditorOpen implementation.
void AppController::setMaskEditorOpen(bool open) {
  if (mask_editor_open_ == open) {
    return;
  }
  mask_editor_open_ = open;
  if (open && !session_.calibration_datasets.empty()) {
    mask_preview_dataset_index_ = std::clamp(
        active_calibration_dataset_index_, 0,
        static_cast<int>(session_.calibration_datasets.size()) - 1);
    emit maskPreviewDatasetIndexChanged();
    refreshRangeImage();
  } else if (!open) {
    refreshRangeImage();
  }
  emit maskEditorOpenChanged();
  emit activeDatasetSelectionLockChanged();
}

/// AppController::activeDatasetIsSingleScan implementation.
bool AppController::activeDatasetIsSingleScan() const {
  return clc::dataset_is_single_scan(activeDatasetEntry());
}

/// AppController::addDatasetDefaultImagePath implementation.
QString AppController::addDatasetDefaultImagePath() const {
#ifdef CLC_DEV_ADD_DATASET_DEFAULTS
  return QStringLiteral(
      "/Users/rasmus/Downloads/pbb-muc-255-1-fasssauna-high-freq-approach/"
      "sequence1/2026-10-01T18_42_13_624Z.png");
#else
  return {};
#endif
}

/// AppController::addDatasetDefaultLidarPath implementation.
QString AppController::addDatasetDefaultLidarPath() const {
#ifdef CLC_DEV_ADD_DATASET_DEFAULTS
  return QStringLiteral(
      "/Users/rasmus/Downloads/pbb-muc-255-1-fasssauna-high-freq-approach/"
      "sequence1.pcd");
#else
  return {};
#endif
}

/// AppController::addDatasetDefaultIntrinsicsPath implementation.
QString AppController::addDatasetDefaultIntrinsicsPath() const {
#ifdef CLC_DEV_ADD_DATASET_DEFAULTS
  return QStringLiteral("/Users/rasmus/Documents/Projects/autocrane-core/"
                        "config/pbb/muc/shared/calibration/"
                        "cam_bridge_narrow_rotated_upright.yml");
#else
  return {};
#endif
}

/// AppController::pickAddDatasetImage implementation.
QString AppController::pickAddDatasetImage(const QString &initial_dir) {
  const QString start =
      initial_dir.isEmpty() ? addDatasetDefaultImagePath() : initial_dir;
  return QFileDialog::getOpenFileName(
      nullptr, QStringLiteral("Camera image for new dataset"), start,
      QStringLiteral("Images (*.png *.jpg *.jpeg *.bmp *.tif *.tiff)"));
}

/// AppController::pickAddDatasetLidarFile implementation.
QString AppController::pickAddDatasetLidarFile(const QString &initial_dir) {
  QString start = initial_dir;
  if (start.isEmpty()) {
    start = addDatasetDefaultLidarPath();
  }
  return pickPcdFileDialog(QStringLiteral("LiDAR scan file for new dataset"),
                           start);
}

/// AppController::pickAddDatasetLidarFolder implementation.
QString AppController::pickAddDatasetLidarFolder(const QString &initial_dir) {
  QString start = initial_dir;
  if (start.isEmpty()) {
    start = addDatasetDefaultLidarPath();
  }
  return pickPcdFolderDialog(
      QStringLiteral("LiDAR scan folder for new dataset"), start);
}

/// AppController::pickAddDatasetIntrinsics implementation.
QString AppController::pickAddDatasetIntrinsics(const QString &initial_dir) {
  const QString start =
      initial_dir.isEmpty() ? addDatasetDefaultIntrinsicsPath() : initial_dir;
  return QFileDialog::getOpenFileName(
      nullptr, QStringLiteral("Camera intrinsics for session"), start,
      QStringLiteral("YAML (*.yaml *.yml);;OpenCV (*.xml)"));
}

/// AppController::addCalibrationDatasetFromPaths implementation.
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

  if (!clc::session_has_calibration_datasets(session_)) {
    resetSessionStructuredScanState();
  }

  int ref_w = 0;
  int ref_h = 0;
  clc::session_structured_scan_reference(session_, ref_w, ref_h);
  const auto scan =
      clc::scan_pcd_source(lidar_path.toStdString(), ref_w, ref_h);
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
  if (scan.paths.size() > 1) {
    if (ref_w <= 0) {
      session_.reference_scan_width = scan.reference_width;
      session_.reference_scan_height = scan.reference_height;
    } else if (scan.reference_width != ref_w || scan.reference_height != ref_h) {
      setStatus(
          QStringLiteral("Scan geometry does not match session reference (%1×%2)")
              .arg(ref_w)
              .arg(ref_h));
      return false;
    }
  }

  clc::DatasetEntry time_probe;
  time_probe.image_path = QFileInfo(image_path).absoluteFilePath().toStdString();
  time_probe.pcd_paths = scan.paths;
  std::string image_time_err;
  if (!clc::validate_dataset_image_first_scan_time(time_probe, image_time_err)) {
    setStatus(QString::fromStdString(image_time_err));
    return false;
  }

  session_.calibration_datasets.erase(
      std::remove_if(session_.calibration_datasets.begin(),
                     session_.calibration_datasets.end(),
                     clc::calibration_dataset_is_placeholder),
      session_.calibration_datasets.end());
  session_.calibration_datasets.push_back(entry);
  ensureImageMaskStorage();
  image_mask_ops_by_dataset_.emplace_back();
  image_mask_pixels_by_dataset_.emplace_back();
  clc::clear_session_structured_scan_reference_if_no_multi_scan(session_);
  active_calibration_dataset_index_ =
      static_cast<int>(session_.calibration_datasets.size()) - 1;
  fusion_blocked_ = false;
  clearFusedCloudCache();
  emit calibrationDatasetsChanged();
  emit pcdCatalogChanged();
  emit sessionPathsChanged();
  refreshRangeImage();
  refreshActiveDatasetVisuals();
  setStatus(QStringLiteral("Added calibration dataset %1")
                .arg(QString::fromStdString(id)));
  logDatasetCatalogEntry(active_calibration_dataset_index_,
                         session_.calibration_datasets.back(),
                         QStringLiteral("added"));
  return true;
}

/// AppController::removeActiveCalibrationDataset implementation.
void AppController::removeActiveCalibrationDataset() {
  if (session_.calibration_datasets.empty()) {
    return;
  }
  logDatasetCatalogEntry(active_calibration_dataset_index_,
                         activeDatasetEntry(), QStringLiteral("removing"));
  session_.calibration_datasets.erase(session_.calibration_datasets.begin() +
                                      active_calibration_dataset_index_);
  if (active_calibration_dataset_index_ <
      static_cast<int>(image_mask_ops_by_dataset_.size())) {
    image_mask_ops_by_dataset_.erase(image_mask_ops_by_dataset_.begin() +
                                     active_calibration_dataset_index_);
  }
  if (active_calibration_dataset_index_ <
      static_cast<int>(image_mask_pixels_by_dataset_.size())) {
    image_mask_pixels_by_dataset_.erase(image_mask_pixels_by_dataset_.begin() +
                                        active_calibration_dataset_index_);
  }
  clc::clear_session_structured_scan_reference_if_no_multi_scan(session_);
  clearFusedCloudCache();
  pending_correspondence_3d_.reset();
  pending_correspondence_2d_.reset();
  selected_correspondence_row_ = -1;
  if (session_.calibration_datasets.empty()) {
    active_calibration_dataset_index_ = 0;
    empty_calibration_dataset_scratch_ = {};
    resetSessionStructuredScanState();
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
  mask_preview_dataset_index_ = std::clamp(
      mask_preview_dataset_index_, 0,
      static_cast<int>(session_.calibration_datasets.size()) - 1);
  emit maskPreviewDatasetIndexChanged();
  emit calibrationDatasetsChanged();
  emit pcdCatalogChanged();
  emit sessionPathsChanged();
  emit fusionParamsChanged();
  emit scanTimeChanged();
  syncActiveDatasetFusionToSession();
  refreshActiveDatasetVisuals();
  emit correspondencesChanged();
}

/// Copy Active Dataset Fusion To Session between session model and UI.
void AppController::syncActiveDatasetFusionToSession() {
  if (main_work_tab_ == 3 || session_.calibration_datasets.empty()) {
    return;
  }
  const auto &ds = activeDatasetEntry();
  session_.fusion_params = ds.fusion_params;
  session_.scan_time = ds.scan_time;
  session_.fusion_params.scan_time = ds.scan_time;
}

/// AppController::correspondenceTable implementation.
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
      row.insert(QStringLiteral("x"), pending_correspondence_3d_->x);
      row.insert(QStringLiteral("y"), pending_correspondence_3d_->y);
      row.insert(QStringLiteral("z"), pending_correspondence_3d_->z);
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

/// AppController::canComputeInitialAlignment implementation.
bool AppController::canComputeInitialAlignment() const {
  if (session_.camera.intrinsics.empty()) {
    return false;
  }
  return correspondenceCount() >= 3;
}

/// Update Selected Correspondence Row (QML property backing).
void AppController::setSelectedCorrespondenceRow(int global_row) {
  selected_correspondence_row_ = global_row;
  emit correspondencesChanged();
}

/// AppController::deleteCorrespondenceRow implementation.
void AppController::deleteCorrespondenceRow(int global_row) {
  if (main_work_tab_ != 1) {
    return;
  }
  if (global_row == -1) {
    pending_correspondence_2d_.reset();
    selected_correspondence_row_ = -1;
    emit correspondencesChanged();
    qCInfo(lcClcLabeling) << "pending image pick removed";
    setStatus(QStringLiteral("Removed pending image pick"));
    return;
  }
  if (global_row == -2) {
    pending_correspondence_3d_.reset();
    selected_correspondence_row_ = -1;
    emit correspondencesChanged();
    qCInfo(lcClcLabeling) << "pending 3D pick removed";
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
      qCInfo(lcClcLabeling).noquote()
          << QStringLiteral("correspondence deleted dataset=%1 local_row=%2 "
                            "remaining=%3")
                 .arg(QString::fromStdString(ds.id))
                 .arg(local)
                 .arg(correspondenceCount());
      setStatus(QStringLiteral("Correspondence removed (%1 remaining)")
                    .arg(correspondenceCount()));
      return;
    }
    cursor += count;
  }
  setStatus(QStringLiteral("Could not delete correspondence (row not found)"));
}

/// Recompute and push Camera Preview to the UI or viewer.
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

/// Update Intrinsics Path (QML property backing).
void AppController::setIntrinsicsPath(const QString &path) {
  const QString normalized = QFileInfo(path).absoluteFilePath();
  if (QString::fromStdString(session_.intrinsics_path) == normalized) {
    return;
  }
  session_.intrinsics_path = normalized.toStdString();
  emit sessionPathsChanged();
}

/// AppController::repopulateActiveDatasetPcdCatalogIfEmpty implementation.
bool AppController::repopulateActiveDatasetPcdCatalogIfEmpty() {
  auto &dataset = activeDatasetEntry();
  if (!dataset.pcd_paths.empty()) {
    return true;
  }
  if (dataset.cloud_directory.empty()) {
    return false;
  }

  int ref_w = 0;
  int ref_h = 0;
  clc::session_structured_scan_reference(session_, ref_w, ref_h);
  const auto scan = clc::scan_pcd_source(dataset.cloud_directory, ref_w, ref_h);
  if (!scan.error.empty()) {
    dataset.pcd_paths.clear();
    dataset.geometry_error = true;
    fusion_blocked_ = true;
    setStatus(QString::fromStdString(scan.error));
    emit pcdCatalogChanged();
    emit calibrationDatasetsChanged();
    return false;
  }

  dataset.pcd_paths = scan.paths;
  if (dataset.id.empty() && !dataset.pcd_paths.empty()) {
    dataset.id = clc::dataset_id_from_first_scan(dataset.pcd_paths);
  }
  std::string image_time_err;
  if (!clc::validate_dataset_image_first_scan_time(dataset, image_time_err)) {
    dataset.geometry_error = true;
    fusion_blocked_ = true;
    setStatus(QString::fromStdString(image_time_err));
  } else {
    dataset.geometry_error = false;
  }
  clc::sync_dataset_fusion_status(dataset);
  if (dataset.pcd_paths.size() > 1) {
    session_.reference_scan_width = scan.reference_width;
    session_.reference_scan_height = scan.reference_height;
  } else {
    clc::clear_session_structured_scan_reference_if_no_multi_scan(session_);
  }
  emit pcdCatalogChanged();
  emit calibrationDatasetsChanged();
  return !dataset.pcd_paths.empty();
}

/// Recompute and push Pcd Catalog to the UI or viewer.
void AppController::refreshPcdCatalog() {
  fusion_blocked_ = false;
  if (main_work_tab_ == 3) {
    session_.ensurePrimaryInspection();
  } else if (session_.calibration_datasets.empty()) {
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

  clearFusedCloudCache();
  setStatus(QStringLiteral("Found %1 PCD files (%2×%3)")
                .arg(static_cast<int>(dataset.pcd_paths.size()))
                .arg(session_.reference_scan_width)
                .arg(session_.reference_scan_height));
  qCInfo(lcClcDataset).noquote()
      << QStringLiteral("PCD catalog refreshed for active dataset[%1] id=%2: "
                        "%3 files (%4×%5)")
             .arg(main_work_tab_ == 3 ? 0 : active_calibration_dataset_index_)
             .arg(QString::fromStdString(dataset.id))
             .arg(static_cast<int>(dataset.pcd_paths.size()))
             .arg(session_.reference_scan_width)
             .arg(session_.reference_scan_height);
  refreshRangeImage();
  refreshPreviewInViewer();
}

/// Recompute and push Preview In Viewer to the UI or viewer.
void AppController::refreshPreviewInViewer() {
  if (fusedCloudBelongsToActiveContext()) {
    uploadFusedCloudToViewer();
  } else {
    uploadPreviewScanToViewer();
  }
}

/// AppController::maskWidth implementation.
int AppController::maskWidth() const { return maskRasterWidth(); }

/// AppController::maskHeight implementation.
int AppController::maskHeight() const { return maskRasterHeight(); }

/// AppController::maskPreview implementation.
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

/// Update Range Colormap Min (QML property backing).
void AppController::setRangeColormapMin(double v) {
  if (qFuzzyCompare(range_colormap_min_, v)) {
    return;
  }
  range_colormap_min_ = v;
  refreshRangeImage();
}

/// Update Range Colormap Max (QML property backing).
void AppController::setRangeColormapMax(double v) {
  if (qFuzzyCompare(range_colormap_max_, v)) {
    return;
  }
  range_colormap_max_ = v;
  refreshRangeImage();
}

/// Update Range Channel (QML property backing).
void AppController::setRangeChannel(int channel) {
  if (range_channel_ == channel) {
    return;
  }
  range_channel_ = channel;
  refreshRangeImage();
}

/// Update Range YStretch (QML property backing).
void AppController::setRangeYStretch(double stretch) {
  stretch = std::clamp(stretch, 1.0, 8.0);
  if (qFuzzyCompare(range_y_stretch_, stretch)) {
    return;
  }
  range_y_stretch_ = stretch;
  refreshRangeImage();
  emit rangeYStretchChanged();
}

/// Update Viewer Colormap Min (QML property backing).
void AppController::setViewerColormapMin(double v) {
  previewViewerColormapRange(v, viewer_colormap_max_);
}

/// Update Viewer Colormap Max (QML property backing).
void AppController::setViewerColormapMax(double v) {
  previewViewerColormapRange(viewer_colormap_min_, v);
}

/// AppController::applyViewerScalarRangeToItem implementation.
void AppController::applyViewerScalarRangeToItem() {
  if (auto *item = qobject_cast<psiviz::PointCloudItem *>(viewer_)) {
    item->setScalarMin(viewer_colormap_min_);
    item->setScalarMax(viewer_colormap_max_);
  }
}

/// AppController::previewViewerColormapRange implementation.
void AppController::previewViewerColormapRange(double min_v, double max_v) {
  if (max_v < min_v) {
    std::swap(min_v, max_v);
  }
  viewer_colormap_min_ = min_v;
  viewer_colormap_max_ = max_v;
  emit viewerColormapChanged();
  applyViewerScalarRangeToItem();
}

/// AppController::commitViewerColormapRange implementation.
void AppController::commitViewerColormapRange(double min_v, double max_v) {
  viewer_colormap_debounce_.stop();
  previewViewerColormapRange(min_v, max_v);
}

/// React to Viewer Colormap Debounce.
void AppController::onViewerColormapDebounce() {
  applyViewerScalarRangeToItem();
}

/// AppController::previewRangeColormapRange implementation.
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

/// AppController::commitRangeColormapRange implementation.
void AppController::commitRangeColormapRange(double min_v, double max_v) {
  range_colormap_debounce_.stop();
  previewRangeColormapRange(min_v, max_v);
}

/// React to Range Colormap Debounce.
void AppController::onRangeColormapDebounce() { refreshRangeImageDisplay(); }

/// Update Viewer Scalar Channel (QML property backing).
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

/// Update Fusion Voxel Resolution (QML property backing).
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

/// Update Fusion Num Threads (QML property backing).
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

/// Update Fusion Max Correspondence (QML property backing).
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

/// Update Fusion KNeighbors (QML property backing).
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

/// Update Fusion Backend (QML property backing).
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

/// Update Fusion Target Num Points (QML property backing).
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

/// Update Fusion Min Distance (QML property backing).
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

/// Update Fusion Integrator Ivox Resolution (QML property backing).
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

/// Update Fusion Integrator Ivox Min Points Dist (QML property backing).
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

/// Update Fusion Ivox Resolution (QML property backing).
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

/// Update Fusion Lm Max Iterations (QML property backing).
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

/// AppController::calibrationMaxOuterIterations implementation.
int AppController::calibrationMaxOuterIterations() const {
  return session_.calibration_params.max_outer_iterations;
}

/// AppController::calibrationMaxInnerIterations implementation.
int AppController::calibrationMaxInnerIterations() const {
  return session_.calibration_params.max_inner_iterations;
}

/// AppController::calibrationNumThreads implementation.
int AppController::calibrationNumThreads() const {
  return session_.calibration_params.num_threads;
}

/// AppController::calibrationDeltaTransThresh implementation.
double AppController::calibrationDeltaTransThresh() const {
  return session_.calibration_params.delta_trans_thresh;
}

/// AppController::calibrationDeltaRotDeg implementation.
double AppController::calibrationDeltaRotDeg() const {
  return session_.calibration_params.delta_rot_thresh * 180.0 / M_PI;
}

/// AppController::calibrationDisableZBufferCulling implementation.
bool AppController::calibrationDisableZBufferCulling() const {
  return session_.calibration_params.disable_z_buffer_culling;
}

/// AppController::calibrationNidBins implementation.
int AppController::calibrationNidBins() const {
  return session_.calibration_params.nid_bins;
}

/// AppController::calibrationRegistrationType implementation.
int AppController::calibrationRegistrationType() const {
  return session_.calibration_params.registration_type ==
                 vlcal::RegistrationType::NID_NELDER_MEAD
             ? 1
             : 0;
}

/// AppController::calibrationNelderMeadInitStep implementation.
double AppController::calibrationNelderMeadInitStep() const {
  return session_.calibration_params.nelder_mead_init_step;
}

/// AppController::calibrationNelderMeadConvergence implementation.
double AppController::calibrationNelderMeadConvergence() const {
  return session_.calibration_params.nelder_mead_convergence_criteria;
}

/// Update Calibration Max Outer Iterations (QML property backing).
void AppController::setCalibrationMaxOuterIterations(int v) {
  const int clamped = std::max(1, v);
  if (session_.calibration_params.max_outer_iterations == clamped) {
    return;
  }
  session_.calibration_params.max_outer_iterations = clamped;
  emit calibrationParamsChanged();
}

/// Update Calibration Max Inner Iterations (QML property backing).
void AppController::setCalibrationMaxInnerIterations(int v) {
  const int clamped = std::max(1, v);
  if (session_.calibration_params.max_inner_iterations == clamped) {
    return;
  }
  session_.calibration_params.max_inner_iterations = clamped;
  emit calibrationParamsChanged();
}

/// Update Calibration Num Threads (QML property backing).
void AppController::setCalibrationNumThreads(int v) {
  const int clamped = std::max(1, v);
  if (session_.calibration_params.num_threads == clamped) {
    return;
  }
  session_.calibration_params.num_threads = clamped;
  emit calibrationParamsChanged();
}

/// Update Calibration Delta Trans Thresh (QML property backing).
void AppController::setCalibrationDeltaTransThresh(double v) {
  if (qFuzzyCompare(session_.calibration_params.delta_trans_thresh, v)) {
    return;
  }
  session_.calibration_params.delta_trans_thresh = v;
  emit calibrationParamsChanged();
}

/// Update Calibration Delta Rot Deg (QML property backing).
void AppController::setCalibrationDeltaRotDeg(double v) {
  const double rad = v * M_PI / 180.0;
  if (qFuzzyCompare(session_.calibration_params.delta_rot_thresh, rad)) {
    return;
  }
  session_.calibration_params.delta_rot_thresh = rad;
  emit calibrationParamsChanged();
}

/// Update Calibration Disable ZBuffer Culling (QML property backing).
void AppController::setCalibrationDisableZBufferCulling(bool v) {
  if (session_.calibration_params.disable_z_buffer_culling == v) {
    return;
  }
  session_.calibration_params.disable_z_buffer_culling = v;
  emit calibrationParamsChanged();
}

/// Update Calibration Nid Bins (QML property backing).
void AppController::setCalibrationNidBins(int v) {
  const int clamped = std::max(2, v);
  if (session_.calibration_params.nid_bins == clamped) {
    return;
  }
  session_.calibration_params.nid_bins = clamped;
  emit calibrationParamsChanged();
}

/// Update Calibration Registration Type (QML property backing).
void AppController::setCalibrationRegistrationType(int v) {
  const auto reg = (v == 1) ? vlcal::RegistrationType::NID_NELDER_MEAD
                            : vlcal::RegistrationType::NID_BFGS;
  if (session_.calibration_params.registration_type == reg) {
    return;
  }
  session_.calibration_params.registration_type = reg;
  emit calibrationParamsChanged();
}

/// Update Calibration Nelder Mead Init Step (QML property backing).
void AppController::setCalibrationNelderMeadInitStep(double v) {
  if (qFuzzyCompare(session_.calibration_params.nelder_mead_init_step, v)) {
    return;
  }
  session_.calibration_params.nelder_mead_init_step = v;
  emit calibrationParamsChanged();
}

/// Update Calibration Nelder Mead Convergence (QML property backing).
void AppController::setCalibrationNelderMeadConvergence(double v) {
  if (qFuzzyCompare(
          session_.calibration_params.nelder_mead_convergence_criteria, v)) {
    return;
  }
  session_.calibration_params.nelder_mead_convergence_criteria = v;
  emit calibrationParamsChanged();
}

/// Recompute and push Viewer Colors to the UI or viewer.
void AppController::refreshViewerColors() { applyViewerScalarRangeToItem(); }

/// AppController::invalidateProjectionImageCache implementation.
void AppController::invalidateProjectionImageCache() {
  projection_image_.release();
  projection_image_path_.clear();
}

/// AppController::clearFusedCloudCache implementation.
void AppController::clearFusedCloudCache() {
  fused_cloud_ = {};
  fused_cloud_dataset_index_.reset();
  fused_cloud_is_inspection_ = false;
}

/// AppController::fusedCloudBelongsToActiveContext implementation.
bool AppController::fusedCloudBelongsToActiveContext() const {
  if (fused_cloud_.points.empty() || !fused_cloud_dataset_index_.has_value()) {
    return false;
  }
  if (main_work_tab_ == 3) {
    return fused_cloud_is_inspection_ && *fused_cloud_dataset_index_ == 0;
  }
  return !fused_cloud_is_inspection_ &&
         *fused_cloud_dataset_index_ == active_calibration_dataset_index_;
}

/// Create or validate Projection Image For Active Dataset before continuing.
bool AppController::ensureProjectionImageForActiveDataset() {
  if (main_work_tab_ == 3) {
    session_.ensurePrimaryInspection();
  } else if (session_.calibration_datasets.empty()) {
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

/// Reset Calibration Dataset Visuals to empty defaults.
void AppController::clearCalibrationDatasetVisuals() {
  invalidateProjectionImageCache();
  clearFusedCloudCache();
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
  refreshViewerAnnotationState();
}

/// AppController::resetSessionStructuredScanState implementation.
void AppController::resetSessionStructuredScanState() {
  clc::reset_session_structured_scan_reference(session_);
  fusion_blocked_ = false;
  mask_ops_.clear();
  pending_mask_polygon_ = {};
  setMaskPolygonPending(false);
  global_mask_pixels_.clear();
  emit maskChanged();
}

/// Recompute and push Active Dataset Visuals to the UI or viewer.
void AppController::refreshActiveDatasetVisuals() {
  if (main_work_tab_ == 3) {
    session_.ensurePrimaryInspection();
    const std::string image_path = activeDatasetEntry().image_path;
    if (projection_image_path_ != image_path) {
      invalidateProjectionImageCache();
    }
    refreshCameraPreview();
    loadActiveDatasetCloudToViewer();
    return;
  }
  if (session_.calibration_datasets.empty()) {
    clearCalibrationDatasetVisuals();
    return;
  }
  const std::string image_path = activeDatasetEntry().image_path;
  if (projection_image_path_ != image_path) {
    invalidateProjectionImageCache();
  }
  if (!fusedCloudBelongsToActiveContext()) {
    clearFusedCloudCache();
  }
  refreshCameraPreview();
  loadActiveDatasetCloudToViewer();
}

/// Copy Color Model From Session between session model and UI.
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
  std::vector<clc::Vec3d> points;
  std::vector<double> intensities;
  points.reserve(n);
  intensities.reserve(n);
  for (int i = 0; i < n; ++i) {
    points.push_back({viewer_positions_[i * 3], viewer_positions_[i * 3 + 1], viewer_positions_[i * 3 + 2]});
    intensities.push_back(i < viewer_scalars_.size()
                              ? static_cast<double>(viewer_scalars_[i])
                              : 0.0);
  }
  color_model_.setPoints(points, intensities);
}

/// AppController::reprojectViewerCameraRgb implementation.
void AppController::reprojectViewerCameraRgb() {
  auto *item = qobject_cast<psiviz::PointCloudItem *>(viewer_);
  if (!item || viewer_positions_.isEmpty()) {
    return;
  }
  syncColorModelFromSession();
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

/// Construct Alignment Input from current session state.
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
  // Seed NID from the last pose shown in the viewer: manual/gizmo alignment and
  // prior NID runs update T_camera_lidar; T_camera_lidar_initial is only the
  // correspondence PnP result and stays stale after reoptimization.
  input.init_T_camera_lidar = session_.T_camera_lidar;
  input.params = session_.calibration_params;

  auto load_fused_for_dataset =
      [&](int ds_index, const clc::DatasetEntry &ds,
          glim_cloud_fusion::DensePointCloud &out) -> bool {
    if (!ds.fused_cloud_path.empty()) {
      std::string err;
      if (clc::load_dense_pcd(ds.fused_cloud_path, out, err)) {
        return !out.points.empty();
      }
    }
    if (fused_cloud_dataset_index_.has_value() &&
        *fused_cloud_dataset_index_ == ds_index && !fused_cloud_.points.empty()) {
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

  int skipped_not_ready = 0;
  for (int ds_index = 0; ds_index < static_cast<int>(session_.calibration_datasets.size());
       ++ds_index) {
    const auto &ds = session_.calibration_datasets[static_cast<std::size_t>(ds_index)];
    if (!clc::dataset_ready(ds)) {
      ++skipped_not_ready;
      qCInfo(lcClcOptimization).noquote()
          << QStringLiteral("alignment input: skip dataset[%1] id=%2 (not ready)")
                 .arg(ds_index)
                 .arg(QString::fromStdString(ds.id));
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
    if (!load_fused_for_dataset(ds_index, ds, dense)) {
      error = "fused cloud missing for dataset: " + ds.id;
      return false;
    }
    std::vector<std::uint8_t> image_mask;
    if (!resolveImageMaskForAlignment(ds_index, ds, img.cols, img.rows, image_mask, error)) {
      return false;
    }
    input.images.push_back(img);
    input.image_masks.push_back(std::move(image_mask));
    input.clouds.push_back(clc::dense_cloud_to_nid_frame(dense));
    qCInfo(lcClcOptimization).noquote()
        << QStringLiteral(
               "alignment input: include dataset[%1] id=%2 image=%3 "
               "cloud_points=%4")
               .arg(ds_index)
               .arg(QString::fromStdString(ds.id))
               .arg(QString::fromStdString(ds.image_path))
               .arg(static_cast<qulonglong>(dense.points.size()));
  }
  if (input.images.empty()) {
    error = "no ready calibration datasets for alignment";
    return false;
  }
  logAlignmentOptimizationInput(input, skipped_not_ready);
  return true;
}

/// Build the dense cloud and RGB buffer currently shown in the 3D viewer for export.
bool AppController::buildViewerExportCloud(glim_cloud_fusion::DensePointCloud &cloud_out,
                                          std::vector<float> &rgb_out) const {
  const int point_count = viewer_positions_.size() / 3;
  if (point_count <= 0) {
    return false;
  }
  cloud_out.points.resize(static_cast<std::size_t>(point_count));
  cloud_out.intensities.resize(static_cast<std::size_t>(point_count));
  for (int i = 0; i < point_count; ++i) {
    const int base = i * 3;
    cloud_out.points[static_cast<std::size_t>(i)] =
        Eigen::Vector3d(viewer_positions_[base], viewer_positions_[base + 1],
                        viewer_positions_[base + 2]);
    cloud_out.intensities[static_cast<std::size_t>(i)] =
        i < viewer_scalars_.size() ? static_cast<double>(viewer_scalars_[i]) : 0.0;
  }
  rgb_out.resize(static_cast<std::size_t>(viewer_rgb_.size()));
  for (int i = 0; i < viewer_rgb_.size(); ++i) {
    rgb_out[static_cast<std::size_t>(i)] = viewer_rgb_[i];
  }
  return rgb_out.size() == cloud_out.points.size() * 3;
}

/// Save Fused Pcd to a user-chosen file path.
QString AppController::exportFusedPcd() {
  reprojectViewerCameraRgb();
  glim_cloud_fusion::DensePointCloud export_cloud;
  std::vector<float> rgb;
  if (!buildViewerExportCloud(export_cloud, rgb)) {
    if (fused_cloud_.points.empty()) {
      setStatus(QStringLiteral("No point cloud to export"));
      return {};
    }
    export_cloud = fused_cloud_;
    color_model_.projectCameraRgb(session_.T_camera_lidar, rgb);
  }
  const QString path =
      QFileDialog::getSaveFileName(nullptr, QStringLiteral("Export fused PCD"),
                                   QString(), QStringLiteral("PCD (*.pcd)"));
  if (path.isEmpty()) {
    return {};
  }
  std::string err;
  if (!clc::save_dense_pcd_rgb(path.toStdString(), export_cloud, rgb, err)) {
    setStatus(QString::fromStdString(err));
    return {};
  }
  setStatus(QStringLiteral("Exported %1").arg(path));
  return path;
}

/// Recompute and push Viewer Colormap Bounds to the UI or viewer.
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

/// Update Mask Polygon Pending (QML property backing).
void AppController::setMaskPolygonPending(bool pending) {
  if (mask_polygon_pending_ == pending) {
    return;
  }
  mask_polygon_pending_ = pending;
  emit maskPolygonPendingChanged();
  emit maskChanged();
}

/// AppController::pendingMaskPolygonVertices implementation.
QVariantList AppController::pendingMaskPolygonVertices() const {
  QVariantList out;
  if (!mask_polygon_pending_) {
    return out;
  }
  for (const auto &v : pending_mask_polygon_.vertices) {
    out.append(QVariant::fromValue(QPointF(v.x, v.y)));
  }
  return out;
}

/// AppController::pendingMaskPolygonNativePoints implementation.
std::vector<QPointF> AppController::pendingMaskPolygonNativePoints() const {
  std::vector<QPointF> out;
  if (!mask_polygon_pending_ || pending_mask_polygon_.vertices.size() < 3) {
    return out;
  }
  out.reserve(pending_mask_polygon_.vertices.size());
  for (const auto &v : pending_mask_polygon_.vertices) {
    out.emplace_back(v.x, v.y);
  }
  return out;
}

/// AppController::maskRasterWidth implementation.
int AppController::maskRasterWidth() const {
  if (session_.reference_scan_width > 0) {
    return session_.reference_scan_width;
  }
  return range_image_.width;
}

/// AppController::maskRasterHeight implementation.
int AppController::maskRasterHeight() const {
  if (session_.reference_scan_height > 0) {
    return session_.reference_scan_height;
  }
  return range_image_.height;
}

/// AppController::rebuildMaskFromOps implementation.
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

/// AppController::commitMaskPolygonNative implementation.
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

/// AppController::commitMaskPolygon implementation.
void AppController::commitMaskPolygon(const QVariantList &nativeVertices) {
  pending_mask_polygon_ = polygonFromVariantList(nativeVertices);
  if (pending_mask_polygon_.vertices.size() < 3) {
    return;
  }
  setMaskPolygonPending(true);
  setStatus(QStringLiteral("Polygon closed — choose Keep or Remove to apply"));
}

/// AppController::applyPendingMaskKeep implementation.
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

/// AppController::applyPendingMaskRemove implementation.
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
/// AppController::installSyntheticMaskEditorScan implementation.
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

/// Update Reference Scan Size For Test (QML property backing).
void AppController::setReferenceScanSizeForTest(int width, int height) {
  session_.reference_scan_width = width;
  session_.reference_scan_height = height;
  emit maskChanged();
}
#endif

/// AppController::discardPendingMaskPolygon implementation.
void AppController::discardPendingMaskPolygon() {
  if (!mask_polygon_pending_) {
    return;
  }
  pending_mask_polygon_ = {};
  setMaskPolygonPending(false);
  emit maskChanged();
  setStatus(QStringLiteral("Pending polygon discarded"));
}

/// Reset Mask to empty defaults.
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

/// AppController::saveMask implementation.
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

/// Load Mask from disk or session.
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

/// AppController::browseMaskFile implementation.
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

namespace {

QString sanitizedDatasetIdForFilename(const std::string& id) {
  QString safe = QString::fromStdString(id.empty() ? QStringLiteral("dataset").toStdString() : id);
  for (int i = 0; i < safe.size(); ++i) {
    const QChar c = safe[i];
    if (!c.isLetterOrNumber() && c != QLatin1Char('_') && c != QLatin1Char('-')) {
      safe[i] = QLatin1Char('_');
    }
  }
  return safe;
}

}  // namespace

void AppController::ensureImageMaskStorage() {
  const std::size_t n = session_.calibration_datasets.size();
  if (image_mask_ops_by_dataset_.size() < n) {
    image_mask_ops_by_dataset_.resize(n);
  }
  if (image_mask_pixels_by_dataset_.size() < n) {
    image_mask_pixels_by_dataset_.resize(n);
  }
}

QString AppController::imageMaskPath() const {
  if (session_.calibration_datasets.empty()) {
    return {};
  }
  return QString::fromStdString(activeDatasetEntry().image_mask.path);
}

int AppController::imageMaskRasterWidth() const {
  if (!activeDatasetEntry().image_mask.width) {
    const QImage img(imagePath());
    return img.width();
  }
  return activeDatasetEntry().image_mask.width;
}

int AppController::imageMaskRasterHeight() const {
  if (!activeDatasetEntry().image_mask.height) {
    const QImage img(imagePath());
    return img.height();
  }
  return activeDatasetEntry().image_mask.height;
}

int AppController::imageMaskWidth() const { return imageMaskRasterWidth(); }

int AppController::imageMaskHeight() const { return imageMaskRasterHeight(); }

QByteArray AppController::imageMaskPreview() const {
  const int w = imageMaskRasterWidth();
  const int h = imageMaskRasterHeight();
  if (w <= 0 || h <= 0 || session_.calibration_datasets.empty()) {
    return {};
  }
  const std::size_t n = static_cast<std::size_t>(w * h);
  std::vector<std::uint8_t> preview;
  if (active_calibration_dataset_index_ >= 0 &&
      static_cast<std::size_t>(active_calibration_dataset_index_) < image_mask_pixels_by_dataset_.size()) {
    const auto& pixels =
      image_mask_pixels_by_dataset_[static_cast<std::size_t>(active_calibration_dataset_index_)];
    if (pixels.size() == n) {
      preview = pixels;
    }
  }
  if (preview.empty()) {
    preview.assign(n, 255);
  }
  QByteArray out;
  out.resize(static_cast<int>(preview.size()));
  std::memcpy(out.data(), preview.data(), preview.size());
  return out;
}

void AppController::setImageMaskPolygonPending(bool pending) {
  if (image_mask_polygon_pending_ == pending) {
    return;
  }
  image_mask_polygon_pending_ = pending;
  emit imageMaskPolygonPendingChanged();
  emit imageMaskChanged();
}

void AppController::rebuildImageMaskFromOps(int dataset_index) {
  ensureImageMaskStorage();
  if (dataset_index < 0 ||
      dataset_index >= static_cast<int>(session_.calibration_datasets.size())) {
    return;
  }
  const int w = imageMaskRasterWidth();
  const int h = imageMaskRasterHeight();
  auto& ds = session_.calibration_datasets[static_cast<std::size_t>(dataset_index)];
  if (w <= 0 || h <= 0) {
    emit imageMaskChanged();
    return;
  }
  auto& ops = image_mask_ops_by_dataset_[static_cast<std::size_t>(dataset_index)];
  auto& pixels = image_mask_pixels_by_dataset_[static_cast<std::size_t>(dataset_index)];
  if (ops.empty()) {
    pixels.clear();
    emit imageMaskChanged();
    return;
  }
  pixels = clc::build_mask_from_ops(w, h, ops);
  ds.image_mask.width = w;
  ds.image_mask.height = h;
  emit imageMaskChanged();
}

void AppController::commitImageMaskPolygonNative(const std::vector<QPointF>& nativeVertices) {
  if (nativeVertices.size() < 3) {
    return;
  }
  pending_image_mask_polygon_ = {};
  for (const QPointF& p : nativeVertices) {
    pending_image_mask_polygon_.vertices.emplace_back(static_cast<float>(p.x()),
                                                      static_cast<float>(p.y()));
  }
  setImageMaskPolygonPending(true);
  setStatus(QStringLiteral("Image mask polygon closed — choose Keep or Remove"));
}

std::vector<QPointF> AppController::pendingImageMaskPolygonNativePoints() const {
  if (!image_mask_polygon_pending_) {
    return {};
  }
  std::vector<QPointF> out;
  for (const auto& v : pending_image_mask_polygon_.vertices) {
    out.emplace_back(v.x, v.y);
  }
  return out;
}

void AppController::applyPendingImageMaskKeep() {
  if (!image_mask_polygon_pending_) {
    return;
  }
  ensureImageMaskStorage();
  auto& ops = image_mask_ops_by_dataset_[static_cast<std::size_t>(active_calibration_dataset_index_)];
  ops.push_back({pending_image_mask_polygon_, true});
  pending_image_mask_polygon_ = {};
  setImageMaskPolygonPending(false);
  rebuildImageMaskFromOps(active_calibration_dataset_index_);
  setStatus(QStringLiteral("Image mask updated (keep region)"));
}

void AppController::applyPendingImageMaskRemove() {
  if (!image_mask_polygon_pending_) {
    return;
  }
  ensureImageMaskStorage();
  auto& ops = image_mask_ops_by_dataset_[static_cast<std::size_t>(active_calibration_dataset_index_)];
  ops.push_back({pending_image_mask_polygon_, false});
  pending_image_mask_polygon_ = {};
  setImageMaskPolygonPending(false);
  rebuildImageMaskFromOps(active_calibration_dataset_index_);
  setStatus(QStringLiteral("Image mask updated (remove region)"));
}

void AppController::discardPendingImageMaskPolygon() {
  if (!image_mask_polygon_pending_) {
    return;
  }
  pending_image_mask_polygon_ = {};
  setImageMaskPolygonPending(false);
  emit imageMaskChanged();
  setStatus(QStringLiteral("Pending image mask polygon discarded"));
}

void AppController::clearImageMask() {
  ensureImageMaskStorage();
  const std::size_t idx = static_cast<std::size_t>(active_calibration_dataset_index_);
  image_mask_ops_by_dataset_[idx].clear();
  image_mask_pixels_by_dataset_[idx].clear();
  pending_image_mask_polygon_ = {};
  setImageMaskPolygonPending(false);
  auto& ds = activeDatasetEntry();
  ds.image_mask.path.clear();
  ds.image_mask.width = 0;
  ds.image_mask.height = 0;
  emit imageMaskChanged();
  setStatus(QStringLiteral("Image optimization mask cleared"));
}

bool AppController::saveImageMask() {
  ensureImageMaskStorage();
  const int w = imageMaskRasterWidth();
  const int h = imageMaskRasterHeight();
  const std::size_t idx = static_cast<std::size_t>(active_calibration_dataset_index_);
  const auto& pixels = image_mask_pixels_by_dataset_[idx];
  if (w <= 0 || h <= 0 || pixels.size() != static_cast<std::size_t>(w * h)) {
    setStatus(QStringLiteral("No image mask to save (draw polygons or load a mask first)"));
    return false;
  }
  QString dir = session_directory_;
  if (dir.isEmpty()) {
    dir = QFileDialog::getExistingDirectory(nullptr, QStringLiteral("Save image mask to session folder"));
    if (dir.isEmpty()) {
      return false;
    }
    setSessionDirectory(dir);
  }
  const QString path =
    dir + QStringLiteral("/image_mask_") + sanitizedDatasetIdForFilename(activeDatasetEntry().id) +
    QStringLiteral(".png");
  if (!clc::save_mask_png(path.toStdString(), w, h, pixels)) {
    setStatus(QStringLiteral("Failed to write image mask"));
    return false;
  }
  auto& ds = activeDatasetEntry();
  ds.image_mask.path = path.toStdString();
  ds.image_mask.width = w;
  ds.image_mask.height = h;
  emit imageMaskChanged();
  setStatus(QStringLiteral("Image mask saved to %1").arg(path));
  return true;
}

bool AppController::loadImageMask() {
  const int w = imageMaskRasterWidth();
  const int h = imageMaskRasterHeight();
  if (w <= 0 || h <= 0) {
    setStatus(QStringLiteral("Load a calibration image before loading an image mask"));
    return false;
  }
  QString path = imageMaskPath();
  if (path.isEmpty()) {
    path = browseImageMaskFile();
    return !path.isEmpty();
  }
  ensureImageMaskStorage();
  std::string error;
  auto& pixels = image_mask_pixels_by_dataset_[static_cast<std::size_t>(active_calibration_dataset_index_)];
  if (!clc::load_mask_png(path.toStdString(), w, h, pixels, error)) {
    setStatus(QString::fromStdString(error));
    return false;
  }
  image_mask_ops_by_dataset_[static_cast<std::size_t>(active_calibration_dataset_index_)].clear();
  setImageMaskPolygonPending(false);
  auto& ds = activeDatasetEntry();
  ds.image_mask.path = path.toStdString();
  ds.image_mask.width = w;
  ds.image_mask.height = h;
  emit imageMaskChanged();
  setStatus(QStringLiteral("Image mask loaded"));
  return true;
}

QString AppController::browseImageMaskFile() {
  const QString file = QFileDialog::getOpenFileName(
    nullptr,
    QStringLiteral("Load image mask PNG"),
    session_directory_,
    QStringLiteral("PNG mask (*.png)"));
  if (file.isEmpty()) {
    return QString();
  }
  const int w = imageMaskRasterWidth();
  const int h = imageMaskRasterHeight();
  if (w <= 0 || h <= 0) {
    setStatus(QStringLiteral("Load a calibration image before loading an image mask"));
    return QString();
  }
  ensureImageMaskStorage();
  std::string error;
  auto& pixels = image_mask_pixels_by_dataset_[static_cast<std::size_t>(active_calibration_dataset_index_)];
  if (!clc::load_mask_png(file.toStdString(), w, h, pixels, error)) {
    setStatus(QString::fromStdString(error));
    return QString();
  }
  image_mask_ops_by_dataset_[static_cast<std::size_t>(active_calibration_dataset_index_)].clear();
  setImageMaskPolygonPending(false);
  auto& ds = activeDatasetEntry();
  ds.image_mask.path = file.toStdString();
  ds.image_mask.width = w;
  ds.image_mask.height = h;
  emit imageMaskChanged();
  setStatus(QStringLiteral("Image mask loaded"));
  return file;
}

void AppController::setImageMaskEditorOpen(bool open) {
  if (image_mask_editor_open_ == open) {
    return;
  }
  image_mask_editor_open_ = open;
  if (open) {
    refreshCameraPreview();
  }
  emit imageMaskEditorOpenChanged();
  emit activeDatasetSelectionLockChanged();
}

bool AppController::resolveImageMaskForAlignment(
  int dataset_index,
  const clc::DatasetEntry& ds,
  int image_cols,
  int image_rows,
  std::vector<std::uint8_t>& mask_out,
  std::string& error) {
  mask_out.clear();
  error.clear();
  const std::size_t expected =
    static_cast<std::size_t>(image_cols) * static_cast<std::size_t>(image_rows);
  if (expected == 0) {
    return true;
  }
  ensureImageMaskStorage();
  if (dataset_index >= 0 &&
      dataset_index < static_cast<int>(image_mask_pixels_by_dataset_.size())) {
    const auto& mem = image_mask_pixels_by_dataset_[static_cast<std::size_t>(dataset_index)];
    if (mem.size() == expected) {
      mask_out = mem;
      return true;
    }
  }
  if (ds.image_mask.path.empty()) {
    return true;
  }
  if (!std::filesystem::exists(ds.image_mask.path)) {
    error = "image mask file missing: " + ds.image_mask.path;
    return false;
  }
  std::string load_err;
  if (!clc::load_mask_png(ds.image_mask.path, image_cols, image_rows, mask_out, load_err)) {
    error = load_err;
    return false;
  }
  return true;
}

/// Load First Readable Scan from disk or session.
bool AppController::loadFirstReadableScan(clc::PointXYZITCloud &cloud,
                                          std::string &err) {
  if (main_work_tab_ == 3) {
    session_.ensurePrimaryInspection();
  }
  const auto &dataset = activeDatasetEntry();
  if (dataset.pcd_paths.empty()) {
    err = "no PCD files in catalog";
    return false;
  }
  const bool check_filename_stamp = dataset.pcd_paths.size() > 1;
  for (const auto &path : dataset.pcd_paths) {
    if (clc::load_pcd_xyzit(path, cloud, err, check_filename_stamp)) {
      return true;
    }
  }
  if (err.empty()) {
    err = "no readable PCD in catalog";
  }
  return false;
}

/// Load the first readable PCD from a calibration dataset catalog.
bool AppController::loadFirstReadableScanForDataset(
    int calibration_dataset_index, clc::PointXYZITCloud &cloud,
    std::string &err) {
  if (calibration_dataset_index < 0 ||
      calibration_dataset_index >=
          static_cast<int>(session_.calibration_datasets.size())) {
    err = "invalid calibration dataset index";
    return false;
  }
  const auto &dataset =
      session_.calibration_datasets[static_cast<std::size_t>(
          calibration_dataset_index)];
  const bool check_filename_stamp = dataset.pcd_paths.size() > 1;
  for (const auto &path : dataset.pcd_paths) {
    if (clc::load_pcd_xyzit(path, cloud, err, check_filename_stamp)) {
      return true;
    }
  }
  if (err.empty()) {
    err = "no readable PCD in catalog";
  }
  return false;
}

/// Recompute and push Range Image to the UI or viewer.
void AppController::refreshRangeImage() {
  if (session_.calibration_datasets.empty()) {
    refreshRangeImageForDataset(-1);
    return;
  }
  int index = active_calibration_dataset_index_;
  if (mask_editor_open_) {
    index = mask_preview_dataset_index_;
  }
  refreshRangeImageForDataset(index);
}

/// Build range_image_ from the first scan of the given calibration dataset.
void AppController::refreshRangeImageForDataset(int calibration_dataset_index) {
  const int previous_width = range_image_.width;
  const int previous_height = range_image_.height;
  range_image_ = {};
  range_image_display_ = {};
  if (calibration_dataset_index < 0 ||
      calibration_dataset_index >=
          static_cast<int>(session_.calibration_datasets.size()) ||
      fusion_blocked_) {
    emit rangeImageChanged();
    return;
  }
  const auto &dataset =
      session_.calibration_datasets[static_cast<std::size_t>(
          calibration_dataset_index)];
  if (dataset.pcd_paths.empty()) {
    emit rangeImageChanged();
    return;
  }
  clc::PointXYZITCloud cloud;
  std::string err;
  if (!loadFirstReadableScanForDataset(calibration_dataset_index, cloud, err)) {
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

/// Recompute and push Range Image Display to the UI or viewer.
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

/// AppController::applySessionToViewer implementation.
void AppController::applySessionToViewer() {
  auto *item = qobject_cast<psiviz::PointCloudItem *>(viewer_);
  if (!item) {
    return;
  }
  item->setPoseMatrix(cameraPoseInLidarFrameMatrix(session_.T_camera_lidar));
  item->setCameraAxisLength(1.0);
}

/// Recompute and push Viewer Gizmo State to the UI or viewer.
void AppController::refreshViewerAnnotationState() {
  auto *item = qobject_cast<psiviz::PointCloudItem *>(viewer_);
  if (!item) {
    return;
  }
  const bool align_tab = main_work_tab_ == 1 || main_work_tab_ == 2;
  const bool datasets_tab = main_work_tab_ == 0;
  const bool has_cloud = item->hasPointCloudContent();
  const bool dataset_loaded = !session_.calibration_datasets.empty() &&
                              active_calibration_dataset_index_ >= 0;
  const bool show_lidar_triad =
      main_work_tab_ != 3 && has_cloud && dataset_loaded;
  const bool has_extrinsic = clc::session_extrinsic_available(session_);
  const bool show_camera_triad = show_lidar_triad && has_extrinsic;

  item->setSceneOverlayTriads(show_camera_triad, show_lidar_triad);
  item->setGizmoEnabled(has_cloud);
  item->setGizmoManipulable(false);
  item->setPickingEnabled(pickingAllowed() && main_work_tab_ == 1);

  bool frustum_valid = false;
  double fx = 0.;
  double fy = 0.;
  double cx = 0.;
  double cy = 0.;
  double img_w = 0.;
  double img_h = 0.;
  if (has_extrinsic && (align_tab || datasets_tab) && has_cloud && dataset_loaded &&
      session_.camera.intrinsics.size() >= 4 && !projection_image_.empty()) {
    fx = session_.camera.intrinsics[0];
    fy = session_.camera.intrinsics[1];
    cx = session_.camera.intrinsics[2];
    cy = session_.camera.intrinsics[3];
    img_w = static_cast<double>(projection_image_.cols);
    img_h = static_cast<double>(projection_image_.rows);
    frustum_valid = fx > 0. && fy > 0. && img_w > 0. && img_h > 0.;
  }
  item->setCameraFrustumIntrinsics(frustum_valid, fx, fy, cx, cy, img_w, img_h, 2.0);
}

/// Load Active Dataset Cloud To Viewer from disk or session.
void AppController::loadActiveDatasetCloudToViewer() {
  if (main_work_tab_ != 3 && session_.calibration_datasets.empty()) {
    refreshPreviewInViewer();
    return;
  }
  if (main_work_tab_ == 3) {
    session_.ensurePrimaryInspection();
    auto &ds = activeDatasetEntry();
    if (fusedCloudBelongsToActiveContext() && ds.fusion_complete) {
      uploadFusedCloudToViewer();
      return;
    }
    if (!ds.fused_cloud_path.empty()) {
      glim_cloud_fusion::DensePointCloud dense;
      std::string err;
      if (clc::load_dense_pcd(ds.fused_cloud_path, dense, err)) {
        fused_cloud_ = dense;
        fused_cloud_dataset_index_ = 0;
        fused_cloud_is_inspection_ = true;
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
    refreshViewerAnnotationState();
    return;
  }
  auto &ds = activeDatasetEntry();
  const bool fused_cache_for_active =
      fusedCloudBelongsToActiveContext() && !fused_cloud_.points.empty();
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
      fused_cloud_is_inspection_ = false;
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
  refreshViewerAnnotationState();
}

/// AppController::emitSessionUiState implementation.
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

/// Load session.json from a directory and refresh all UI state.
bool AppController::loadSession(const QString &directory) {
  const QString dir = normalizedDirectoryPath(directory);
  if (dir.isEmpty()) {
    setStatus(QStringLiteral("Session directory is empty"));
    return false;
  }
  const QString path = dir + QStringLiteral("/session.json");
  clc::SessionData loaded;
  std::string load_error;
  if (!clc::load_session(path.toStdString(), loaded, load_error)) {
    const QString detail =
        load_error.empty() ? QString() : QStringLiteral(": %1").arg(QString::fromStdString(load_error));
    setStatus(QStringLiteral("Failed to load session from %1%2").arg(path, detail));
    qCWarning(lcClcDataset).noquote()
        << QStringLiteral("session load failed (%1): %2")
               .arg(path)
               .arg(QString::fromStdString(load_error));
    clearCalibrationDatasetVisuals();
    return false;
  }
  session_ = std::move(loaded);
  clc::prune_placeholder_calibration_datasets(session_);
  invalidateProjectionImageCache();
  clearFusedCloudCache();
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
    // Keep catalog from session.json when present; rescanning would pick up fused_*.pcd exports.
    if (activeDatasetEntry().pcd_paths.empty()) {
      refreshPcdCatalog();
    } else {
      fusion_blocked_ = false;
      emit pcdCatalogChanged();
      emit calibrationDatasetsChanged();
    }
  } else if (!activeDatasetEntry().pcd_paths.empty()) {
    emit pcdCatalogChanged();
  }
  if (!session_.intrinsics_path.empty()) {
    std::string error;
    if (!clc::load_camera_intrinsics_file(session_.intrinsics_path, session_.camera, error)) {
      qCWarning(lcClcDataset).noquote()
          << QStringLiteral("session intrinsics load failed (%1): %2")
                 .arg(QString::fromStdString(session_.intrinsics_path))
                 .arg(QString::fromStdString(error));
    }
  }
  mask_ops_.clear();
  pending_mask_polygon_ = {};
  setMaskPolygonPending(false);
  global_mask_pixels_.clear();
  if (!session_.global_mask.path.empty() && session_.global_mask.width > 0 &&
      session_.global_mask.height > 0) {
    std::string error;
    if (!clc::load_mask_png(session_.global_mask.path, session_.global_mask.width,
                            session_.global_mask.height, global_mask_pixels_, error)) {
      qCWarning(lcClcDataset).noquote()
          << QStringLiteral("session global mask load failed (%1): %2")
                 .arg(QString::fromStdString(session_.global_mask.path))
                 .arg(QString::fromStdString(error));
    }
  }
  refreshCameraPreview();
  setStatus(QStringLiteral("Session loaded"));
  qCInfo(lcClcDataset).noquote()
      << QStringLiteral("session loaded from %1: %2 calibration dataset(s)")
             .arg(path)
             .arg(static_cast<int>(session_.calibration_datasets.size()));
  for (int i = 0; i < static_cast<int>(session_.calibration_datasets.size()); ++i) {
    logDatasetCatalogEntry(
        i, session_.calibration_datasets[static_cast<std::size_t>(i)],
        QStringLiteral("loaded"));
  }
  emit maskChanged();
  image_mask_ops_by_dataset_.clear();
  image_mask_pixels_by_dataset_.clear();
  ensureImageMaskStorage();
  for (int i = 0; i < static_cast<int>(session_.calibration_datasets.size()); ++i) {
    const auto& ds = session_.calibration_datasets[static_cast<std::size_t>(i)];
    if (ds.image_mask.path.empty() || ds.image_mask.width <= 0 || ds.image_mask.height <= 0) {
      continue;
    }
    std::string err;
    clc::load_mask_png(
      ds.image_mask.path,
      ds.image_mask.width,
      ds.image_mask.height,
      image_mask_pixels_by_dataset_[static_cast<std::size_t>(i)],
      err);
  }
  emit imageMaskChanged();
  active_calibration_dataset_index_ = 0;
  emit calibrationDatasetsChanged();
  emitSessionUiState();
  emit correspondencesChanged();
  applySessionToViewer();
  refreshViewerAnnotationState();
  refreshActiveDatasetVisuals();
  return true;
}

/// Write session.json and related paths to a directory.
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

/// AppController::browseSessionDirectory implementation.
QString AppController::browseSessionDirectory() {
  const QString dir = QFileDialog::getExistingDirectory(
      nullptr, QStringLiteral("Session directory"), session_directory_);
  if (!dir.isEmpty()) {
    setSessionDirectory(dir);
  }
  return session_directory_;
}

/// AppController::browsePcdFile implementation.
QString AppController::browsePcdFile() {
  const QString path =
      pickPcdFileDialog(QStringLiteral("LiDAR scan file"), pcdDirectory());
  if (!path.isEmpty()) {
    setPcdDirectory(path);
  }
  return pcdDirectory();
}

/// AppController::browsePcdFolder implementation.
QString AppController::browsePcdFolder() {
  const QString path =
      pickPcdFolderDialog(QStringLiteral("LiDAR scan folder"), pcdDirectory());
  if (!path.isEmpty()) {
    setPcdDirectory(path);
  }
  return pcdDirectory();
}

/// AppController::browseImageFile implementation.
QString AppController::browseImageFile() {
  const QString file = QFileDialog::getOpenFileName(
      nullptr, QStringLiteral("Camera image"), imagePath(),
      QStringLiteral("Images (*.png *.jpg *.jpeg *.bmp *.tif *.tiff)"));
  if (!file.isEmpty()) {
    setImagePath(file);
  }
  return imagePath();
}

/// Load Intrinsics File from disk or session.
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
  qCInfo(lcClcDataset).noquote()
      << QStringLiteral("intrinsics loaded: path=%1 model=%2")
             .arg(normalized)
             .arg(QString::fromStdString(session_.camera.model));
  emit sessionPathsChanged();
  return true;
}

/// Attach the QML PointCloudItem and wire pick/pose signals.
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
          &AppController::refreshViewerAnnotationState);
  syncCorrespondenceMarkers();
  refreshViewerAnnotationState();
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

/// Recompute and push Preview Cloud From Mask to the UI or viewer.
void AppController::refreshPreviewCloudFromMask(bool preserve_view_state) {
  if (!fused_cloud_.points.empty()) {
    return;
  }
  uploadPreviewScanToViewer(preserve_view_state);
}

/// Send Lidar Scan To Viewer data to the point cloud viewer.
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

/// Send Preview Scan To Viewer data to the point cloud viewer.
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

/// Send Dense Cloud To Viewer data to the point cloud viewer.
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

/// Send Fused Cloud To Viewer data to the point cloud viewer.
void AppController::uploadFusedCloudToViewer() {
  uploadDenseCloudToViewer(fused_cloud_, false);
}

/// Save Inspection Colored Pcd to a user-chosen file path.
QString AppController::exportInspectionColoredPcd() {
  if (main_work_tab_ != 3) {
    setStatus(QStringLiteral(
        "Switch to the Inspection tab to export inspection clouds"));
    return {};
  }
  return exportFusedPcd();
}

/// Copy Correspondence Markers between session model and UI.
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
    item->setPendingSphereMarker((*pending_correspondence_3d_).x, (*pending_correspondence_3d_).y,
                                 (*pending_correspondence_3d_).z);
  } else {
    item->clearPendingSphereMarker();
  }
}

/// AppController::finalizePendingCorrespondenceIfReady implementation.
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
  c.x = pending_correspondence_3d_->x;
  c.y = pending_correspondence_3d_->y;
  c.z = pending_correspondence_3d_->z;
  ds.correspondences.push_back(c);
  pending_correspondence_3d_.reset();
  pending_correspondence_2d_.reset();
  selected_correspondence_row_ = -1;
  emit correspondencesChanged();
  qCInfo(lcClcLabeling).noquote()
      << QStringLiteral(
             "correspondence added dataset=%1 u=%2 v=%3 x=%4 y=%5 z=%6 "
             "(session total %7)")
             .arg(QString::fromStdString(ds.id))
             .arg(c.u, 0, 'f', 2)
             .arg(c.v, 0, 'f', 2)
             .arg(c.x, 0, 'g', 6)
             .arg(c.y, 0, 'g', 6)
             .arg(c.z, 0, 'g', 6)
             .arg(correspondenceCount());
  setStatus(QStringLiteral("Correspondence added (%1 total)")
                .arg(correspondenceCount()));
}

/// Handle a 3D pick from the viewer for correspondence pairing.
void AppController::onPointPicked(double x, double y, double z) {
  qCInfo(lcClcLabeling).noquote()
      << QStringLiteral("3D pick: x=%1 y=%2 z=%3")
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
        qCInfo(lcClcLabeling).noquote()
            << QStringLiteral(
                   "correspondence 3D updated dataset=%1 row=%2 x=%3 y=%4 z=%5")
                   .arg(QString::fromStdString(row_ds.id))
                   .arg(selected_correspondence_row_ - cursor)
                   .arg(x, 0, 'g', 6)
                   .arg(y, 0, 'g', 6)
                   .arg(z, 0, 'g', 6);
        setStatus(QStringLiteral("Updated 3D point for selected correspondence "
                                 "— Shift+click image to update pixel"));
        return;
      }
      cursor += count;
    }
  }
  pending_correspondence_3d_ = clc::Vec3d{x, y, z};
  if (pending_correspondence_2d_.has_value()) {
    finalizePendingCorrespondenceIfReady();
    return;
  }
  emit correspondencesChanged();
  setStatus(QStringLiteral("3D point picked — click the matching location "
                           "on the camera image"));
}

/// AppController::pickImageCorrespondence implementation.
void AppController::pickImageCorrespondence(double u, double v) {
  if (main_work_tab_ != 1 || !pickingAllowed()) {
    return;
  }
  qCInfo(lcClcLabeling).noquote()
      << QStringLiteral("image pick: u=%1 v=%2").arg(u, 0, 'f', 2).arg(v, 0, 'f', 2);
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
          c.x = (*pending_correspondence_3d_).x;
          c.y = (*pending_correspondence_3d_).y;
          c.z = (*pending_correspondence_3d_).z;
          pending_correspondence_3d_.reset();
        }
        pending_correspondence_2d_.reset();
        emit correspondencesChanged();
        qCInfo(lcClcLabeling).noquote()
            << QStringLiteral(
                   "correspondence image updated dataset=%1 row=%2 u=%3 v=%4")
                   .arg(QString::fromStdString(row_ds.id))
                   .arg(selected_correspondence_row_ - cursor)
                   .arg(u, 0, 'f', 2)
                   .arg(v, 0, 'f', 2);
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

/// Reset Correspondences to empty defaults.
void AppController::clearCorrespondences() {
  ensureActiveCalibrationDataset();
  const QString dataset_id = QString::fromStdString(activeDatasetEntry().id);
  activeDatasetEntry().correspondences.clear();
  pending_correspondence_3d_.reset();
  pending_correspondence_2d_.reset();
  selected_correspondence_row_ = -1;
  emit correspondencesChanged();
  qCInfo(lcClcLabeling).noquote()
      << QStringLiteral("correspondences cleared for dataset=%1").arg(dataset_id);
  setStatus(QStringLiteral("Correspondences cleared for active dataset"));
}

/// AppController::estimateInitialGuessFromCorrespondences implementation.
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
  qCInfo(lcClcLabeling).noquote()
      << QStringLiteral("initial guess from %1 correspondences across datasets")
             .arg(static_cast<int>(input.correspondences.size()));
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
  session_.extrinsic_available = true;
  applySessionToViewer();
  refreshViewerAnnotationState();
  reprojectViewerCameraRgb();
  setStatus(QString::fromStdString(result.message));
}

/// React to Viewer Pose Changed.
void AppController::onViewerPoseChanged() {
  auto *item = qobject_cast<psiviz::PointCloudItem *>(viewer_);
  if (!item) {
    return;
  }
  const QVariantList m = item->poseMatrix();
  if (m.size() != 16) {
    return;
  }
  clc::Mat4d T = clc::Mat4d::identity();
  int k = 0;
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      T(r, c) = m[k++].toDouble();
    }
  }
  session_.T_camera_lidar_initial = T;
  session_.T_camera_lidar = session_.T_camera_lidar_initial;
  session_.extrinsic_available = true;
  refreshViewerAnnotationState();
  reprojectViewerCameraRgb();
}

/// Run multi-scan fusion on a background thread for the active dataset.
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
  const auto &fusion_scan_time =
      main_work_tab_ == 3 ? session_.fusion_params.scan_time
                          : activeDatasetEntry().fusion_params.scan_time;
  if (!clc::dataset_is_single_scan(activeDatasetEntry()) &&
      !clc::scan_time_duration_valid(fusion_scan_time)) {
    setStatus(QStringLiteral(
        "Set scan duration (seconds) in Scan time before fusion"));
    return;
  }
  worker_cancel_requested_ = false;
  setWorkerRunning(true);
  setStatus(QStringLiteral("Fusion running..."));
  joinWorker();
  int ref_w = 0;
  int ref_h = 0;
  clc::session_structured_scan_reference(session_, ref_w, ref_h);
  syncActiveDatasetFusionToSession();
  const bool fusion_is_inspection = main_work_tab_ == 3;
  if (fusion_is_inspection) {
    session_.ensurePrimaryInspection();
  }
  const int fusion_dataset_index =
      fusion_is_inspection ? 0 : active_calibration_dataset_index_;
  const std::string fusion_dataset_id = activeDatasetEntry().id;
  const auto fusion_params = fusion_is_inspection ? session_.fusion_params
                                                  : activeDatasetEntry().fusion_params;
  const std::vector<std::uint8_t> mask_pixels = global_mask_pixels_;
  worker_ = std::make_unique<std::thread>([this, catalog, ref_w, ref_h,
                                           fusion_params, mask_pixels,
                                           fusion_dataset_index,
                                           fusion_dataset_id,
                                           fusion_is_inspection]() {
    auto params = fusion_params;
    params.scan_time = fusion_params.scan_time;
    const int total_scans = static_cast<int>(catalog.size());
    const bool check_filename_stamp = catalog.size() > 1;

    const auto is_cancelled = [this]() { return worker_cancel_requested_.load(); };

    std::vector<glim_cloud_fusion::LidarScan> scans;
    for (const auto &path : catalog) {
      if (is_cancelled()) {
        QMetaObject::invokeMethod(
            this,
            [this]() {
              setWorkerRunning(false);
              setStatus(QStringLiteral("Fusion cancelled"));
              emit fusionFinished(false, QStringLiteral("Fusion cancelled"));
            },
            Qt::QueuedConnection);
        return;
      }
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
        },
        is_cancelled);

    QMetaObject::invokeMethod(
        this,
        [this, result, fusion_dataset_index, fusion_dataset_id,
         fusion_is_inspection]() {
          setWorkerRunning(false);
          if (result.error == glim_cloud_fusion::FusionError::Cancelled) {
            setStatus(QStringLiteral("Fusion cancelled"));
            emit fusionFinished(false, QStringLiteral("Fusion cancelled"));
            return;
          }
          if (result.error != glim_cloud_fusion::FusionError::Ok) {
            setStatus(QString::fromStdString(result.message));
            emit fusionFinished(false, QString::fromStdString(result.message));
            return;
          }
          clc::DatasetEntry *target_ds = nullptr;
          if (fusion_is_inspection) {
            if (session_.inspection_datasets.empty()) {
              setStatus(QStringLiteral("Fusion finished but inspection dataset is gone"));
              emit fusionFinished(false,
                                  QStringLiteral("Target dataset no longer exists"));
              return;
            }
            target_ds = &session_.inspection_datasets.front();
          } else {
            if (fusion_dataset_index < 0 ||
                fusion_dataset_index >=
                    static_cast<int>(session_.calibration_datasets.size())) {
              setStatus(QStringLiteral("Fusion finished but target dataset is gone"));
              emit fusionFinished(false,
                                  QStringLiteral("Target dataset no longer exists"));
              return;
            }
            target_ds = &session_.calibration_datasets[static_cast<std::size_t>(
                fusion_dataset_index)];
          }
          fused_cloud_ = result.cloud;
          if (!result.scan_poses.empty()) {
            clc::transform_dense_cloud_from_world(
                fused_cloud_,
                clc::mat4dFromColumnMajor(result.scan_poses.front().matrix().data()));
          }
          fused_cloud_dataset_index_ = fusion_dataset_index;
          fused_cloud_is_inspection_ = fusion_is_inspection;
          auto &ds = *target_ds;
          ds.fusion_complete = true;
          qCInfo(lcClcDataset).noquote()
              << QStringLiteral("fusion complete dataset[%1] id=%2 points=%3 "
                                "inspection=%4")
                     .arg(fusion_dataset_index)
                     .arg(QString::fromStdString(fusion_dataset_id))
                     .arg(static_cast<qulonglong>(result.cloud.points.size()))
                     .arg(fusion_is_inspection);
          if (!session_directory_.isEmpty() && !fusion_dataset_id.empty()) {
            const QString prefix =
                fusion_is_inspection ? QStringLiteral("fused_inspection_")
                                     : QStringLiteral("fused_");
            const QString fused_path =
                session_directory_ + prefix +
                QString::fromStdString(fusion_dataset_id) +
                QStringLiteral(".pcd");
            std::string save_err;
            if (clc::save_dense_pcd(fused_path.toStdString(), fused_cloud_,
                                    save_err)) {
              ds.fused_cloud_path = fused_path.toStdString();
            }
          }
          setPointCount(static_cast<int>(result.cloud.points.size()));
          if (!fusion_is_inspection) {
            emit calibrationDatasetsChanged();
          }
          setStatus(QStringLiteral("Fusion complete"));
          uploadFusedCloudToViewer();
          refreshViewerAnnotationState();
          emit fusionFinished(true, QStringLiteral("Fusion complete"));
        },
        Qt::QueuedConnection);
  });
}

/// Run NID / visual-lidar calibration on a background thread.
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
  clc::AlignmentInput input;
  std::string build_err;
  if (!buildAlignmentInput(input, build_err)) {
    qCWarning(lcClcOptimization).noquote()
        << QStringLiteral("alignment input build failed: %1")
               .arg(QString::fromStdString(build_err));
    setStatus(QString::fromStdString(build_err));
    emit alignmentFinished(false, QString::fromStdString(build_err));
    return;
  }
  qCInfo(lcClcOptimization) << "starting NID alignment worker";
  worker_cancel_requested_ = false;
  setWorkerRunning(true);
  setStatus(QStringLiteral("Alignment running..."));
  session_.extrinsic_available = true;
  refreshViewerAnnotationState();
  joinWorker();
  worker_ = std::make_unique<std::thread>([this, input = std::move(input)]() {
    const auto is_cancelled = [this]() { return worker_cancel_requested_.load(); };

    const auto result = clc::run_alignment(
        input,
        [this](const vlcal::CalibrationProgress &p) {
          QMetaObject::invokeMethod(
              this,
              [this, p]() {
                session_.T_camera_lidar =
                    clc::mat4dFromColumnMajor(p.T_camera_lidar.matrix().data());
                applySessionToViewer();
                reprojectViewerCameraRgb();
                setStatus(QStringLiteral("cost=%1 outer=%2 inner=%3")
                              .arg(p.cost, 0, 'g', 6)
                              .arg(p.outer_iteration)
                              .arg(p.inner_iteration));
              },
              Qt::QueuedConnection);
        },
        is_cancelled);

    QMetaObject::invokeMethod(
        this,
        [this, result]() {
          setWorkerRunning(false);
          refreshViewerAnnotationState();
          if (result.error == vlcal::CalibrationError::Cancelled) {
            setStatus(QStringLiteral("Alignment cancelled"));
            emit alignmentFinished(false, QStringLiteral("Alignment cancelled"));
            return;
          }
          if (result.error != vlcal::CalibrationError::Ok) {
            setStatus(QString::fromStdString(result.message));
            emit alignmentFinished(false,
                                   QString::fromStdString(result.message));
            return;
          }
          session_.T_camera_lidar =
              clc::mat4dFromColumnMajor(result.T_camera_lidar.matrix().data());
          session_.extrinsic_available = true;
          applySessionToViewer();
          refreshViewerAnnotationState();
          reprojectViewerCameraRgb();
          setStatus(QStringLiteral("Alignment complete"));
          emit alignmentFinished(true, QStringLiteral("Alignment complete"));
        },
        Qt::QueuedConnection);
  });
}
