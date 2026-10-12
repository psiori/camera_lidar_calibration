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

#include <camera/create_camera.hpp>
#include <QFileDialog>
#include <QFileInfo>

#include <clc_core/calibration_providers.hpp>
#include <clc_core/calibration_service.hpp>
#include <clc_core/point_color_model.hpp>
#include <clc_core/camera_intrinsics_io.hpp>
#include <clc_core/colormap.hpp>
#include <clc_core/fusion_service.hpp>
#include <clc_core/pcd_io.hpp>
#include <clc_core/range_image.hpp>
#include <clc_core/scan_geometry.hpp>

#include <psiviz/point_cloud_item.hpp>

namespace {

glim_cloud_fusion::LidarScan to_fusion_scan(const clc::PointXYZITCloud& cloud) {
  glim_cloud_fusion::LidarScan scan;
  scan.width = cloud.width;
  scan.height = cloud.height;
  scan.points = cloud.scan.points;
  scan.times = cloud.scan.times;
  scan.intensities = cloud.scan.intensities;
  scan.keep = cloud.scan.keep;
  return scan;
}

vlcal::FrameCPU::Ptr to_frame_cpu(const glim_cloud_fusion::DensePointCloud& cloud) {
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> points;
  std::vector<double> intensities;
  points.reserve(cloud.points.size());
  intensities.reserve(cloud.intensities.size());
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    points.emplace_back(cloud.points[i].x(), cloud.points[i].y(), cloud.points[i].z(), 1.0);
    intensities.push_back(i < cloud.intensities.size() ? cloud.intensities[i] : 0.0);
  }
  auto frame = std::make_shared<vlcal::FrameCPU>(points);
  frame->add_intensities(intensities);
  return frame;
}

std::optional<QPointF> variantToPoint(const QVariant& value) {
  if (value.canConvert<QPointF>()) {
    return value.toPointF();
  }
  const QVariantMap map = value.toMap();
  if (map.contains(QLatin1String("x")) && map.contains(QLatin1String("y"))) {
    return QPointF(map.value(QLatin1String("x")).toDouble(), map.value(QLatin1String("y")).toDouble());
  }
  return std::nullopt;
}

clc::Polygon2f polygonFromVariantList(const QVariantList& native_vertices) {
  clc::Polygon2f polygon;
  for (const QVariant& v : native_vertices) {
    const auto point = variantToPoint(v);
    if (!point.has_value()) {
      continue;
    }
    polygon.vertices.emplace_back(static_cast<float>(point->x()), static_cast<float>(point->y()));
  }
  return polygon;
}

QVariantList isometryToPoseMatrix(const Eigen::Isometry3d& T) {
  QVariantList out;
  const Eigen::Matrix4d m = T.matrix();
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      out.append(m(r, c));
    }
  }
  return out;
}

QImage rangeImageToDisplay(const clc::RangeImage& image, clc::RangeImageChannel channel, float min_v, float max_v, double y_stretch) {
  if (image.width <= 0 || image.height <= 0) {
    return {};
  }
  const std::vector<float>& values = channel == clc::RangeImageChannel::Range ? image.range : image.intensity;
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
    const int out_h = std::max(1, static_cast<int>(std::lround(static_cast<double>(image.height) * y_stretch)));
    out = out.scaled(image.width, out_h, Qt::IgnoreAspectRatio, Qt::FastTransformation);
  }
  return out;
}

float autoMaxValue(const std::vector<float>& values) {
  float max_v = 0.f;
  for (float v : values) {
    if (std::isfinite(v)) {
      max_v = std::max(max_v, v);
    }
  }
  return max_v > 0.f ? max_v : 1.f;
}

QString normalizedDirectoryPath(const QString& path) {
  if (path.isEmpty()) {
    return path;
  }
  QFileInfo info(path);
  if (info.isDir()) {
    return info.absoluteFilePath();
  }
  return info.absolutePath();
}

QString normalizedPcdPath(const QString& path) {
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

}  // namespace

AppController::AppController(QObject* parent) : QObject(parent) {
  clc::apply_default_fusion_params(session_.fusion_params);
  clc::apply_default_scan_time_params(session_.scan_time);
  session_.fusion_params.scan_time = session_.scan_time;
  viewer_colormap_debounce_.setSingleShot(true);
  viewer_colormap_debounce_.setInterval(50);
  connect(&viewer_colormap_debounce_, &QTimer::timeout, this, &AppController::onViewerColormapDebounce);
  range_colormap_debounce_.setSingleShot(true);
  range_colormap_debounce_.setInterval(50);
  connect(&range_colormap_debounce_, &QTimer::timeout, this, &AppController::onRangeColormapDebounce);
}

AppController::~AppController() {
  joinWorker();
}

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
  }
}

double AppController::scanDuration() const {
  return session_.scan_time.scan_duration.value_or(0.0);
}

int AppController::timeOrigin() const {
  return static_cast<int>(session_.scan_time.time_origin);
}

int AppController::timeUnit() const {
  return static_cast<int>(session_.scan_time.time_unit);
}

void AppController::setScanDuration(double seconds) {
  session_.scan_time.scan_duration = seconds;
  session_.fusion_params.scan_time = session_.scan_time;
  emit scanTimeChanged();
}

void AppController::setTimeOrigin(int origin) {
  session_.scan_time.time_origin = static_cast<glim_cloud_fusion::TimeOrigin>(origin);
  session_.fusion_params.scan_time = session_.scan_time;
  emit scanTimeChanged();
}

void AppController::setTimeUnit(int unit) {
  session_.scan_time.time_unit = static_cast<glim_cloud_fusion::TimeUnit>(unit);
  session_.fusion_params.scan_time = session_.scan_time;
  emit scanTimeChanged();
}

void AppController::setStatus(const QString& message) {
  status_message_ = message;
  emit statusMessageChanged();
}

void AppController::setPointCount(int count) {
  point_count_ = count;
  emit pointCountChanged();
}

clc::DatasetEntry& AppController::activeDatasetEntry() {
  if (inspection_mode_) {
    session_.ensurePrimaryInspection();
    return session_.inspection_datasets.front();
  }
  session_.ensurePrimaryCalibration();
  return session_.calibration_datasets.front();
}

const clc::DatasetEntry& AppController::activeDatasetEntry() const {
  if (inspection_mode_) {
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
  return session_.calibration_datasets.front();
}

int AppController::pcdFileCount() const {
  return static_cast<int>(activeDatasetEntry().pcd_paths.size());
}

int AppController::correspondenceCount() const {
  if (inspection_mode_ || session_.calibration_datasets.empty()) {
    return 0;
  }
  return static_cast<int>(session_.calibration_datasets.front().correspondences.size());
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

void AppController::setSessionDirectory(const QString& directory) {
  const QString normalized = normalizedDirectoryPath(directory);
  if (session_directory_ == normalized) {
    return;
  }
  session_directory_ = normalized;
  emit sessionPathsChanged();
}

void AppController::setImagePath(const QString& path) {
  const QString normalized = QFileInfo(path).absoluteFilePath();
  if (imagePath() == normalized) {
    return;
  }
  activeDatasetEntry().image_path = normalized.toStdString();
  projection_image_.release();
  refreshCameraPreview();
  emit sessionPathsChanged();
}

void AppController::setPcdDirectory(const QString& path) {
  const QString normalized = normalizedPcdPath(path);
  if (pcdDirectory() == normalized) {
    return;
  }
  activeDatasetEntry().cloud_directory = normalized.toStdString();
  refreshPcdCatalog();
  emit sessionPathsChanged();
}

void AppController::setInspectionMode(bool inspection) {
  if (inspection_mode_ == inspection) {
    return;
  }
  inspection_mode_ = inspection;
  pending_correspondence_3d_.reset();
  emit inspectionModeChanged();
  emit correspondencesChanged();
  emit sessionPathsChanged();
  if (!activeDatasetEntry().cloud_directory.empty()) {
    refreshPcdCatalog();
  } else {
    refreshCameraPreview();
  }
  if (!fused_cloud_.points.empty()) {
    uploadFusedCloudToViewer();
    refreshViewerColors();
  } else {
    refreshPreviewInViewer();
  }
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

void AppController::setIntrinsicsPath(const QString& path) {
  const QString normalized = QFileInfo(path).absoluteFilePath();
  if (QString::fromStdString(session_.intrinsics_path) == normalized) {
    return;
  }
  session_.intrinsics_path = normalized.toStdString();
  emit sessionPathsChanged();
}

void AppController::refreshPcdCatalog() {
  fusion_blocked_ = false;
  auto& dataset = activeDatasetEntry();
  if (dataset.cloud_directory.empty()) {
    dataset.pcd_paths.clear();
    emit pcdCatalogChanged();
    return;
  }

  const auto scan = clc::scan_pcd_source(
    dataset.cloud_directory, session_.reference_scan_width, session_.reference_scan_height);
  if (!scan.error.empty()) {
    dataset.pcd_paths.clear();
    fusion_blocked_ = true;
    setStatus(QString::fromStdString(scan.error));
    emit pcdCatalogChanged();
    return;
  }

  dataset.pcd_paths = scan.paths;
  session_.reference_scan_width = scan.reference_width;
  session_.reference_scan_height = scan.reference_height;
  fused_cloud_ = {};
  setStatus(QStringLiteral("Found %1 PCD files (%2×%3)")
              .arg(static_cast<int>(dataset.pcd_paths.size()))
              .arg(session_.reference_scan_width)
              .arg(session_.reference_scan_height));
  emit pcdCatalogChanged();
  refreshRangeImage();
  refreshPreviewInViewer();
}

void AppController::refreshPreviewInViewer() {
  if (fused_cloud_.points.empty()) {
    uploadPreviewScanToViewer();
  }
}

int AppController::maskWidth() const {
  return maskRasterWidth();
}

int AppController::maskHeight() const {
  return maskRasterHeight();
}

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
  return QByteArray(reinterpret_cast<const char*>(preview.data()), static_cast<int>(preview.size()));
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
  stretch = std::clamp(stretch, 0.25, 8.0);
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
  if (auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_)) {
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
  auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_);
  if (!item || item->blendWeight() <= 0.0) {
    return;
  }
  viewer_colormap_debounce_.start();
}

void AppController::commitViewerColormapRange(double min_v, double max_v) {
  viewer_colormap_debounce_.stop();
  previewViewerColormapRange(min_v, max_v);
  if (auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_)) {
    if (item->blendWeight() > 0.0) {
      refreshViewerRgb(static_cast<float>(item->blendWeight()));
    }
  }
}

void AppController::onViewerColormapDebounce() {
  if (auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_)) {
    refreshViewerRgb(static_cast<float>(item->blendWeight()));
  }
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

void AppController::onRangeColormapDebounce() {
  refreshRangeImageDisplay();
}

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
  if (qFuzzyCompare(session_.fusion_params.voxel_resolution, v)) {
    return;
  }
  session_.fusion_params.voxel_resolution = v;
  emit fusionParamsChanged();
}

void AppController::setFusionNumThreads(int n) {
  if (session_.fusion_params.num_threads == n) {
    return;
  }
  session_.fusion_params.num_threads = std::max(1, n);
  emit fusionParamsChanged();
}

void AppController::setFusionMaxCorrespondence(double v) {
  if (qFuzzyCompare(session_.fusion_params.max_correspondence_distance, v)) {
    return;
  }
  session_.fusion_params.max_correspondence_distance = v;
  emit fusionParamsChanged();
}

void AppController::refreshViewerColors() {
  auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_);
  if (!item) {
    return;
  }
  refreshViewerRgb(static_cast<float>(item->blendWeight()));
}

void AppController::syncColorModelFromSession() {
  const std::string image_path = activeDatasetEntry().image_path;
  if (session_.camera.intrinsics.empty() || image_path.empty() || viewer_positions_.isEmpty()) {
    return;
  }
  if (projection_image_.empty()) {
    projection_image_ = cv::imread(image_path, cv::IMREAD_COLOR);
  }
  if (projection_image_.empty()) {
    return;
  }
  const auto camera =
    camera::create_camera(session_.camera.model, session_.camera.intrinsics, session_.camera.distortion_coeffs);
  color_model_.setCamera(camera);
  color_model_.setImage(projection_image_);

  const int n = viewer_positions_.size() / 3;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> points;
  std::vector<double> intensities;
  points.reserve(n);
  intensities.reserve(n);
  for (int i = 0; i < n; ++i) {
    points.emplace_back(viewer_positions_[i * 3], viewer_positions_[i * 3 + 1], viewer_positions_[i * 3 + 2]);
    intensities.push_back(i < viewer_scalars_.size() ? static_cast<double>(viewer_scalars_[i]) : 0.0);
  }
  color_model_.setPoints(points, intensities);
}

void AppController::refreshViewerRgb(float blend_weight) {
  auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_);
  if (!item || viewer_positions_.isEmpty()) {
    return;
  }
  if (!color_model_.ready()) {
    syncColorModelFromSession();
  }
  if (!color_model_.ready()) {
    viewer_rgb_.fill(0.5f, viewer_positions_.size() * 3);
  } else {
    std::vector<float> rgb;
    color_model_.projectColors(session_.T_camera_lidar, blend_weight, static_cast<float>(viewer_colormap_min_),
                               static_cast<float>(viewer_colormap_max_), rgb);
    viewer_rgb_ = QVector<float>(static_cast<int>(rgb.size()));
    if (!rgb.empty()) {
      std::memcpy(viewer_rgb_.data(), rgb.data(), rgb.size() * sizeof(float));
    }
  }
  item->setScalarMin(viewer_colormap_min_);
  item->setScalarMax(viewer_colormap_max_);
  const int point_count = viewer_positions_.size() / 3;
  const bool same_geometry =
    item->hasPointCloudContent() && item->pointCloudPointCount() == static_cast<std::uint64_t>(point_count);
  if (same_geometry) {
    item->updatePointCloudScalarsAndColors(viewer_scalars_, viewer_rgb_);
  } else {
    item->setPointCloudData(viewer_positions_, viewer_scalars_, viewer_rgb_);
  }
}

bool AppController::buildAlignmentInput(clc::AlignmentInput& input, std::string& error) {
  static_assert(clc::OptimizerTraits<clc::NidOptimizerProvider>::supports_multi_dataset);
  if (session_.camera.intrinsics.empty()) {
    error = "camera intrinsics required";
    return false;
  }
  if (fused_cloud_.points.empty()) {
    error = "fused cloud required";
    return false;
  }
  const std::string image_path = activeDatasetEntry().image_path;
  projection_image_ = cv::imread(image_path, cv::IMREAD_COLOR);
  if (projection_image_.empty()) {
    error = "failed to load image";
    return false;
  }
  input.camera = camera::create_camera(session_.camera.model, session_.camera.intrinsics, session_.camera.distortion_coeffs);
  input.init_T_camera_lidar = session_.T_camera_lidar_initial;
  input.params = session_.calibration_params;

  session_.ensurePrimaryCalibration();
  if (session_.calibration_datasets.size() > 1 &&
      clc::InitialGuessTraits<clc::ManualGuessProvider>::supports_multi_dataset) {
    for (const auto& ds : session_.calibration_datasets) {
      if (ds.image_path.empty()) {
        continue;
      }
      cv::Mat img = cv::imread(ds.image_path, cv::IMREAD_COLOR);
      if (img.empty()) {
        error = "failed to load dataset image: " + ds.image_path;
        return false;
      }
      input.images.push_back(img);
      input.clouds.push_back(to_frame_cpu(fused_cloud_));
    }
  }
  if (input.images.empty()) {
    input.images = {projection_image_};
    input.clouds = {to_frame_cpu(fused_cloud_)};
  }
  return true;
}

QString AppController::exportFusedPcd() {
  if (fused_cloud_.points.empty()) {
    setStatus(QStringLiteral("No fused cloud to export"));
    return {};
  }
  const QString path =
    QFileDialog::getSaveFileName(nullptr, QStringLiteral("Export fused PCD"), QString(), QStringLiteral("PCD (*.pcd)"));
  if (path.isEmpty()) {
    return {};
  }
  refreshViewerColors();
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
        s = i < static_cast<int>(fused_cloud_.intensities.size()) ? static_cast<float>(fused_cloud_.intensities[i]) : 0.f;
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
  for (const auto& v : pending_mask_polygon_.vertices) {
    out.append(QVariant::fromValue(QPointF(v.x(), v.y())));
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

void AppController::commitMaskPolygonNative(const std::vector<QPointF>& nativeVertices) {
  if (nativeVertices.size() < 3) {
    return;
  }
  pending_mask_polygon_ = {};
  for (const QPointF& p : nativeVertices) {
    pending_mask_polygon_.vertices.emplace_back(static_cast<float>(p.x()), static_cast<float>(p.y()));
  }
  mask_polygon_pending_ = true;
  emit maskPolygonPendingChanged();
  emit maskChanged();
  setStatus(QStringLiteral("Polygon closed — choose Keep or Remove to apply"));
}

void AppController::commitMaskPolygon(const QVariantList& nativeVertices) {
  pending_mask_polygon_ = polygonFromVariantList(nativeVertices);
  if (pending_mask_polygon_.vertices.size() < 3) {
    return;
  }
  mask_polygon_pending_ = true;
  emit maskPolygonPendingChanged();
  emit maskChanged();
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
  setStatus(QStringLiteral("Mask updated (keep region, %1 ops)").arg(static_cast<int>(mask_ops_.size())));
}

void AppController::applyPendingMaskRemove() {
  if (!mask_polygon_pending_) {
    return;
  }
  mask_ops_.push_back({pending_mask_polygon_, false});
  pending_mask_polygon_ = {};
  setMaskPolygonPending(false);
  rebuildMaskFromOps();
  setStatus(QStringLiteral("Mask updated (remove region, %1 ops)").arg(static_cast<int>(mask_ops_.size())));
}

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
  if (w <= 0 || h <= 0 || global_mask_pixels_.size() != static_cast<std::size_t>(w * h)) {
    setStatus(QStringLiteral("No mask to save (draw polygons or load a mask first)"));
    return false;
  }
  QString dir = session_directory_;
  if (dir.isEmpty()) {
    dir = QFileDialog::getExistingDirectory(nullptr, QStringLiteral("Save mask to session folder"));
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
  if (!clc::load_mask_png(path.toStdString(), w, h, global_mask_pixels_, error)) {
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
    nullptr,
    QStringLiteral("Load mask PNG"),
    session_directory_,
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
  if (!clc::load_mask_png(file.toStdString(), w, h, global_mask_pixels_, error)) {
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

void AppController::maybeInferScanDurationFromScan(const clc::PointXYZITCloud& cloud) {
  if (session_.scan_time.scan_duration.has_value() && *session_.scan_time.scan_duration > 0.0) {
    return;
  }
  if (session_.scan_time.time_origin != glim_cloud_fusion::TimeOrigin::ScanStart) {
    return;
  }
  const auto duration = clc::infer_scan_duration_seconds(cloud.scan, session_.scan_time);
  if (!duration.has_value()) {
    return;
  }
  session_.scan_time.scan_duration = *duration;
  session_.fusion_params.scan_time = session_.scan_time;
  emit scanTimeChanged();
}

bool AppController::loadFirstReadableScan(clc::PointXYZITCloud& cloud, std::string& err) {
  for (const auto& path : activeDatasetEntry().pcd_paths) {
    if (clc::load_pcd_xyzit(path, cloud, err)) {
      maybeInferScanDurationFromScan(cloud);
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
      (range_image_.width != previous_width || range_image_.height != previous_height)) {
    discardPendingMaskPolygon();
  }
  const auto channel = range_channel_ == 0 ? clc::RangeImageChannel::Range : clc::RangeImageChannel::Intensity;
  const std::vector<float>& values = channel == clc::RangeImageChannel::Range ? range_image_.range : range_image_.intensity;
  if (!values.empty()) {
    const auto [min_it, max_it] = std::minmax_element(values.begin(), values.end());
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
  const auto channel = range_channel_ == 0 ? clc::RangeImageChannel::Range : clc::RangeImageChannel::Intensity;
  range_image_display_ = rangeImageToDisplay(range_image_, channel, static_cast<float>(range_colormap_min_),
                                             static_cast<float>(range_colormap_max_), range_y_stretch_);
  emit rangeImageChanged();
}

void AppController::applySessionToViewer() {
  auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_);
  if (!item) {
    return;
  }
  item->setPoseMatrix(isometryToPoseMatrix(session_.T_camera_lidar_initial));
}

void AppController::emitSessionUiState() {
  emit scanTimeChanged();
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

bool AppController::loadSession(const QString& directory) {
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
  session_.fusion_params.scan_time = session_.scan_time;
  setSessionDirectory(dir);
  if (!activeDatasetEntry().cloud_directory.empty()) {
    refreshPcdCatalog();
  } else if (!activeDatasetEntry().pcd_paths.empty()) {
    emit pcdCatalogChanged();
  }
  if (!session_.intrinsics_path.empty()) {
    std::string error;
    clc::load_camera_intrinsics_file(session_.intrinsics_path, session_.camera, error);
  }
  mask_ops_.clear();
  pending_mask_polygon_ = {};
  setMaskPolygonPending(false);
  global_mask_pixels_.clear();
  if (!session_.global_mask.path.empty() && session_.global_mask.width > 0 && session_.global_mask.height > 0) {
    std::string error;
    clc::load_mask_png(session_.global_mask.path, session_.global_mask.width, session_.global_mask.height,
                       global_mask_pixels_, error);
  }
  refreshCameraPreview();
  setStatus(QStringLiteral("Session loaded"));
  emit maskChanged();
  emitSessionUiState();
  emit correspondencesChanged();
  return true;
}

bool AppController::saveSession(const QString& directory) {
  const QString dir = normalizedDirectoryPath(directory.isEmpty() ? session_directory_ : directory);
  if (dir.isEmpty()) {
    setStatus(QStringLiteral("Session directory is empty"));
    return false;
  }
  setSessionDirectory(dir);
  session_.fusion_params.scan_time = session_.scan_time;
  if (!global_mask_pixels_.empty() && session_.reference_scan_width > 0 && session_.reference_scan_height > 0 &&
      global_mask_pixels_.size() == static_cast<std::size_t>(session_.reference_scan_width * session_.reference_scan_height)) {
    const QString mask_path = dir + QStringLiteral("/global_mask.png");
    clc::save_mask_png(mask_path.toStdString(), session_.reference_scan_width, session_.reference_scan_height,
                       global_mask_pixels_);
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
  const QString dir = QFileDialog::getExistingDirectory(nullptr, QStringLiteral("Session directory"), session_directory_);
  if (!dir.isEmpty()) {
    setSessionDirectory(dir);
  }
  return session_directory_;
}

QString AppController::browsePcdDirectory() {
  QString path = QFileDialog::getExistingDirectory(nullptr, QStringLiteral("PCD directory"), pcdDirectory());
  if (path.isEmpty()) {
    path = QFileDialog::getOpenFileName(
      nullptr, QStringLiteral("PCD file"), pcdDirectory(), QStringLiteral("PCD files (*.pcd);;All files (*)"));
  }
  if (!path.isEmpty()) {
    setPcdDirectory(path);
  }
  return pcdDirectory();
}

QString AppController::browseImageFile() {
  const QString file = QFileDialog::getOpenFileName(
    nullptr,
    QStringLiteral("Camera image"),
    "/Users/rasmus/Downloads/pbb-muc-255-1-fasssauna-high-freq-approach/sequence1/2026-10-01T18_42_13_624Z.png",//imagePath(),
    QStringLiteral("Images (*.png *.jpg *.jpeg *.bmp *.tif *.tiff)"));
  if (!file.isEmpty()) {
    setImagePath(file);
  }
  return imagePath();
}

QString AppController::browseIntrinsicsFile() {
  const QString file = QFileDialog::getOpenFileName(
    nullptr,
    QStringLiteral("Camera intrinsics"),
    "/Users/rasmus/Documents/Projects/autocrane-core/config/pbb/muc/shared/calibration/cam_bridge_narrow_rotated_upright.yml",//intrinsicsPath(),
    QStringLiteral("YAML (*.yaml *.yml);;OpenCV (*.xml)"));
  if (!file.isEmpty()) {
    loadIntrinsicsFile(file);
  }
  return intrinsicsPath();
}

bool AppController::loadIntrinsicsFile(const QString& path) {
  const QString normalized = QFileInfo(path).absoluteFilePath();
  if (normalized.isEmpty()) {
    setStatus(QStringLiteral("Intrinsics path is empty"));
    return false;
  }
  std::string error;
  if (!clc::load_camera_intrinsics_file(normalized.toStdString(), session_.camera, error)) {
    setStatus(QString::fromStdString(error));
    return false;
  }
  session_.intrinsics_path = normalized.toStdString();
  setStatus(QStringLiteral("Intrinsics loaded (%1)").arg(QString::fromStdString(session_.camera.model)));
  emit sessionPathsChanged();
  return true;
}

void AppController::setViewer(QObject* viewer) {
  if (viewer_ == viewer) {
    return;
  }
  if (viewer_) {
    disconnect(viewer_, nullptr, this, nullptr);
  }
  viewer_ = viewer;
  auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_);
  if (!item) {
    setStatus(QStringLiteral("3D viewer type mismatch; point cloud preview disabled"));
    return;
  }
  connect(item, &psiviz::PointCloudItem::pointPicked, this, &AppController::onPointPicked);
  connect(item, &psiviz::PointCloudItem::poseChanged, this, &AppController::onViewerPoseChanged);
  connect(item, &psiviz::PointCloudItem::blendWeightChanged, this, &AppController::refreshViewerColors);
  connect(this, &AppController::correspondencesChanged, this, &AppController::syncCorrespondenceMarkers);
  syncCorrespondenceMarkers();
  if (!fused_cloud_.points.empty()) {
    uploadFusedCloudToViewer();
  } else if (preview_upload_pending_ || !activeDatasetEntry().pcd_paths.empty()) {
    uploadPreviewScanToViewer();
  }
}

void AppController::refreshPreviewCloudFromMask(bool preserve_view_state) {
  if (!fused_cloud_.points.empty()) {
    return;
  }
  uploadPreviewScanToViewer(preserve_view_state);
}

void AppController::uploadLidarScanToViewer(const clc::PointXYZITCloud& cloud, bool preserve_view_state) {
  auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_);
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
    const auto& p = cloud.scan.points[i];
    if (!cloud.scan.keep.empty() && i < static_cast<int>(cloud.scan.keep.size()) && !cloud.scan.keep[i]) {
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
        scalar = i < static_cast<int>(cloud.scan.intensities.size()) ? static_cast<float>(cloud.scan.intensities[i]) : 0.f;
        break;
    }
    scalars << scalar;
    rgb << 0.5f << 0.5f << 0.5f;
    min_v = std::min(min_v, scalar);
    max_v = std::max(max_v, scalar);
  }
  if (positions.isEmpty()) {
    setPointCount(0);
    setStatus(QStringLiteral("No finite points to show in 3D preview (check mask / PCD)"));
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
  refreshViewerRgb(static_cast<float>(item->blendWeight()));
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
    setStatus(QString::fromStdString(err.empty() ? "failed to load preview scan" : err));
    setPointCount(0);
    return;
  }
  const std::size_t expected_mask = static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height);
  if (!global_mask_pixels_.empty() && global_mask_pixels_.size() == expected_mask) {
    auto scan = to_fusion_scan(cloud);
    clc::apply_mask_to_scan(scan, global_mask_pixels_);
    cloud.scan = std::move(scan);
  }
  uploadLidarScanToViewer(cloud, preserve_view_state);
}

void AppController::uploadDenseCloudToViewer(const glim_cloud_fusion::DensePointCloud& cloud, bool preserve_view_state) {
  auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_);
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
    const auto& p = cloud.points[i];
    positions << static_cast<float>(p.x()) << static_cast<float>(p.y()) << static_cast<float>(p.z());
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
        scalar = i < static_cast<int>(cloud.intensities.size()) ? static_cast<float>(cloud.intensities[i]) : 0.f;
        break;
    }
    scalars << scalar;
    rgb << 0.5f << 0.5f << 0.5f;
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
  refreshViewerRgb(static_cast<float>(item->blendWeight()));
}

void AppController::uploadFusedCloudToViewer() {
  uploadDenseCloudToViewer(fused_cloud_, viewer_scalar_bounds_valid_);
}

QString AppController::exportInspectionColoredPcd() {
  if (!inspection_mode_) {
    setStatus(QStringLiteral("Switch to the Inspection tab to export inspection clouds"));
    return {};
  }
  return exportFusedPcd();
}

void AppController::syncCorrespondenceMarkers() {
  auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_);
  if (!item) {
    return;
  }
  if (inspection_mode_) {
    item->clearSphereMarkers();
    return;
  }
  session_.ensurePrimaryCalibration();
  QVector<double> positions;
  for (const auto& c : session_.calibration_datasets.front().correspondences) {
    positions << c.x << c.y << c.z;
  }
  item->setSphereMarkerPositions(positions);
  if (pending_correspondence_3d_) {
    item->setPendingSphereMarker((*pending_correspondence_3d_).x(), (*pending_correspondence_3d_).y(),
                                 (*pending_correspondence_3d_).z());
  } else {
    item->clearPendingSphereMarker();
  }
}

void AppController::onPointPicked(double x, double y, double z) {
  qInfo().noquote() << QStringLiteral("3D correspondence pick: x=%1 y=%2 z=%3")
                           .arg(x, 0, 'f', 4)
                           .arg(y, 0, 'f', 4)
                           .arg(z, 0, 'f', 4);
  if (inspection_mode_) {
    setStatus(QStringLiteral("Correspondence picking is only available in Calibration mode"));
    return;
  }
  pending_correspondence_3d_ = Eigen::Vector3d(x, y, z);
  emit correspondencesChanged();
  setStatus(QStringLiteral("3D point picked — click the matching location on the camera image"));
}

void AppController::pickImageCorrespondence(double u, double v) {
  if (inspection_mode_) {
    return;
  }
  if (!pending_correspondence_3d_) {
    setStatus(QStringLiteral("Shift+click a 3D point in the viewer first"));
    return;
  }
  session_.ensurePrimaryCalibration();
  clc::PointCorrespondence c;
  c.u = u;
  c.v = v;
  c.x = (*pending_correspondence_3d_).x();
  c.y = (*pending_correspondence_3d_).y();
  c.z = (*pending_correspondence_3d_).z();
  session_.calibration_datasets.front().correspondences.push_back(c);
  pending_correspondence_3d_.reset();
  emit correspondencesChanged();
  setStatus(QStringLiteral("Correspondence added (%1 total)").arg(correspondenceCount()));
}

void AppController::clearCorrespondences() {
  session_.ensurePrimaryCalibration();
  session_.calibration_datasets.front().correspondences.clear();
  pending_correspondence_3d_.reset();
  emit correspondencesChanged();
  setStatus(QStringLiteral("Correspondences cleared"));
}

void AppController::estimateInitialGuessFromCorrespondences() {
  if (session_.camera.intrinsics.empty()) {
    setStatus(QStringLiteral("Load camera intrinsics before estimating initial guess"));
    return;
  }
  session_.ensurePrimaryCalibration();
  clc::ManualGuessInput input;
  input.camera = camera::create_camera(session_.camera.model, session_.camera.intrinsics, session_.camera.distortion_coeffs);
  input.correspondences = session_.calibration_datasets.front().correspondences;
  input.gizmo_T_camera_lidar = session_.T_camera_lidar_initial;
  const auto result = clc::compute_manual_initial_guess(input);
  if (!result.ok) {
    setStatus(QString::fromStdString(result.message));
    return;
  }
  session_.T_camera_lidar_initial = result.T_camera_lidar;
  session_.T_camera_lidar = result.T_camera_lidar;
  applySessionToViewer();
  refreshViewerColors();
  setStatus(QString::fromStdString(result.message));
}

void AppController::onViewerPoseChanged() {
  auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_);
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
  refreshViewerColors();
}

void AppController::runFusion() {
  if (worker_running_) {
    return;
  }
  if (fusion_blocked_) {
    setStatus(QStringLiteral("Fusion blocked: fix PCD scan geometry"));
    return;
  }
  const auto& catalog = activeDatasetEntry().pcd_paths;
  if (catalog.empty()) {
    setStatus(QStringLiteral("No PCD files loaded"));
    return;
  }
  setWorkerRunning(true);
  setStatus(QStringLiteral("Fusion running..."));
  joinWorker();
  const int ref_w = session_.reference_scan_width;
  const int ref_h = session_.reference_scan_height;
  worker_ = std::make_unique<std::thread>([this, catalog, ref_w, ref_h]() {
    auto params = session_.fusion_params;
    params.scan_time = session_.scan_time;
    const int total_scans = static_cast<int>(catalog.size());

    std::vector<glim_cloud_fusion::LidarScan> scans;
    for (const auto& path : catalog) {
      clc::PointXYZITCloud cloud;
      std::string err;
      if (!clc::load_pcd_xyzit(path, cloud, err)) {
        QMetaObject::invokeMethod(this, [this, err]() {
          setWorkerRunning(false);
          setStatus(QString::fromStdString(err));
          emit fusionFinished(false, QString::fromStdString(err));
        }, Qt::QueuedConnection);
        return;
      }
      if (scans.empty()) {
        if (!params.scan_time.scan_duration.has_value() || *params.scan_time.scan_duration <= 0.0) {
          if (const auto duration = clc::infer_scan_duration_seconds(cloud.scan, params.scan_time)) {
            params.scan_time.scan_duration = *duration;
          }
        }
      }
      if (ref_w > 0 && (cloud.width != ref_w || cloud.height != ref_h)) {
        const std::string dim_err = "scan dimension mismatch in " + path;
        QMetaObject::invokeMethod(this, [this, dim_err]() {
          setWorkerRunning(false);
          setStatus(QString::fromStdString(dim_err));
          emit fusionFinished(false, QString::fromStdString(dim_err));
        }, Qt::QueuedConnection);
        return;
      }
      scans.push_back(to_fusion_scan(cloud));
    }

    const auto result = clc::run_fusion(scans, params, [this](const glim_cloud_fusion::FusionProgress& p) {
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

    QMetaObject::invokeMethod(this, [this, result]() {
      setWorkerRunning(false);
      if (result.error != glim_cloud_fusion::FusionError::Ok) {
        setStatus(QString::fromStdString(result.message));
        emit fusionFinished(false, QString::fromStdString(result.message));
        return;
      }
      fused_cloud_ = result.cloud;
      setPointCount(static_cast<int>(result.cloud.points.size()));
      refreshViewerColormapBounds(false);
      emit viewerColormapChanged();
      setStatus(QStringLiteral("Fusion complete"));
      uploadFusedCloudToViewer();
      emit fusionFinished(true, QStringLiteral("Fusion complete"));
    }, Qt::QueuedConnection);
  });
}

void AppController::runAlignment() {
  if (worker_running_) {
    return;
  }
  if (activeDatasetEntry().image_path.empty() || fused_cloud_.points.empty()) {
    setStatus(QStringLiteral("Need image and fused cloud before alignment"));
    return;
  }
  setWorkerRunning(true);
  setStatus(QStringLiteral("Alignment running..."));
  if (auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_)) {
    item->setInteractionEnabled(false);
    item->setGizmoEnabled(false);
  }
  joinWorker();
  worker_ = std::make_unique<std::thread>([this]() {
    clc::AlignmentInput input;
    std::string build_err;
    if (!buildAlignmentInput(input, build_err)) {
      QMetaObject::invokeMethod(this, [this, build_err]() {
        setWorkerRunning(false);
        if (auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_)) {
          item->setInteractionEnabled(true);
          item->setGizmoEnabled(true);
        }
        setStatus(QString::fromStdString(build_err));
        emit alignmentFinished(false, QString::fromStdString(build_err));
      }, Qt::QueuedConnection);
      return;
    }

    const auto result = clc::run_alignment(input, [this](const vlcal::CalibrationProgress& p) {
      QMetaObject::invokeMethod(
        this,
        [this, p]() {
          session_.T_camera_lidar = p.T_camera_lidar;
          applySessionToViewer();
          refreshViewerColors();
          setStatus(QStringLiteral("cost=%1").arg(p.cost));
        },
        Qt::QueuedConnection);
    });

    QMetaObject::invokeMethod(this, [this, result]() {
      setWorkerRunning(false);
      if (auto* item = qobject_cast<psiviz::PointCloudItem*>(viewer_)) {
        item->setInteractionEnabled(true);
        item->setGizmoEnabled(true);
      }
      if (result.error != vlcal::CalibrationError::Ok) {
        setStatus(QString::fromStdString(result.message));
        emit alignmentFinished(false, QString::fromStdString(result.message));
        return;
      }
      session_.T_camera_lidar = result.T_camera_lidar;
      applySessionToViewer();
      refreshViewerColors();
      setStatus(QStringLiteral("Alignment complete"));
      emit alignmentFinished(true, QStringLiteral("Alignment complete"));
    }, Qt::QueuedConnection);
  });
}
