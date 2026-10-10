#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

#include <QImage>
#include <QObject>
#include <QPointF>
#include <QString>
#include <QVariant>
#include <QTimer>
#include <QVector>

#include <glim_cloud_fusion/fusion.hpp>

#include <clc_core/calibration_service.hpp>
#include <clc_core/point_color_model.hpp>
#include <clc_core/range_image.hpp>
#include <clc_core/session.hpp>

#include <opencv2/core.hpp>

class AppController : public QObject {
  Q_OBJECT
  Q_PROPERTY(QString statusMessage READ statusMessage NOTIFY statusMessageChanged)
  Q_PROPERTY(bool busyWorking READ busyWorking NOTIFY busyWorkingChanged)
  Q_PROPERTY(int pointCount READ pointCount NOTIFY pointCountChanged)
  Q_PROPERTY(double scanDuration READ scanDuration WRITE setScanDuration NOTIFY scanTimeChanged)
  Q_PROPERTY(int timeOrigin READ timeOrigin WRITE setTimeOrigin NOTIFY scanTimeChanged)
  Q_PROPERTY(int timeUnit READ timeUnit WRITE setTimeUnit NOTIFY scanTimeChanged)
  Q_PROPERTY(QString sessionDirectory READ sessionDirectory WRITE setSessionDirectory NOTIFY sessionPathsChanged)
  Q_PROPERTY(QString imagePath READ imagePath WRITE setImagePath NOTIFY sessionPathsChanged)
  Q_PROPERTY(QString pcdDirectory READ pcdDirectory WRITE setPcdDirectory NOTIFY sessionPathsChanged)
  Q_PROPERTY(QString intrinsicsPath READ intrinsicsPath WRITE setIntrinsicsPath NOTIFY sessionPathsChanged)
  Q_PROPERTY(int pcdFileCount READ pcdFileCount NOTIFY pcdCatalogChanged)
  Q_PROPERTY(bool fusionBlocked READ fusionBlocked NOTIFY pcdCatalogChanged)
  Q_PROPERTY(QImage rangeImage READ rangeImage NOTIFY rangeImageChanged)
  Q_PROPERTY(QByteArray maskPreview READ maskPreview NOTIFY maskChanged)
  Q_PROPERTY(int maskWidth READ maskWidth NOTIFY maskChanged)
  Q_PROPERTY(int maskHeight READ maskHeight NOTIFY maskChanged)
  Q_PROPERTY(double rangeColormapMin READ rangeColormapMin WRITE setRangeColormapMin NOTIFY rangeImageChanged)
  Q_PROPERTY(double rangeColormapMax READ rangeColormapMax WRITE setRangeColormapMax NOTIFY rangeImageChanged)
  Q_PROPERTY(int rangeChannel READ rangeChannel WRITE setRangeChannel NOTIFY rangeImageChanged)
  Q_PROPERTY(double rangeYStretch READ rangeYStretch WRITE setRangeYStretch NOTIFY rangeYStretchChanged)
  Q_PROPERTY(bool maskPolygonPending READ maskPolygonPending NOTIFY maskPolygonPendingChanged)
  Q_PROPERTY(QVariantList pendingMaskPolygonVertices READ pendingMaskPolygonVertices NOTIFY maskPolygonPendingChanged)
  Q_PROPERTY(QString maskPath READ maskPath NOTIFY maskChanged)
  Q_PROPERTY(double viewerColormapMin READ viewerColormapMin WRITE setViewerColormapMin NOTIFY viewerColormapChanged)
  Q_PROPERTY(double viewerColormapMax READ viewerColormapMax WRITE setViewerColormapMax NOTIFY viewerColormapChanged)
  Q_PROPERTY(double viewerDataMin READ viewerDataMin NOTIFY viewerColormapChanged)
  Q_PROPERTY(double viewerDataMax READ viewerDataMax NOTIFY viewerColormapChanged)
  Q_PROPERTY(double rangeDataMin READ rangeDataMin NOTIFY rangeImageChanged)
  Q_PROPERTY(double rangeDataMax READ rangeDataMax NOTIFY rangeImageChanged)
  Q_PROPERTY(int viewerScalarChannel READ viewerScalarChannel WRITE setViewerScalarChannel NOTIFY viewerColormapChanged)
  Q_PROPERTY(double fusionVoxelResolution READ fusionVoxelResolution WRITE setFusionVoxelResolution NOTIFY fusionParamsChanged)
  Q_PROPERTY(int fusionNumThreads READ fusionNumThreads WRITE setFusionNumThreads NOTIFY fusionParamsChanged)
  Q_PROPERTY(double fusionMaxCorrespondence READ fusionMaxCorrespondence WRITE setFusionMaxCorrespondence NOTIFY fusionParamsChanged)
  Q_PROPERTY(bool inspectionMode READ inspectionMode WRITE setInspectionMode NOTIFY inspectionModeChanged)
  Q_PROPERTY(int correspondenceCount READ correspondenceCount NOTIFY correspondencesChanged)
  Q_PROPERTY(bool pendingCorrespondence3d READ pendingCorrespondence3d NOTIFY correspondencesChanged)
  Q_PROPERTY(QImage cameraPreviewImage READ cameraPreviewImage NOTIFY cameraPreviewChanged)

public:
  explicit AppController(QObject* parent = nullptr);
  ~AppController() override;

  QString statusMessage() const { return status_message_; }
  bool busyWorking() const { return worker_running_; }
  int pointCount() const { return point_count_; }
  double scanDuration() const;
  int timeOrigin() const;
  int timeUnit() const;
  QString sessionDirectory() const { return session_directory_; }
  QString imagePath() const;
  QString pcdDirectory() const;
  QString intrinsicsPath() const;
  int pcdFileCount() const { return static_cast<int>(session_.pcd_paths.size()); }
  bool fusionBlocked() const { return fusion_blocked_; }
  QImage rangeImage() const { return range_image_display_; }
  QByteArray maskPreview() const;
  int maskWidth() const;
  int maskHeight() const;
  double rangeColormapMin() const { return range_colormap_min_; }
  double rangeColormapMax() const { return range_colormap_max_; }
  int rangeChannel() const { return range_channel_; }
  double rangeYStretch() const { return range_y_stretch_; }
  bool maskPolygonPending() const { return mask_polygon_pending_; }
  QVariantList pendingMaskPolygonVertices() const;
  QString maskPath() const { return QString::fromStdString(session_.global_mask.path); }
  double viewerColormapMin() const { return viewer_colormap_min_; }
  double viewerColormapMax() const { return viewer_colormap_max_; }
  double viewerDataMin() const { return viewer_data_min_; }
  double viewerDataMax() const { return viewer_data_max_; }
  double rangeDataMin() const { return range_data_min_; }
  double rangeDataMax() const { return range_data_max_; }
  int viewerScalarChannel() const { return viewer_scalar_channel_; }
  double fusionVoxelResolution() const { return session_.fusion_params.voxel_resolution; }
  int fusionNumThreads() const { return session_.fusion_params.num_threads; }
  double fusionMaxCorrespondence() const { return session_.fusion_params.max_correspondence_distance; }
  bool inspectionMode() const { return inspection_mode_; }
  int correspondenceCount() const;
  bool pendingCorrespondence3d() const { return pending_correspondence_3d_.has_value(); }
  QImage cameraPreviewImage() const { return camera_preview_; }

  void setScanDuration(double seconds);
  void setTimeOrigin(int origin);
  void setTimeUnit(int unit);
  void setSessionDirectory(const QString& directory);
  void setImagePath(const QString& path);
  void setPcdDirectory(const QString& path);
  void setIntrinsicsPath(const QString& path);

  Q_INVOKABLE bool loadSession(const QString& directory);
  Q_INVOKABLE bool saveSession(const QString& directory);
  Q_INVOKABLE bool loadIntrinsicsFile(const QString& path);
  Q_INVOKABLE QString browseSessionDirectory();
  Q_INVOKABLE QString browsePcdDirectory();
  Q_INVOKABLE QString browseImageFile();
  Q_INVOKABLE QString browseIntrinsicsFile();
  Q_INVOKABLE void setRangeColormapMin(double v);
  Q_INVOKABLE void setRangeColormapMax(double v);
  Q_INVOKABLE void setRangeChannel(int channel);
  Q_INVOKABLE void setRangeYStretch(double stretch);
  Q_INVOKABLE void setViewerColormapMin(double v);
  Q_INVOKABLE void setViewerColormapMax(double v);
  Q_INVOKABLE void setViewerScalarChannel(int channel);
  Q_INVOKABLE void setFusionVoxelResolution(double v);
  Q_INVOKABLE void setFusionNumThreads(int n);
  Q_INVOKABLE void setFusionMaxCorrespondence(double v);
  Q_INVOKABLE void refreshViewerColors();
  Q_INVOKABLE void previewViewerColormapRange(double min_v, double max_v);
  Q_INVOKABLE void commitViewerColormapRange(double min_v, double max_v);
  Q_INVOKABLE void previewRangeColormapRange(double min_v, double max_v);
  Q_INVOKABLE void commitRangeColormapRange(double min_v, double max_v);
  Q_INVOKABLE QString exportFusedPcd();
  Q_INVOKABLE void commitMaskPolygon(const QVariantList& nativeVertices);
  void commitMaskPolygonNative(const std::vector<QPointF>& nativeVertices);
  Q_INVOKABLE void applyPendingMaskKeep();
  Q_INVOKABLE void applyPendingMaskRemove();
  Q_INVOKABLE void discardPendingMaskPolygon();
  Q_INVOKABLE void clearMask();
  Q_INVOKABLE bool saveMask();
  Q_INVOKABLE bool loadMask();
  Q_INVOKABLE QString browseMaskFile();
  Q_INVOKABLE void setInspectionMode(bool inspection);
  Q_INVOKABLE void runFusion();
  Q_INVOKABLE void runAlignment();
  Q_INVOKABLE void estimateInitialGuessFromCorrespondences();
  Q_INVOKABLE void clearCorrespondences();
  Q_INVOKABLE void pickImageCorrespondence(double u, double v);
  Q_INVOKABLE QString exportInspectionColoredPcd();
  Q_INVOKABLE void setViewer(QObject* viewer);
  Q_INVOKABLE void refreshPreviewInViewer();
  Q_INVOKABLE void uploadFusedCloudToViewer();

signals:
  void statusMessageChanged();
  void busyWorkingChanged();
  void pointCountChanged();
  void scanTimeChanged();
  void sessionPathsChanged();
  void pcdCatalogChanged();
  void sessionLoaded();
  void rangeImageChanged();
  void rangeYStretchChanged();
  void maskChanged();
  void maskPolygonPendingChanged();
  void viewerColormapChanged();
  void fusionParamsChanged();
  void fusionFinished(bool ok, const QString& message);
  void alignmentFinished(bool ok, const QString& message);
  void inspectionModeChanged();
  void correspondencesChanged();
  void cameraPreviewChanged();
private slots:
  void onPointPicked(qint64 point_index);
  void onViewerPoseChanged();
  void onViewerColormapDebounce();
  void onRangeColormapDebounce();

private:
  void setStatus(const QString& message);
  void setPointCount(int count);
  void refreshPcdCatalog();
  void refreshRangeImage();
  void uploadPreviewScanToViewer(bool preserve_view_state = false);
  void uploadDenseCloudToViewer(const glim_cloud_fusion::DensePointCloud& cloud, bool preserve_view_state = false);
  void uploadLidarScanToViewer(const clc::PointXYZITCloud& cloud, bool preserve_view_state = false);
  void refreshPreviewCloudFromMask(bool preserve_view_state = true);
  void refreshCameraPreview();
  void syncActiveDatasetFromLegacy();
  void syncLegacyFromActiveDataset();
  clc::DatasetEntry& activeDatasetEntry();
  const clc::DatasetEntry& activeDatasetEntry() const;
  bool loadFirstReadableScan(clc::PointXYZITCloud& cloud, std::string& err);
  void maybeInferScanDurationFromScan(const clc::PointXYZITCloud& cloud);
  void rebuildMaskFromOps();
  int maskRasterWidth() const;
  int maskRasterHeight() const;
  void setMaskPolygonPending(bool pending);
  void refreshViewerColormapBounds(bool reset_colormap_range = true);
  void refreshRangeImageDisplay();
  void applyViewerScalarRangeToItem();
  void applySessionToViewer();
  void emitSessionUiState();
  void syncColorModelFromSession();
  void refreshViewerRgb(float blend_weight);
  void syncCorrespondenceMarkers();
  void mutateViewerPointAtIndex(qint64 point_index);
  bool buildAlignmentInput(clc::AlignmentInput& input, std::string& error);
  void joinWorker();
  void setWorkerRunning(bool running);

  clc::SessionData session_;
  clc::PointColorModel color_model_;
  cv::Mat projection_image_;
  QVector<float> viewer_positions_;
  QVector<float> viewer_scalars_;
  QVector<float> viewer_rgb_;
  std::vector<std::uint8_t> global_mask_pixels_;
  std::vector<clc::MaskPolygonOp> mask_ops_;
  clc::Polygon2f pending_mask_polygon_;
  bool mask_polygon_pending_ = false;
  clc::RangeImage range_image_;
  QImage range_image_display_;
  double range_colormap_min_ = 0.0;
  double range_colormap_max_ = 50.0;
  int range_channel_ = 0;
  double range_y_stretch_ = 1.0;
  bool preview_upload_pending_ = false;
  bool viewer_scalar_bounds_valid_ = false;
  double viewer_colormap_min_ = 0.0;
  double viewer_colormap_max_ = 1.0;
  double viewer_data_min_ = 0.0;
  double viewer_data_max_ = 1.0;
  double range_data_min_ = 0.0;
  double range_data_max_ = 50.0;
  int viewer_scalar_channel_ = 0;
  QTimer viewer_colormap_debounce_;
  QTimer range_colormap_debounce_;
  glim_cloud_fusion::DensePointCloud fused_cloud_;
  QString status_message_ = QStringLiteral("Ready");
  QString session_directory_;
  int point_count_ = 0;
  bool fusion_blocked_ = false;
  bool inspection_mode_ = false;
  std::optional<Eigen::Vector3d> pending_correspondence_3d_;
  QImage camera_preview_;
  std::atomic<bool> worker_running_{false};
  std::unique_ptr<std::thread> worker_;
  class QObject* viewer_{nullptr};
};
