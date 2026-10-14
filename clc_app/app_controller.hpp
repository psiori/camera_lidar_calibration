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
  Q_PROPERTY(int rangeImageNativeWidth READ rangeImageNativeWidth NOTIFY rangeImageChanged)
  Q_PROPERTY(int rangeImageNativeHeight READ rangeImageNativeHeight NOTIFY rangeImageChanged)
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
  Q_PROPERTY(int fusionKNeighbors READ fusionKNeighbors WRITE setFusionKNeighbors NOTIFY fusionParamsChanged)
  Q_PROPERTY(int fusionBackend READ fusionBackend WRITE setFusionBackend NOTIFY fusionParamsChanged)
  Q_PROPERTY(bool fusionBackendIsIntegrator READ fusionBackendIsIntegrator NOTIFY fusionParamsChanged)
  Q_PROPERTY(int fusionTargetNumPoints READ fusionTargetNumPoints WRITE setFusionTargetNumPoints NOTIFY fusionParamsChanged)
  Q_PROPERTY(double fusionMinDistance READ fusionMinDistance WRITE setFusionMinDistance NOTIFY fusionParamsChanged)
  Q_PROPERTY(double fusionIntegratorIvoxResolution READ fusionIntegratorIvoxResolution WRITE setFusionIntegratorIvoxResolution
               NOTIFY fusionParamsChanged)
  Q_PROPERTY(double fusionIntegratorIvoxMinPointsDist READ fusionIntegratorIvoxMinPointsDist
               WRITE setFusionIntegratorIvoxMinPointsDist NOTIFY fusionParamsChanged)
  Q_PROPERTY(double fusionIvoxResolution READ fusionIvoxResolution WRITE setFusionIvoxResolution NOTIFY fusionParamsChanged)
  Q_PROPERTY(int fusionLmMaxIterations READ fusionLmMaxIterations WRITE setFusionLmMaxIterations NOTIFY fusionParamsChanged)
  Q_PROPERTY(int calibrationMaxOuterIterations READ calibrationMaxOuterIterations WRITE setCalibrationMaxOuterIterations
                 NOTIFY calibrationParamsChanged)
  Q_PROPERTY(int calibrationMaxInnerIterations READ calibrationMaxInnerIterations WRITE setCalibrationMaxInnerIterations
                 NOTIFY calibrationParamsChanged)
  Q_PROPERTY(int calibrationNumThreads READ calibrationNumThreads WRITE setCalibrationNumThreads NOTIFY calibrationParamsChanged)
  Q_PROPERTY(double calibrationDeltaTransThresh READ calibrationDeltaTransThresh WRITE setCalibrationDeltaTransThresh
                 NOTIFY calibrationParamsChanged)
  Q_PROPERTY(double calibrationDeltaRotDeg READ calibrationDeltaRotDeg WRITE setCalibrationDeltaRotDeg NOTIFY calibrationParamsChanged)
  Q_PROPERTY(bool calibrationDisableZBufferCulling READ calibrationDisableZBufferCulling WRITE setCalibrationDisableZBufferCulling
                 NOTIFY calibrationParamsChanged)
  Q_PROPERTY(int calibrationNidBins READ calibrationNidBins WRITE setCalibrationNidBins NOTIFY calibrationParamsChanged)
  Q_PROPERTY(int calibrationRegistrationType READ calibrationRegistrationType WRITE setCalibrationRegistrationType
                 NOTIFY calibrationParamsChanged)
  Q_PROPERTY(double calibrationNelderMeadInitStep READ calibrationNelderMeadInitStep WRITE setCalibrationNelderMeadInitStep
                 NOTIFY calibrationParamsChanged)
  Q_PROPERTY(double calibrationNelderMeadConvergence READ calibrationNelderMeadConvergence WRITE setCalibrationNelderMeadConvergence
                 NOTIFY calibrationParamsChanged)
  Q_PROPERTY(bool inspectionMode READ inspectionMode WRITE setInspectionMode NOTIFY inspectionModeChanged)
  Q_PROPERTY(int mainWorkTab READ mainWorkTab WRITE setMainWorkTab NOTIFY mainWorkTabChanged)
  Q_PROPERTY(int activeCalibrationDatasetIndex READ activeCalibrationDatasetIndex WRITE setActiveCalibrationDatasetIndex
                 NOTIFY calibrationDatasetsChanged)
  Q_PROPERTY(QStringList calibrationDatasetIds READ calibrationDatasetIds NOTIFY calibrationDatasetsChanged)
  Q_PROPERTY(bool activeDatasetReady READ activeDatasetReady NOTIFY calibrationDatasetsChanged)
  Q_PROPERTY(bool canPickCorrespondences READ canPickCorrespondences NOTIFY calibrationDatasetsChanged)
  Q_PROPERTY(bool activeDatasetIsSingleScan READ activeDatasetIsSingleScan NOTIFY pcdCatalogChanged)
  Q_PROPERTY(QString addDatasetDefaultImagePath READ addDatasetDefaultImagePath CONSTANT)
  Q_PROPERTY(QString addDatasetDefaultLidarPath READ addDatasetDefaultLidarPath CONSTANT)
  Q_PROPERTY(QString addDatasetDefaultIntrinsicsPath READ addDatasetDefaultIntrinsicsPath CONSTANT)
  Q_PROPERTY(int correspondenceCount READ correspondenceCount NOTIFY correspondencesChanged)
  Q_PROPERTY(bool pendingCorrespondence3d READ pendingCorrespondence3d NOTIFY correspondencesChanged)
  Q_PROPERTY(bool pendingCorrespondence2d READ pendingCorrespondence2d NOTIFY correspondencesChanged)
  Q_PROPERTY(int selectedCorrespondenceRow READ selectedCorrespondenceRow WRITE setSelectedCorrespondenceRow
                 NOTIFY correspondencesChanged)
  Q_PROPERTY(QVariantList correspondenceTable READ correspondenceTable NOTIFY correspondencesChanged)
  Q_PROPERTY(bool canComputeInitialAlignment READ canComputeInitialAlignment NOTIFY correspondencesChanged)
  Q_PROPERTY(bool manualInitialGuessSupportsMultiDataset READ manualInitialGuessSupportsMultiDataset CONSTANT)
  Q_PROPERTY(int initialAlignmentMethod READ initialAlignmentMethod WRITE setInitialAlignmentMethod
                 NOTIFY initialAlignmentMethodChanged)
  Q_PROPERTY(bool pickingAllowed READ pickingAllowed NOTIFY busyWorkingChanged)
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
  int pcdFileCount() const;
  bool fusionBlocked() const { return fusion_blocked_; }
  QImage rangeImage() const { return range_image_display_; }
  int rangeImageNativeWidth() const { return range_image_.width; }
  int rangeImageNativeHeight() const { return range_image_.height; }
  QByteArray maskPreview() const;
  int maskWidth() const;
  int maskHeight() const;
  double rangeColormapMin() const { return range_colormap_min_; }
  double rangeColormapMax() const { return range_colormap_max_; }
  int rangeChannel() const { return range_channel_; }
  double rangeYStretch() const { return range_y_stretch_; }
  bool maskPolygonPending() const { return mask_polygon_pending_; }
  QVariantList pendingMaskPolygonVertices() const;
  std::vector<QPointF> pendingMaskPolygonNativePoints() const;
  QString maskPath() const { return QString::fromStdString(session_.global_mask.path); }
  double viewerColormapMin() const { return viewer_colormap_min_; }
  double viewerColormapMax() const { return viewer_colormap_max_; }
  double viewerDataMin() const { return viewer_data_min_; }
  double viewerDataMax() const { return viewer_data_max_; }
  double rangeDataMin() const { return range_data_min_; }
  double rangeDataMax() const { return range_data_max_; }
  int viewerScalarChannel() const { return viewer_scalar_channel_; }
  double fusionVoxelResolution() const;
  int fusionNumThreads() const;
  double fusionMaxCorrespondence() const;
  int fusionKNeighbors() const;
  int fusionBackend() const;
  bool fusionBackendIsIntegrator() const;
  int fusionTargetNumPoints() const;
  double fusionMinDistance() const;
  double fusionIntegratorIvoxResolution() const;
  double fusionIntegratorIvoxMinPointsDist() const;
  double fusionIvoxResolution() const;
  int fusionLmMaxIterations() const;
  int calibrationMaxOuterIterations() const;
  int calibrationMaxInnerIterations() const;
  int calibrationNumThreads() const;
  double calibrationDeltaTransThresh() const;
  double calibrationDeltaRotDeg() const;
  bool calibrationDisableZBufferCulling() const;
  int calibrationNidBins() const;
  int calibrationRegistrationType() const;
  double calibrationNelderMeadInitStep() const;
  double calibrationNelderMeadConvergence() const;
  bool inspectionMode() const { return main_work_tab_ == 3; }
  int mainWorkTab() const { return main_work_tab_; }
  int activeCalibrationDatasetIndex() const { return active_calibration_dataset_index_; }
  QStringList calibrationDatasetIds() const;
  bool activeDatasetReady() const;
  bool canPickCorrespondences() const;
  bool activeDatasetIsSingleScan() const;
  QString addDatasetDefaultImagePath() const;
  QString addDatasetDefaultLidarPath() const;
  QString addDatasetDefaultIntrinsicsPath() const;
  int correspondenceCount() const;
  bool pendingCorrespondence3d() const { return pending_correspondence_3d_.has_value(); }
  bool pendingCorrespondence2d() const { return pending_correspondence_2d_.has_value(); }
  int selectedCorrespondenceRow() const { return selected_correspondence_row_; }
  QVariantList correspondenceTable() const;
  bool canComputeInitialAlignment() const;
  bool manualInitialGuessSupportsMultiDataset() const;
  int initialAlignmentMethod() const { return initial_alignment_method_; }
  bool pickingAllowed() const { return !worker_running_; }
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
  Q_INVOKABLE QString browsePcdFile();
  Q_INVOKABLE QString browsePcdFolder();
  Q_INVOKABLE QString browseImageFile();
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
  void setFusionKNeighbors(int n);
  void setFusionBackend(int backend);
  void setFusionTargetNumPoints(int n);
  void setFusionMinDistance(double v);
  void setFusionIntegratorIvoxResolution(double v);
  void setFusionIntegratorIvoxMinPointsDist(double v);
  void setFusionIvoxResolution(double v);
  void setFusionLmMaxIterations(int n);
  void setCalibrationMaxOuterIterations(int v);
  void setCalibrationMaxInnerIterations(int v);
  void setCalibrationNumThreads(int v);
  void setCalibrationDeltaTransThresh(double v);
  void setCalibrationDeltaRotDeg(double v);
  void setCalibrationDisableZBufferCulling(bool v);
  void setCalibrationNidBins(int v);
  void setCalibrationRegistrationType(int v);
  void setCalibrationNelderMeadInitStep(double v);
  void setCalibrationNelderMeadConvergence(double v);
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
#ifdef CLC_APP_TESTING
  Q_INVOKABLE void installSyntheticMaskEditorScan(int native_width, int native_height, double y_stretch);
  Q_INVOKABLE void setReferenceScanSizeForTest(int width, int height);
#endif
  Q_INVOKABLE void clearMask();
  Q_INVOKABLE bool saveMask();
  Q_INVOKABLE bool loadMask();
  Q_INVOKABLE QString browseMaskFile();
  Q_INVOKABLE void setInspectionMode(bool inspection);
  Q_INVOKABLE void setMainWorkTab(int tab);
  Q_INVOKABLE void setActiveCalibrationDatasetIndex(int index);
  Q_INVOKABLE bool addCalibrationDatasetFromPaths(const QString& image_path, const QString& lidar_path,
                                                const QString& intrinsics_path);
  Q_INVOKABLE QString pickAddDatasetImage(const QString& initial_dir);
  Q_INVOKABLE QString pickAddDatasetLidarFile(const QString& initial_dir);
  Q_INVOKABLE QString pickAddDatasetLidarFolder(const QString& initial_dir);
  Q_INVOKABLE QString pickAddDatasetIntrinsics(const QString& initial_dir);
  Q_INVOKABLE void removeActiveCalibrationDataset();
  Q_INVOKABLE bool isCalibrationDatasetReady(int index) const;
  Q_INVOKABLE QString calibrationDatasetTooltip(int index) const;
  Q_INVOKABLE void setSelectedCorrespondenceRow(int global_row);
  Q_INVOKABLE void deleteCorrespondenceRow(int global_row);
  Q_INVOKABLE void setInitialAlignmentMethod(int method);
  Q_INVOKABLE void setMaskPreviewDatasetIndex(int index);
  Q_INVOKABLE void runFusion();
  Q_INVOKABLE void runAlignment();
  Q_INVOKABLE void cancelWorker();
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
  void calibrationParamsChanged();
  void fusionFinished(bool ok, const QString& message);
  void alignmentFinished(bool ok, const QString& message);
  void inspectionModeChanged();
  void mainWorkTabChanged();
  void calibrationDatasetsChanged();
  void initialAlignmentMethodChanged();
  void correspondencesChanged();
  void cameraPreviewChanged();
private slots:
  void onPointPicked(double x, double y, double z);
  void onViewerPoseChanged();
  void onViewerColormapDebounce();
  void onRangeColormapDebounce();

private:
  void setStatus(const QString& message);
  void setPointCount(int count);
  void refreshPcdCatalog();
  /// Re-scan cloud_directory when the in-memory catalog was cleared (e.g. after fusion).
  bool repopulateActiveDatasetPcdCatalogIfEmpty();
  void refreshRangeImage();
  void uploadPreviewScanToViewer(bool preserve_view_state = false);
  void uploadDenseCloudToViewer(const glim_cloud_fusion::DensePointCloud& cloud, bool preserve_view_state = false);
  void uploadLidarScanToViewer(const clc::PointXYZITCloud& cloud, bool preserve_view_state = false);
  void refreshPreviewCloudFromMask(bool preserve_view_state = true);
  void refreshCameraPreview();
  clc::DatasetEntry& activeDatasetEntry();
  const clc::DatasetEntry& activeDatasetEntry() const;
  void ensureActiveCalibrationDataset();
  bool loadFirstReadableScan(clc::PointXYZITCloud& cloud, std::string& err);
  void rebuildMaskFromOps();
  int maskRasterWidth() const;
  int maskRasterHeight() const;
  void setMaskPolygonPending(bool pending);
  void refreshViewerColormapBounds(bool reset_colormap_range = true);
  void refreshRangeImageDisplay();
  void applyViewerScalarRangeToItem();
  void applySessionToViewer();
  void refreshViewerGizmoState();
  void loadActiveDatasetCloudToViewer();
  void syncActiveDatasetFusionToSession();
  void syncSessionFusionToActiveDataset();
  void emitSessionUiState();
  void syncColorModelFromSession();
  bool ensureProjectionImageForActiveDataset();
  void invalidateProjectionImageCache();
  void clearCalibrationDatasetVisuals();
  void resetSessionStructuredScanState();
  void refreshActiveDatasetVisuals();
  void reprojectViewerCameraRgb();
  void syncCorrespondenceMarkers();
  void finalizePendingCorrespondenceIfReady();
  bool buildAlignmentInput(clc::AlignmentInput& input, std::string& error);
  void joinWorker();
  void setWorkerRunning(bool running);

  clc::SessionData session_;
  clc::DatasetEntry empty_calibration_dataset_scratch_;
  clc::PointColorModel color_model_;
  cv::Mat projection_image_;
  std::string projection_image_path_;
  std::optional<int> fused_cloud_dataset_index_;
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
  double range_y_stretch_ = 2.0;
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
  int main_work_tab_ = 0;
  int active_calibration_dataset_index_ = 0;
  int mask_preview_dataset_index_ = 0;
  int initial_alignment_method_ = 0;
  int selected_correspondence_row_ = -1;
  std::optional<Eigen::Vector3d> pending_correspondence_3d_;
  std::optional<std::pair<double, double>> pending_correspondence_2d_;
  QImage camera_preview_;
  std::atomic<bool> worker_running_{false};
  std::atomic<bool> worker_cancel_requested_{false};
  std::unique_ptr<std::thread> worker_;
  class QObject* viewer_{nullptr};
};
