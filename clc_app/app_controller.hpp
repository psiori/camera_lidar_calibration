#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <thread>
#include <unordered_map>
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

/// Qt bridge between QML UI and clc_core: session I/O, fusion, calibration, mask editor, and psiviz viewer.
class AppController : public QObject {
  Q_OBJECT

  // Status and progress
  /// Text shown in the status bar (errors, hints, worker progress).
  Q_PROPERTY(QString statusMessage READ statusMessage NOTIFY statusMessageChanged)
  /// True while fusion or alignment runs on a background thread.
  Q_PROPERTY(bool busyWorking READ busyWorking NOTIFY busyWorkingChanged)
  /// Number of points currently loaded in the 3D viewer.
  Q_PROPERTY(int pointCount READ pointCount NOTIFY pointCountChanged)

  // Scan timing (multi-scan fusion)
  /// Total time span covered by one fused scan sequence, in session time units.
  Q_PROPERTY(double scanDuration READ scanDuration WRITE setScanDuration NOTIFY scanTimeChanged)
  /// Enum index: where scan timestamps are measured from (session default).
  Q_PROPERTY(int timeOrigin READ timeOrigin WRITE setTimeOrigin NOTIFY scanTimeChanged)
  /// Enum index: unit for scan timestamps (e.g. seconds vs milliseconds).
  Q_PROPERTY(int timeUnit READ timeUnit WRITE setTimeUnit NOTIFY scanTimeChanged)

  // Session paths (primary / legacy single-dataset fields mirror active dataset)
  /// Folder that contains or will contain session.json.
  Q_PROPERTY(QString sessionDirectory READ sessionDirectory WRITE setSessionDirectory NOTIFY sessionPathsChanged)
  /// Camera image path for the active calibration dataset.
  Q_PROPERTY(QString imagePath READ imagePath WRITE setImagePath NOTIFY sessionPathsChanged)
  /// Directory or file path for lidar PCD(s) on the active dataset.
  Q_PROPERTY(QString pcdDirectory READ pcdDirectory WRITE setPcdDirectory NOTIFY sessionPathsChanged)
  /// OpenCV intrinsics YAML/XML path for the active dataset.
  Q_PROPERTY(QString intrinsicsPath READ intrinsicsPath WRITE setIntrinsicsPath NOTIFY sessionPathsChanged)
  /// How many PCD files are in the active dataset catalog.
  Q_PROPERTY(int pcdFileCount READ pcdFileCount NOTIFY pcdCatalogChanged)
  /// True when fusion cannot run (invalid catalog, geometry error, etc.).
  Q_PROPERTY(bool fusionBlocked READ fusionBlocked NOTIFY pcdCatalogChanged)

  // Range image preview (mask editor and calibration UI)
  /// JET-colored range or intensity image for QML Image/RangeImageItem.
  Q_PROPERTY(QImage rangeImage READ rangeImage NOTIFY rangeImageChanged)
  /// Pixel width of the underlying structured scan (not display stretch).
  Q_PROPERTY(int rangeImageNativeWidth READ rangeImageNativeWidth NOTIFY rangeImageChanged)
  Q_PROPERTY(int rangeImageNativeHeight READ rangeImageNativeHeight NOTIFY rangeImageChanged)
  /// Grayscale mask bytes for overlay (255 = keep, 0 = drop).
  Q_PROPERTY(QByteArray maskPreview READ maskPreview NOTIFY maskChanged)
  Q_PROPERTY(int maskWidth READ maskWidth NOTIFY maskChanged)
  Q_PROPERTY(int maskHeight READ maskHeight NOTIFY maskChanged)
  /// Colormap slider low end for the range image display.
  Q_PROPERTY(double rangeColormapMin READ rangeColormapMin WRITE setRangeColormapMin NOTIFY rangeImageChanged)
  Q_PROPERTY(double rangeColormapMax READ rangeColormapMax WRITE setRangeColormapMax NOTIFY rangeImageChanged)
  /// 0 = range channel, 1 = intensity channel.
  Q_PROPERTY(int rangeChannel READ rangeChannel WRITE setRangeChannel NOTIFY rangeImageChanged)
  /// Vertical scale factor when painting the range image in the mask window.
  Q_PROPERTY(double rangeYStretch READ rangeYStretch WRITE setRangeYStretch NOTIFY rangeYStretchChanged)
  /// True when a closed polygon awaits keep/remove in the mask editor.
  Q_PROPERTY(bool maskPolygonPending READ maskPolygonPending NOTIFY maskPolygonPendingChanged)
  /// Vertices of the polygon waiting for keep/remove (native range coordinates).
  Q_PROPERTY(QVariantList pendingMaskPolygonVertices READ pendingMaskPolygonVertices NOTIFY maskPolygonPendingChanged)
  /// On-disk path of the global mask PNG in the session.
  Q_PROPERTY(QString maskPath READ maskPath NOTIFY maskChanged)

  // Point cloud viewer colormap
  Q_PROPERTY(double viewerColormapMin READ viewerColormapMin WRITE setViewerColormapMin NOTIFY viewerColormapChanged)
  Q_PROPERTY(double viewerColormapMax READ viewerColormapMax WRITE setViewerColormapMax NOTIFY viewerColormapChanged)
  /// Auto-detected min of the active scalar channel in the viewer cloud.
  Q_PROPERTY(double viewerDataMin READ viewerDataMin NOTIFY viewerColormapChanged)
  Q_PROPERTY(double viewerDataMax READ viewerDataMax NOTIFY viewerColormapChanged)
  Q_PROPERTY(double rangeDataMin READ rangeDataMin NOTIFY rangeImageChanged)
  Q_PROPERTY(double rangeDataMax READ rangeDataMax NOTIFY rangeImageChanged)
  /// 0 = range-like scalar, 1 = intensity for psiviz coloring.
  Q_PROPERTY(int viewerScalarChannel READ viewerScalarChannel WRITE setViewerScalarChannel NOTIFY viewerColormapChanged)

  // Fusion parameters (session + per-dataset; UI edits active dataset)
  Q_PROPERTY(double fusionVoxelResolution READ fusionVoxelResolution WRITE setFusionVoxelResolution NOTIFY fusionParamsChanged)
  Q_PROPERTY(int fusionNumThreads READ fusionNumThreads WRITE setFusionNumThreads NOTIFY fusionParamsChanged)
  Q_PROPERTY(double fusionMaxCorrespondence READ fusionMaxCorrespondence WRITE setFusionMaxCorrespondence NOTIFY fusionParamsChanged)
  Q_PROPERTY(int fusionKNeighbors READ fusionKNeighbors WRITE setFusionKNeighbors NOTIFY fusionParamsChanged)
  /// Backend enum as int (SLAM vs integrator, etc.).
  Q_PROPERTY(int fusionBackend READ fusionBackend WRITE setFusionBackend NOTIFY fusionParamsChanged)
  /// True when the selected backend is the continuous-time integrator path.
  Q_PROPERTY(bool fusionBackendIsIntegrator READ fusionBackendIsIntegrator NOTIFY fusionParamsChanged)
  Q_PROPERTY(int fusionTargetNumPoints READ fusionTargetNumPoints WRITE setFusionTargetNumPoints NOTIFY fusionParamsChanged)
  Q_PROPERTY(double fusionMinDistance READ fusionMinDistance WRITE setFusionMinDistance NOTIFY fusionParamsChanged)
  Q_PROPERTY(double fusionIntegratorIvoxResolution READ fusionIntegratorIvoxResolution WRITE setFusionIntegratorIvoxResolution
               NOTIFY fusionParamsChanged)
  Q_PROPERTY(double fusionIntegratorIvoxMinPointsDist READ fusionIntegratorIvoxMinPointsDist
               WRITE setFusionIntegratorIvoxMinPointsDist NOTIFY fusionParamsChanged)
  Q_PROPERTY(double fusionIvoxResolution READ fusionIvoxResolution WRITE setFusionIvoxResolution NOTIFY fusionParamsChanged)
  Q_PROPERTY(int fusionLmMaxIterations READ fusionLmMaxIterations WRITE setFusionLmMaxIterations NOTIFY fusionParamsChanged)

  // Visual–lidar calibration (NID / vlcal) parameters
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

  // UI mode and tabs
  /// True when the inspection workflow tab is selected (mainWorkTab == 3).
  Q_PROPERTY(bool inspectionMode READ inspectionMode WRITE setInspectionMode NOTIFY inspectionModeChanged)
  /// Index of the main tab strip (calibration, fusion, alignment, inspection, …).
  Q_PROPERTY(int mainWorkTab READ mainWorkTab WRITE setMainWorkTab NOTIFY mainWorkTabChanged)

  // Multi-dataset calibration list
  Q_PROPERTY(int activeCalibrationDatasetIndex READ activeCalibrationDatasetIndex WRITE setActiveCalibrationDatasetIndex
                 NOTIFY calibrationDatasetsChanged)
  /// Short ids shown in the dataset combo box.
  Q_PROPERTY(QStringList calibrationDatasetIds READ calibrationDatasetIds NOTIFY calibrationDatasetsChanged)
  /// True when the active dataset has image, intrinsics, and lidar ready for alignment.
  Q_PROPERTY(bool activeDatasetReady READ activeDatasetReady NOTIFY calibrationDatasetsChanged)
  /// True when the user may add 2D/3D correspondence pairs.
  Q_PROPERTY(bool canPickCorrespondences READ canPickCorrespondences NOTIFY calibrationDatasetsChanged)
  /// False while fusion/alignment runs or the modal mask editor is open.
  Q_PROPERTY(bool canChangeActiveCalibrationDataset READ canChangeActiveCalibrationDataset
                 NOTIFY activeDatasetSelectionLockChanged)
  /// True while the global mask editor window is visible (main window is blocked).
  Q_PROPERTY(bool maskEditorOpen READ maskEditorOpen NOTIFY maskEditorOpenChanged)
  /// True while the per-dataset image optimization mask editor is visible.
  Q_PROPERTY(bool imageMaskEditorOpen READ imageMaskEditorOpen NOTIFY imageMaskEditorOpenChanged)
  Q_PROPERTY(bool imageMaskPolygonPending READ imageMaskPolygonPending NOTIFY imageMaskPolygonPendingChanged)
  Q_PROPERTY(QString imageMaskPath READ imageMaskPath NOTIFY imageMaskChanged)
  Q_PROPERTY(QByteArray imageMaskPreview READ imageMaskPreview NOTIFY imageMaskChanged)
  Q_PROPERTY(int imageMaskWidth READ imageMaskWidth NOTIFY imageMaskChanged)
  Q_PROPERTY(int imageMaskHeight READ imageMaskHeight NOTIFY imageMaskChanged)
  /// Which calibration dataset drives the range image inside the mask editor.
  Q_PROPERTY(int maskPreviewDatasetIndex READ maskPreviewDatasetIndex WRITE setMaskPreviewDatasetIndex
                 NOTIFY maskPreviewDatasetIndexChanged)
  /// True when the active catalog is a single PCD (fusion is a no-op passthrough).
  Q_PROPERTY(bool activeDatasetIsSingleScan READ activeDatasetIsSingleScan NOTIFY pcdCatalogChanged)
  /// Default file dialog start paths for Add Dataset (fixed for app lifetime).
  Q_PROPERTY(QString addDatasetDefaultImagePath READ addDatasetDefaultImagePath CONSTANT)
  Q_PROPERTY(QString addDatasetDefaultLidarPath READ addDatasetDefaultLidarPath CONSTANT)
  Q_PROPERTY(QString addDatasetDefaultIntrinsicsPath READ addDatasetDefaultIntrinsicsPath CONSTANT)

  // Manual initial alignment correspondences
  Q_PROPERTY(int correspondenceCount READ correspondenceCount NOTIFY correspondencesChanged)
  /// User picked a 3D point in the viewer and still needs the matching image click.
  Q_PROPERTY(bool pendingCorrespondence3d READ pendingCorrespondence3d NOTIFY correspondencesChanged)
  Q_PROPERTY(bool pendingCorrespondence2d READ pendingCorrespondence2d NOTIFY correspondencesChanged)
  Q_PROPERTY(int selectedCorrespondenceRow READ selectedCorrespondenceRow WRITE setSelectedCorrespondenceRow
                 NOTIFY correspondencesChanged)
  /// Table rows for QML ListView: u, v, x, y, z per pair.
  Q_PROPERTY(QVariantList correspondenceTable READ correspondenceTable NOTIFY correspondencesChanged)
  Q_PROPERTY(bool canComputeInitialAlignment READ canComputeInitialAlignment NOTIFY correspondencesChanged)
  Q_PROPERTY(bool manualInitialGuessSupportsMultiDataset READ manualInitialGuessSupportsMultiDataset CONSTANT)
  /// 0 = manual correspondences, 1 = feature match stub.
  Q_PROPERTY(int initialAlignmentMethod READ initialAlignmentMethod WRITE setInitialAlignmentMethod
                 NOTIFY initialAlignmentMethodChanged)
  /// False while a worker thread holds the pipeline (disables picking).
  Q_PROPERTY(bool pickingAllowed READ pickingAllowed NOTIFY busyWorkingChanged)
  /// Scaled camera image for the correspondence / preview pane.
  Q_PROPERTY(QImage cameraPreviewImage READ cameraPreviewImage NOTIFY cameraPreviewChanged)

public:
  explicit AppController(QObject* parent = nullptr);
  ~AppController() override;

  // --- Status and session paths (bound from Main.qml) ---
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
  std::vector<QPointF> pendingImageMaskPolygonNativePoints() const;
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
  bool canChangeActiveCalibrationDataset() const;
  bool maskEditorOpen() const { return mask_editor_open_; }
  bool imageMaskEditorOpen() const { return image_mask_editor_open_; }
  bool imageMaskPolygonPending() const { return image_mask_polygon_pending_; }
  QString imageMaskPath() const;
  QByteArray imageMaskPreview() const;
  int imageMaskWidth() const;
  int imageMaskHeight() const;
  int maskPreviewDatasetIndex() const { return mask_preview_dataset_index_; }
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

  // --- Property setters invoked from QML bindings ---
  void setScanDuration(double seconds);
  void setTimeOrigin(int origin);
  void setTimeUnit(int unit);
  void setSessionDirectory(const QString& directory);
  void setImagePath(const QString& path);
  void setPcdDirectory(const QString& path);
  void setIntrinsicsPath(const QString& path);

  // --- User actions (buttons, menus, file dialogs); callable from QML as controller.foo() ---
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
  /// Live colormap preview while dragging the viewer slider (no session write).
  Q_INVOKABLE void previewViewerColormapRange(double min_v, double max_v);
  Q_INVOKABLE void commitViewerColormapRange(double min_v, double max_v);
  Q_INVOKABLE void previewRangeColormapRange(double min_v, double max_v);
  Q_INVOKABLE void commitRangeColormapRange(double min_v, double max_v);
  Q_INVOKABLE QString exportFusedPcd();
  /// Called from RangeImageItem.polygonCommitted with native range-image vertices.
  Q_INVOKABLE void commitMaskPolygon(const QVariantList& nativeVertices);
  void commitMaskPolygonNative(const std::vector<QPointF>& nativeVertices);
  void commitImageMaskPolygonNative(const std::vector<QPointF>& nativeVertices);
  Q_INVOKABLE void applyPendingMaskKeep();
  Q_INVOKABLE void applyPendingMaskRemove();
  Q_INVOKABLE void discardPendingMaskPolygon();
  Q_INVOKABLE void applyPendingImageMaskKeep();
  Q_INVOKABLE void applyPendingImageMaskRemove();
  Q_INVOKABLE void discardPendingImageMaskPolygon();
#ifdef CLC_APP_TESTING
  Q_INVOKABLE void installSyntheticMaskEditorScan(int native_width, int native_height, double y_stretch);
  Q_INVOKABLE void setReferenceScanSizeForTest(int width, int height);
#endif
  Q_INVOKABLE void clearMask();
  Q_INVOKABLE bool saveMask();
  Q_INVOKABLE bool loadMask();
  Q_INVOKABLE QString browseMaskFile();
  Q_INVOKABLE void clearImageMask();
  Q_INVOKABLE bool saveImageMask();
  Q_INVOKABLE bool loadImageMask();
  Q_INVOKABLE QString browseImageMaskFile();
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
  /// Called from MaskEditorWindow when shown or closed (modal lifecycle).
  Q_INVOKABLE void setMaskEditorOpen(bool open);
  Q_INVOKABLE void setImageMaskEditorOpen(bool open);
  Q_INVOKABLE void runFusion();
  Q_INVOKABLE void runAlignment();
  Q_INVOKABLE void cancelWorker();
  /// PnP initial T_camera_lidar from correspondenceTable and gizmo pose.
  Q_INVOKABLE void estimateInitialGuessFromCorrespondences();
  Q_INVOKABLE void clearCorrespondences();
  Q_INVOKABLE void pickImageCorrespondence(double u, double v);
  Q_INVOKABLE QString exportInspectionColoredPcd();
  /// Pass the PointCloudScene QML object; connects pointPicked and poseChanged.
  Q_INVOKABLE void setViewer(QObject* viewer);
  Q_INVOKABLE void refreshPreviewInViewer();
  Q_INVOKABLE void uploadFusedCloudToViewer();

  // --- NOTIFY signals: emitted when the matching Q_PROPERTY may have changed ---
signals:
  void statusMessageChanged();
  void busyWorkingChanged();
  void pointCountChanged();
  void scanTimeChanged();
  void sessionPathsChanged();
  void pcdCatalogChanged();
  /// Emitted after loadSession succeeds; QML can refresh time-unit controls.
  void sessionLoaded();
  void rangeImageChanged();
  void rangeYStretchChanged();
  void maskChanged();
  void maskPolygonPendingChanged();
  void imageMaskChanged();
  void imageMaskPolygonPendingChanged();
  void imageMaskEditorOpenChanged();
  void viewerColormapChanged();
  void fusionParamsChanged();
  void calibrationParamsChanged();
  /// Background fusion completed; show message dialog if needed.
  void fusionFinished(bool ok, const QString& message);
  void alignmentFinished(bool ok, const QString& message);
  void inspectionModeChanged();
  void mainWorkTabChanged();
  void calibrationDatasetsChanged();
  void activeDatasetSelectionLockChanged();
  void maskEditorOpenChanged();
  void maskPreviewDatasetIndexChanged();
  void initialAlignmentMethodChanged();
  void correspondencesChanged();
  void cameraPreviewChanged();

  // --- Slots: C++ connections from psiviz viewer and QTimer debouncers ---
private slots:
  /// Wired to PointCloudItem::pointPicked for correspondence workflow.
  void onPointPicked(double x, double y, double z);
  /// Wired to PointCloudItem::poseChanged; writes gizmo pose into session.
  void onViewerPoseChanged();
  void onViewerColormapDebounce();
  void onRangeColormapDebounce();

private:
  // --- Internal helpers (session sync, viewer uploads, worker thread) ---
  void setStatus(const QString& message);  /// Update status bar text and notify QML.
  void setPointCount(int count);           /// Update displayed point count in the viewer pane.
  void refreshPcdCatalog();                /// Rescan PCD paths for the active dataset.
  /// Re-scan cloud_directory when the in-memory catalog was cleared (e.g. after fusion).
  bool repopulateActiveDatasetPcdCatalogIfEmpty();
  void refreshRangeImage();                /// Rebuild range_image_ for mask editor or active dataset.
  void refreshRangeImageForDataset(int calibration_dataset_index);
  bool loadFirstReadableScanForDataset(int calibration_dataset_index,
                                       clc::PointXYZITCloud& cloud,
                                       std::string& err);
  void uploadPreviewScanToViewer(bool preserve_view_state = false);
  /// First readable scan for the active catalog when no fused cloud is available.
  void uploadActiveDatasetPreviewScanToViewer();
  void uploadDenseCloudToViewer(const glim_cloud_fusion::DensePointCloud& cloud, bool preserve_view_state = false);
  void uploadLidarScanToViewer(const clc::PointXYZITCloud& cloud, bool preserve_view_state = false);
  void refreshPreviewCloudFromMask(bool preserve_view_state = true);  /// Reload scan after mask edits.
  void refreshCameraPreview();           /// Load and scale the active dataset camera image for QML.
  clc::DatasetEntry& activeDatasetEntry();
  const clc::DatasetEntry& activeDatasetEntry() const;
  void ensureActiveCalibrationDataset(); /// Create placeholder row if the list is empty.
  bool loadFirstReadableScan(clc::PointXYZITCloud& cloud, std::string& err);
  void rebuildMaskFromOps();             /// Rasterize mask_ops_ into global_mask_pixels_.
  int maskRasterWidth() const;           /// Mask width (session reference or range image).
  int maskRasterHeight() const;
  void ensureImageMaskStorage();
  void rebuildImageMaskFromOps(int dataset_index);
  /// Width and height of the dataset's calibration image on disk.
  bool imageDimensionsForDataset(int dataset_index, int& width, int& height) const;
  /// Reload in-memory image mask ops/pixels for one dataset from session metadata and disk.
  void reloadImageMaskForDataset(int dataset_index);
  /// Raster size for a dataset's image mask (matches the calibration image).
  bool imageMaskRasterSizeForDataset(int dataset_index, int& width, int& height) const;
  /// Write per-dataset image mask PNGs into the session directory before session.json.
  void persistCalibrationImageMasksToSessionDir(const QString& dir);
  /// Write fused dense PCDs into the session directory and update dataset fused_cloud_path.
  void persistFusedCloudsToSessionDir(const QString& dir);
  int imageMaskRasterWidth() const;
  int imageMaskRasterHeight() const;
  bool resolveImageMaskForAlignment(
    int dataset_index,
    const clc::DatasetEntry& ds,
    int image_cols,
    int image_rows,
    std::vector<std::uint8_t>& mask_out,
    std::string& error);
  void setImageMaskPolygonPending(bool pending);
  void setMaskPolygonPending(bool pending);
  void refreshViewerColormapBounds(bool reset_colormap_range = true);
  void refreshRangeImageDisplay();       /// JET-color range image into range_image_display_.
  void applyViewerScalarRangeToItem();   /// Push colormap min/max to psiviz item.
  void applySessionToViewer();           /// Sync viewer camera pose from inverse(T_camera_lidar).
  void refreshViewerAnnotationState();        /// Enable/disable coordinate systems and frustum
  void loadActiveDatasetCloudToViewer();
  /// True when fused_cloud_ in memory belongs to the given calibration dataset index.
  bool inMemoryFusedCloudForDataset(int dataset_index) const;
  /// Image/LiDAR paths and geometry valid for calibration.
  bool calibrationDatasetBaseFieldsOk(const clc::DatasetEntry& dataset) const;
  /// Fused or single-scan cloud available on disk or in the fusion cache.
  bool calibrationDatasetHasUsableCloud(int dataset_index) const;
  void onFusedCloudDiskLoadFailed(clc::DatasetEntry& dataset, const QString& path);
  /// Clear in-memory fusion result (calibration or inspection).
  void clearFusedCloudCache();
  /// Drop all per-dataset calibration fused clouds kept only in RAM.
  void clearCalibrationFusedCloudMemoryCache();
  /// Copy the active calibration fused cloud into the per-dataset RAM cache before evicting it.
  void stashActiveCalibrationFusedCloudInMemoryCache();
  /// Load a calibration dataset fused cloud from RAM cache into fused_cloud_.
  bool restoreCalibrationFusedCloudFromMemoryCache(int dataset_index);
  /// True when fused_cloud_ matches the dataset context of the current main tab.
  bool fusedCloudBelongsToActiveContext() const;
  void syncActiveDatasetFusionToSession();
  void emitSessionUiState();             /// Emit all path/catalog related signals.
  void syncColorModelFromSession();      /// Reload PointColorModel from intrinsics + image.
  bool ensureProjectionImageForActiveDataset();
  void invalidateProjectionImageCache();
  void clearCalibrationDatasetVisuals(); /// Clear viewer cloud when switching datasets.
  /// Remove point cloud geometry from the 3D viewer (keeps session / fused cache).
  void clearViewerPointCloud();
  void resetSessionStructuredScanState();
  void refreshActiveDatasetVisuals();    /// Range image, viewer cloud, correspondences, preview.
  void reprojectViewerCameraRgb();       /// Re-run color_model_ projection for viewer_rgb_.
  /// Copy viewer buffers into a dense cloud + RGB suitable for PCD export.
  bool buildViewerExportCloud(glim_cloud_fusion::DensePointCloud& cloud_out,
                              std::vector<float>& rgb_out) const;
  void syncCorrespondenceMarkers();      /// Orange spheres at 3D correspondence points.
  void finalizePendingCorrespondenceIfReady();  /// Add pair when both 2D and 3D are picked.
  bool buildAlignmentInput(clc::AlignmentInput& input, std::string& error);
  void joinWorker();
  void setWorkerRunning(bool running);

  clc::SessionData session_;                              /// Loaded session.json state.
  clc::DatasetEntry empty_calibration_dataset_scratch_;   /// Fallback when no datasets exist.
  clc::PointColorModel color_model_;                      /// Camera RGB projection for viewer tint.
  cv::Mat projection_image_;                              /// BGR image used for color_model_.
  std::string projection_image_path_;                     /// Path of projection_image_ (cache key).
  std::optional<int> fused_cloud_dataset_index_;          /// Dataset index that owns fused_cloud_.
  bool fused_cloud_is_inspection_ = false;                /// fused_cloud_ belongs to inspection_datasets.
  /// Fused clouds per calibration dataset index (survives active-dataset switches).
  std::unordered_map<int, glim_cloud_fusion::DensePointCloud>
      calibration_fused_cloud_memory_cache_;
  QVector<float> viewer_positions_;                       /// Last cloud uploaded to psiviz (xyz).
  QVector<float> viewer_scalars_;                         /// Per-point scalar channel for colormap.
  QVector<float> viewer_rgb_;                             /// Per-point camera RGB for blending.
  std::vector<std::uint8_t> global_mask_pixels_;          /// Raster mask (255 = keep point).
  std::vector<clc::MaskPolygonOp> mask_ops_;              /// History of polygon mask edits.
  clc::Polygon2f pending_mask_polygon_;                   /// Polygon awaiting keep/remove.
  bool mask_polygon_pending_ = false;
  clc::RangeImage range_image_;                           /// Raw range/intensity from active scan.
  QImage range_image_display_;                            /// JET-colored QImage for QML preview.
  double range_colormap_min_ = 0.0;
  double range_colormap_max_ = 50.0;
  int range_channel_ = 0;                                 /// 0 = range, 1 = intensity.
  double range_y_stretch_ = 2.0;                          /// Vertical scale for mask editor UI.
  bool preview_upload_pending_ = false;
  bool viewer_scalar_bounds_valid_ = false;
  double viewer_colormap_min_ = 0.0;
  double viewer_colormap_max_ = 1.0;
  double viewer_data_min_ = 0.0;                          /// Auto min of active scalar channel.
  double viewer_data_max_ = 1.0;                          /// Auto max of active scalar channel.
  double range_data_min_ = 0.0;
  double range_data_max_ = 50.0;
  int viewer_scalar_channel_ = 0;
  QTimer viewer_colormap_debounce_;                       /// Delays heavy viewer refresh on slider drag.
  QTimer range_colormap_debounce_;
  glim_cloud_fusion::DensePointCloud fused_cloud_;        /// Result of last fusion for active dataset.
  QString status_message_ = QStringLiteral("Ready");
  QString session_directory_;                             /// Folder containing session.json.
  int point_count_ = 0;                                   /// Points shown in viewer (status bar).
  bool fusion_blocked_ = false;                           /// True when catalog is invalid for fusion.
  int main_work_tab_ = 0;                                 /// 0=calibration tabs, 3=inspection.
  int active_calibration_dataset_index_ = 0;
  int mask_preview_dataset_index_ = 0;                    /// Range-image source while mask editor is open.
  bool mask_editor_open_ = false;
  bool image_mask_editor_open_ = false;
  bool image_mask_polygon_pending_ = false;
  clc::Polygon2f pending_image_mask_polygon_;
  std::vector<std::vector<clc::MaskPolygonOp>> image_mask_ops_by_dataset_;
  std::vector<std::vector<std::uint8_t>> image_mask_pixels_by_dataset_;
  int initial_alignment_method_ = 0;
  int selected_correspondence_row_ = -1;
  std::optional<clc::Vec3d> pending_correspondence_3d_;   /// Lidar pick waiting for image click.
  std::optional<std::pair<double, double>> pending_correspondence_2d_;  /// Image click waiting for 3D.
  QImage camera_preview_;                                 /// Thumbnail of active dataset image.
  std::atomic<bool> worker_running_{false};                 /// Fusion or alignment thread active.
  std::atomic<bool> worker_cancel_requested_{false};
  std::unique_ptr<std::thread> worker_;
  class QObject* viewer_{nullptr};                          /// psiviz::PointCloudItem instance from QML.
};
