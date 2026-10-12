# psiviz

Small Qt Quick / QRhi library for interactive point cloud viewing and in-scene pose manipulation. It is independent of camera–lidar calibration logic and lives inside the `camera_lidar_calibration` repository (may be factored out later).

## Features (v1)

- Large point clouds (QRhi point sprites, scalar JET colormap, optional per-point RGB blend)
- PCL `pcd_viewer`-style trackball navigation
- Depth-buffer picking (Retina-aware)
- Translate / rotate pose gizmo (no ImGui)

## Build

Built automatically when `BUILD_CLC_APP=ON` (default). Requires Qt 6.5+ with Quick and ShaderTools.

```bash
cmake -B build -DBUILD_CLC_APP=ON
cmake --build build --target psiviz
```

## QML usage

```qml
import Psiviz 1.0

PointCloudScene {
    anchors.fill: parent
    onPointPicked: (x, y, z) => console.log(x, y, z)
}
```

See `docs/API.md` for C++ types and threading notes.
