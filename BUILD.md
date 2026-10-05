# Building on macOS

Libraries install into **`~/.local/calib-deps`** by default (override with `CALIB_PREFIX`). Homebrew is only used for build tools and binary packages (Qt, PCL, OpenCV, fmt, spdlog); GTSAM, gtsam_points, GLIM, and vlcal_align are built from source into the prefix.

## Layout

Clone forks as siblings under one directory (default `~/source_builds`):

```
~/source_builds/
  gtsam/                          # upstream borglab/gtsam @ 4.3a1 (cloned by install script)
  gtsam_points/                   # themightyoarfish/gtsam_points
  glim/                           # themightyoarfish/glim
  direct_visual_lidar_calibration/  # themightyoarfish/direct_visual_lidar_calibration
  camera_lidar_calibration/       # themightyoarfish/camera_lidar_calibration
```

All feature branches: **`feature/camera-lidar-calibration-libs`**

## Step 1 — dependencies (one command)

```bash
curl -fsSL https://raw.githubusercontent.com/themightyoarfish/camera_lidar_calibration/feature/camera-lidar-calibration-libs/scripts/install-deps-macos.sh | bash
```

Or from a local checkout:

```bash
CALIB_SRC=~/source_builds CALIB_PREFIX=~/.local/calib-deps \
  bash camera_lidar_calibration/scripts/install-deps-macos.sh
```

## Step 2 — app shell

Uses sibling source trees from step 1 (`CLC_DEV_BUILD=ON`) and the isolated prefix for GTSAM / gtsam_points.

```bash
CALIB_SRC=~/source_builds CALIB_PREFIX=~/.local/calib-deps \
  bash camera_lidar_calibration/scripts/build-app-macos.sh
```

Run:

```bash
~/source_builds/camera_lidar_calibration/build/clc_app/clc_app
```

## Optional: Conan-isolated prefix

For a fully Conan-managed prefix (no cmake install to `~/.local`):

```bash
export CONAN_HOME="$HOME/.calib-conan-prefix"
export CALIB_SRC=~/source_builds   # must contain gtsam, gtsam_points, glim, direct_visual_lidar_calibration siblings
bash camera_lidar_calibration/scripts/conan-prefix.sh
```

On macOS, ensure `conan/profiles/native-macos` is used (auto-selected when `uname` is Darwin).

## Forks / branches

| Project | Upstream | Fork | Branch |
|---------|----------|------|--------|
| gtsam_points | koide3/gtsam_points | themightyoarfish/gtsam_points | feature/camera-lidar-calibration-libs |
| glim | koide3/glim | themightyoarfish/glim | feature/camera-lidar-calibration-libs |
| direct_visual_lidar_calibration | koide3/direct_visual_lidar_calibration | themightyoarfish/direct_visual_lidar_calibration | feature/camera-lidar-calibration-libs |
| camera_lidar_calibration | (new) | themightyoarfish/camera_lidar_calibration | feature/camera-lidar-calibration-libs |
| gtsam | borglab/gtsam | upstream tag `4.3a1` only | — |

Changes not forked: GTSAM uses upstream `4.3a1`; Conan recipe lives in `camera_lidar_calibration/conan/gtsam/`.
