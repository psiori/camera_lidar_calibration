# Building on macOS

All third-party and in-house libraries are managed with **Conan 2**. Packages are built from the **psiori** forks and cached in `~/.calib-conan` (override with `CONAN_HOME`). No Homebrew packages are required beyond what Conan pulls from conan-center (Qt, OpenCV, PCL, Boost, Eigen, Ceres, etc.).

System requirements: Xcode Command Line Tools (or a C++17 compiler), `git`, and Python 3 for installing Conan.

## Layout

Clone repositories as siblings under one directory (default `~/source_builds`):

```
~/source_builds/
  gtsam/                            # psiori/gtsam
  gtsam_points/                     # psiori/gtsam_points
  glim/                             # psiori/glim
  direct_visual_lidar_calibration/  # psiori/direct_visual_lidar_calibration
  camera_lidar_calibration/         # psiori/camera_lidar_calibration
```

All feature branches: **`feature/camera-lidar-calibration-libs`**

### Clone (once)

```bash
export CALIB_SRC=~/source_builds
export GITHUB_ORG=psiori
export CALIB_BRANCH=feature/camera-lidar-calibration-libs

mkdir -p "${CALIB_SRC}"
for repo in gtsam gtsam_points glim direct_visual_lidar_calibration camera_lidar_calibration; do
  git clone --branch "${CALIB_BRANCH}" \
    "git@github.com:${GITHUB_ORG}/${repo}.git" \
    "${CALIB_SRC}/${repo}"
done
```

`install-deps.sh` will clone any missing sibling repos automatically.

## Step 1 — Conan dependencies

Builds and caches `gtsam`, `gtsam_points`, `glim`, and `vlcal_align` from local checkouts:

```bash
CALIB_SRC=~/source_builds CONAN_HOME=~/.calib-conan \
  bash ~/source_builds/camera_lidar_calibration/scripts/install-deps.sh
```

Conan is installed via `pip3` if not already on `PATH`.

Each library ships a `conanfile.py` in its repository root. Dependency order:

1. `gtsam` (psiori fork of borglab/gtsam @ 4.3a1, with Conan recipe)
2. `gtsam_points`
3. `glim` (includes `glim_cloud_fusion`)
4. `vlcal_align` (`direct_visual_lidar_calibration`)

## Step 2 — app shell

```bash
CALIB_SRC=~/source_builds CONAN_HOME=~/.calib-conan \
  bash ~/source_builds/camera_lidar_calibration/scripts/build-app.sh
```

This runs `conan install` for `camera_lidar_calibration` (pulling Qt, PCL, OpenCV, etc. from conan-center) and builds `clc_app` against the cached packages.

Run:

```bash
~/source_builds/camera_lidar_calibration/build/clc_app/clc_app
```

## Docker / CI gate

The Ubuntu 24.04 Dockerfile uses the same Conan workflow:

```bash
bash camera_lidar_calibration/scripts/conan-prefix.sh
```

## Repositories / branches

| Project | Upstream | psiori repo | Branch |
|---------|----------|-------------|--------|
| gtsam | borglab/gtsam | psiori/gtsam | feature/camera-lidar-calibration-libs |
| gtsam_points | koide3/gtsam_points | psiori/gtsam_points | feature/camera-lidar-calibration-libs |
| glim | koide3/glim | psiori/glim | feature/camera-lidar-calibration-libs |
| direct_visual_lidar_calibration | koide3/direct_visual_lidar_calibration | psiori/direct_visual_lidar_calibration | feature/camera-lidar-calibration-libs |
| camera_lidar_calibration | (new) | psiori/camera_lidar_calibration | feature/camera-lidar-calibration-libs |

Every psiori repo includes a root `conanfile.py`. Profiles with `-march=native` live in `camera_lidar_calibration/conan/profiles/`.
