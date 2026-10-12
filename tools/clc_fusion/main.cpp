#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <glim/util/config.hpp>

#include <clc_core/fusion_service.hpp>
#include <clc_core/pcd_io.hpp>
#include <clc_core/session.hpp>

namespace {

using Clock = std::chrono::steady_clock;

std::string format_duration(double seconds) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2) << seconds << "s";
  return oss.str();
}

void print_usage(const char* prog) {
  std::cerr
    << "Usage: " << prog << " <pcd_directory> [options]\n"
    << "  Fuses sorted PCD scans using the same pipeline as clc_app (glim_cloud_fusion).\n\n"
    << "Options:\n"
    << "  --threads N                 Fusion worker threads (default: CPU count)\n"
    << "  --max-correspondence-dist M Max CT-GICP correspondence distance in meters (default: 0.5)\n"
    << "  --voxel-resolution M        Output voxel size in meters (default: 0.05)\n"
    << "  --time-unit seconds|nanoseconds (default: nanoseconds)\n"
    << "  --time-origin scan_start|first_point (default: scan_start)\n"
    << "  --scan-duration S           Sweep duration in seconds (inferred from first scan if omitted)\n"
    << "  --max-scans N               Only process first N files\n"
    << "  --output PATH               Write fused cloud as binary PCD\n"
    << "  --sort lex|timestamp        File order: lexicographic path sort or autocrane filename time\n"
    << "  --glim-config DIR           glim config directory (like pcd_mapper; silences missing /config.json)\n"
    << "  -h, --help                  Show this help\n";
}

bool parse_autocrane_timestamp(const std::string& filename, double& stamp_seconds) {
  int y = 0;
  unsigned m = 0, d = 0, h = 0, mi = 0, s = 0, ms = 0;
  if (7 != std::sscanf(filename.c_str(), "%d-%u-%uT%u_%u_%u_%3uZ.pcd", &y, &m, &d, &h, &mi, &s, &ms)) {
    return false;
  }
  std::tm calendar_time{};
  calendar_time.tm_sec = s;
  calendar_time.tm_min = mi;
  calendar_time.tm_hour = h;
  calendar_time.tm_mday = d;
  calendar_time.tm_mon = m - 1;
  calendar_time.tm_year = y - 1900;
  calendar_time.tm_isdst = 0;
#if defined(_WIN32)
  const std::time_t utc_time = _mkgmtime(&calendar_time);
#else
  const std::time_t utc_time = timegm(&calendar_time);
#endif
  const auto ms_tp = std::chrono::milliseconds(ms) + std::chrono::system_clock::from_time_t(utc_time);
  stamp_seconds =
    std::chrono::duration_cast<std::chrono::duration<double>>(ms_tp.time_since_epoch()).count();
  return true;
}

std::vector<std::string> sort_pcd_paths(const std::vector<std::string>& paths, const std::string& mode) {
  if (mode != "timestamp") {
    return paths;
  }
  std::vector<std::pair<double, std::string>> keyed;
  keyed.reserve(paths.size());
  for (const auto& path : paths) {
    double stamp = 0.0;
    const auto name = std::filesystem::path(path).filename().string();
    if (!parse_autocrane_timestamp(name, stamp)) {
      std::cerr << "Warning: cannot parse timestamp from " << name << ", using lex order key\n";
      stamp = static_cast<double>(keyed.size());
    }
    keyed.emplace_back(stamp, path);
  }
  std::sort(keyed.begin(), keyed.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
  std::vector<std::string> sorted;
  sorted.reserve(keyed.size());
  for (const auto& [_, path] : keyed) {
    sorted.push_back(path);
  }
  return sorted;
}

glim_cloud_fusion::LidarScan to_fusion_scan(const clc::PointXYZITCloud& cloud, double stamp) {
  glim_cloud_fusion::LidarScan scan;
  scan.stamp = stamp;
  scan.width = cloud.width;
  scan.height = cloud.height;
  scan.points = cloud.scan.points;
  scan.times = cloud.scan.times;
  scan.intensities = cloud.scan.intensities;
  scan.keep = cloud.scan.keep;
  return scan;
}

struct Options {
  std::string directory;
  std::optional<int> threads;
  std::optional<double> max_correspondence_distance;
  std::optional<double> voxel_resolution;
  std::optional<double> scan_duration;
  std::optional<int> max_scans;
  std::string output_path;
  std::string sort_mode = "lex";
  std::string glim_config_dir;
  glim_cloud_fusion::TimeUnit time_unit = glim_cloud_fusion::TimeUnit::Nanoseconds;
  glim_cloud_fusion::TimeOrigin time_origin = glim_cloud_fusion::TimeOrigin::ScanStart;
};

std::optional<Options> parse_args(int argc, char** argv) {
  if (argc < 2) {
    return std::nullopt;
  }
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      print_usage(argv[0]);
      return std::nullopt;
    }
    if (arg == "--threads" && i + 1 < argc) {
      opt.threads = std::stoi(argv[++i]);
      continue;
    }
    if (arg == "--max-correspondence-dist" && i + 1 < argc) {
      opt.max_correspondence_distance = std::stod(argv[++i]);
      continue;
    }
    if (arg == "--voxel-resolution" && i + 1 < argc) {
      opt.voxel_resolution = std::stod(argv[++i]);
      continue;
    }
    if (arg == "--scan-duration" && i + 1 < argc) {
      opt.scan_duration = std::stod(argv[++i]);
      continue;
    }
    if (arg == "--max-scans" && i + 1 < argc) {
      opt.max_scans = std::stoi(argv[++i]);
      continue;
    }
    if (arg == "--output" && i + 1 < argc) {
      opt.output_path = argv[++i];
      continue;
    }
    if (arg == "--sort" && i + 1 < argc) {
      opt.sort_mode = argv[++i];
      continue;
    }
    if (arg == "--glim-config" && i + 1 < argc) {
      opt.glim_config_dir = argv[++i];
      continue;
    }
    if (arg == "--time-unit" && i + 1 < argc) {
      const std::string v = argv[++i];
      if (v == "seconds") {
        opt.time_unit = glim_cloud_fusion::TimeUnit::Seconds;
      } else if (v == "nanoseconds") {
        opt.time_unit = glim_cloud_fusion::TimeUnit::Nanoseconds;
      } else {
        std::cerr << "Unknown --time-unit: " << v << "\n";
        return std::nullopt;
      }
      continue;
    }
    if (arg == "--time-origin" && i + 1 < argc) {
      const std::string v = argv[++i];
      if (v == "scan_start") {
        opt.time_origin = glim_cloud_fusion::TimeOrigin::ScanStart;
      } else if (v == "first_point") {
        opt.time_origin = glim_cloud_fusion::TimeOrigin::FirstPoint;
      } else {
        std::cerr << "Unknown --time-origin: " << v << "\n";
        return std::nullopt;
      }
      continue;
    }
    if (!arg.empty() && arg[0] == '-') {
      std::cerr << "Unknown option: " << arg << "\n";
      return std::nullopt;
    }
    if (opt.directory.empty()) {
      opt.directory = arg;
    } else {
      std::cerr << "Unexpected extra argument: " << arg << "\n";
      return std::nullopt;
    }
  }
  if (opt.directory.empty()) {
    return std::nullopt;
  }
  return opt;
}

}  // namespace

int main(int argc, char** argv) {
  const auto parsed = parse_args(argc, argv);
  if (!parsed) {
    if (argc < 2) {
      print_usage(argv[0]);
    }
    return argc < 2 ? 1 : 2;
  }
  const Options& opt = *parsed;

  if (!opt.glim_config_dir.empty()) {
    std::filesystem::path config_path(opt.glim_config_dir);
    if (!config_path.is_absolute()) {
      config_path = std::filesystem::absolute(config_path);
    }
    glim::GlobalConfig::instance(config_path.string(), true);
    std::cout << "glim config: " << config_path << "\n";
  }

  auto paths = clc::list_pcd_files(opt.directory);
  paths = sort_pcd_paths(paths, opt.sort_mode);
  if (paths.empty()) {
    std::cerr << "No PCD files under: " << opt.directory << "\n";
    return 1;
  }
  if (opt.max_scans.has_value() && *opt.max_scans > 0 &&
      static_cast<size_t>(*opt.max_scans) < paths.size()) {
    paths.resize(static_cast<size_t>(*opt.max_scans));
  }

  glim_cloud_fusion::FusionParams params;
  clc::apply_default_fusion_params(params);
  clc::apply_default_scan_time_params(params.scan_time);
  params.scan_time.time_unit = opt.time_unit;
  params.scan_time.time_origin = opt.time_origin;
  if (opt.threads.has_value()) {
    params.num_threads = *opt.threads;
  }
  if (opt.max_correspondence_distance.has_value()) {
    params.max_correspondence_distance = *opt.max_correspondence_distance;
  }
  if (opt.voxel_resolution.has_value()) {
    params.voxel_resolution = *opt.voxel_resolution;
  }
  if (opt.scan_duration.has_value()) {
    params.scan_time.scan_duration = *opt.scan_duration;
  }

  std::cout << "PCD directory: " << opt.directory << "\n";
  std::cout << "Scans: " << paths.size() << "  threads: " << params.num_threads
            << "  max_corr: " << params.max_correspondence_distance
            << "  voxel: " << params.voxel_resolution << "\n";

  std::vector<glim_cloud_fusion::LidarScan> scans;
  scans.reserve(paths.size());

  const auto load_start = Clock::now();
  for (size_t i = 0; i < paths.size(); ++i) {
    clc::PointXYZITCloud cloud;
    std::string err;
    if (!clc::load_pcd_xyzit(paths[i], cloud, err)) {
      std::cerr << "Failed to load " << paths[i] << ": " << err << "\n";
      return 1;
    }
    if (scans.empty() && (!params.scan_time.scan_duration.has_value() ||
                          *params.scan_time.scan_duration <= 0.0)) {
      if (const auto duration = clc::infer_scan_duration_seconds(cloud.scan, params.scan_time)) {
        params.scan_time.scan_duration = *duration;
        std::cout << "Inferred scan_duration: " << *duration << " s\n";
      }
    }

    double stamp = static_cast<double>(i) * 0.1;
    if (opt.sort_mode == "timestamp") {
      parse_autocrane_timestamp(std::filesystem::path(paths[i]).filename().string(), stamp);
    }
    scans.push_back(to_fusion_scan(cloud, stamp));

    if ((i + 1) % 10 == 0 || i + 1 == paths.size()) {
      const double elapsed =
        std::chrono::duration<double>(Clock::now() - load_start).count();
      std::cout << "[load] " << (i + 1) << "/" << paths.size() << " (" << format_duration(elapsed)
                << ")\n";
    }
  }
  const double load_seconds = std::chrono::duration<double>(Clock::now() - load_start).count();
  std::cout << "Load complete: " << format_duration(load_seconds) << "\n";

  const auto fuse_start = Clock::now();
  auto last_progress = fuse_start;

  const auto result = clc::run_fusion(scans, params, [&](const glim_cloud_fusion::FusionProgress& p) {
    const auto now = Clock::now();
    const double total_elapsed = std::chrono::duration<double>(now - fuse_start).count();
    const double since_last = std::chrono::duration<double>(now - last_progress).count();
    last_progress = now;

    const double rate = p.scan_index > 0 ? static_cast<double>(p.scan_index) / total_elapsed : 0.0;
    const double remaining =
      rate > 0.0 ? static_cast<double>(p.total_scans - p.scan_index) / rate : 0.0;

    std::cout << "[fusion] " << p.scan_index << "/" << p.total_scans << "  stage=" << p.stage
              << "  elapsed=" << format_duration(total_elapsed)
              << "  dt=" << format_duration(since_last);
    if (rate > 0.0) {
      std::cout << "  " << std::fixed << std::setprecision(2) << rate << " scans/s"
                << "  eta=" << format_duration(remaining);
    }
    std::cout << "\n";
  });

  const double fuse_seconds = std::chrono::duration<double>(Clock::now() - fuse_start).count();

  if (result.error != glim_cloud_fusion::FusionError::Ok) {
    std::cerr << "Fusion failed: " << result.message << "\n";
    return 1;
  }

  std::cout << "Fusion complete: " << format_duration(fuse_seconds) << "  points="
            << result.cloud.points.size() << "\n";
  std::cout << "Total (load+fuse): " << format_duration(load_seconds + fuse_seconds) << "\n";

  if (!opt.output_path.empty()) {
    std::string err;
    if (!clc::save_dense_pcd(opt.output_path, result.cloud, err)) {
      std::cerr << "Failed to write output: " << err << "\n";
      return 1;
    }
    std::cout << "Wrote " << opt.output_path << "\n";
  }

  return 0;
}
