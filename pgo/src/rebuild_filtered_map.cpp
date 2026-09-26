#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<PointType>;

struct Box
{
  double min_x;
  double max_x;
  double min_y;
  double max_y;
  double min_z;
  double max_z;
};

struct PoseRecord
{
  std::string patch_name;
  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
};

struct Options
{
  std::filesystem::path map_dir;
  std::filesystem::path output_path;
  std::filesystem::path backup_path;
  double map_z_min = 0.05;
  double map_z_max = 3.0;
  bool overwrite = false;
};

void printUsage()
{
  std::cout
      << "Usage:\n"
      << "  ros2 run pgo rebuild_filtered_map --map-dir /path/to/map_dir [options]\n\n"
      << "Options:\n"
      << "  --output <path>       Output PCD path (default: <map-dir>/map.pcd)\n"
      << "  --backup <path>       Backup original map.pcd before overwrite\n"
      << "  --map-z-min <m>       Keep world points above this Z (default: 0.05)\n"
      << "  --map-z-max <m>       Keep world points below this Z (default: 3.0)\n"
      << "  --overwrite           Allow writing over output path\n";
}

std::string requireValue(int &i, int argc, char **argv, const std::string &option)
{
  if (i + 1 >= argc) {
    throw std::invalid_argument("Missing value for option: " + option);
  }
  ++i;
  return argv[i];
}

Options parseOptions(int argc, char **argv)
{
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--help" || arg == "-h") {
      printUsage();
      std::exit(0);
    }
    if (arg == "--map-dir") {
      options.map_dir = requireValue(i, argc, argv, arg);
      continue;
    }
    if (arg == "--output") {
      options.output_path = requireValue(i, argc, argv, arg);
      continue;
    }
    if (arg == "--backup") {
      options.backup_path = requireValue(i, argc, argv, arg);
      continue;
    }
    if (arg == "--map-z-min") {
      options.map_z_min = std::stod(requireValue(i, argc, argv, arg));
      continue;
    }
    if (arg == "--map-z-max") {
      options.map_z_max = std::stod(requireValue(i, argc, argv, arg));
      continue;
    }
    if (arg == "--overwrite") {
      options.overwrite = true;
      continue;
    }
    throw std::invalid_argument("Unknown option: " + arg);
  }

  if (options.map_dir.empty()) {
    throw std::invalid_argument("--map-dir is required");
  }
  if (options.output_path.empty()) {
    options.output_path = options.map_dir / "map.pcd";
  }
  if (options.map_z_min >= options.map_z_max) {
    throw std::invalid_argument("--map-z-min must be < --map-z-max");
  }
  return options;
}

std::vector<PoseRecord> loadPoses(const std::filesystem::path &poses_path)
{
  std::ifstream ifs(poses_path);
  if (!ifs.is_open()) {
    throw std::runtime_error("Failed to open poses file: " + poses_path.string());
  }

  std::vector<PoseRecord> poses;
  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::istringstream iss(line);
    PoseRecord pose;
    double q_w = 1.0;
    double q_x = 0.0;
    double q_y = 0.0;
    double q_z = 0.0;
    if (!(iss >> pose.patch_name >> pose.translation.x() >> pose.translation.y() >> pose.translation.z() >>
          q_w >> q_x >> q_y >> q_z)) {
      continue;
    }

    pose.rotation = Eigen::Quaterniond(q_w, q_x, q_y, q_z).normalized();
    poses.push_back(pose);
  }

  if (poses.empty()) {
    throw std::runtime_error("No valid poses in: " + poses_path.string());
  }
  return poses;
}

bool pointInBox(const PointType &point, const Box &box)
{
  return point.x >= box.min_x && point.x <= box.max_x &&
         point.y >= box.min_y && point.y <= box.max_y &&
         point.z >= box.min_z && point.z <= box.max_z;
}

bool pointInAnyBox(const PointType &point, const std::vector<Box> &boxes)
{
  for (const auto &box : boxes) {
    if (pointInBox(point, box)) {
      return true;
    }
  }
  return false;
}

CloudType::Ptr filterBodyCloud(const CloudType::Ptr &cloud, const std::vector<Box> &boxes)
{
  CloudType::Ptr filtered(new CloudType);
  filtered->reserve(cloud->size());
  for (const auto &point : cloud->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    if (!pointInAnyBox(point, boxes)) {
      filtered->push_back(point);
    }
  }
  filtered->width = filtered->points.size();
  filtered->height = 1;
  filtered->is_dense = false;
  return filtered;
}

int main(int argc, char **argv)
{
  try {
    const Options options = parseOptions(argc, argv);
    const auto poses_path = options.map_dir / "poses.txt";
    const auto patches_dir = options.map_dir / "patches";

    if (!std::filesystem::is_directory(options.map_dir)) {
      throw std::runtime_error("Map directory does not exist: " + options.map_dir.string());
    }
    if (!std::filesystem::is_directory(patches_dir)) {
      throw std::runtime_error("Patches directory does not exist: " + patches_dir.string());
    }
    if (std::filesystem::exists(options.output_path) && !options.overwrite) {
      throw std::runtime_error("Output exists; pass --overwrite: " + options.output_path.string());
    }

    const std::vector<Box> self_filter_boxes{
        {-0.45, 0.45, -0.44, 0.44, -0.30, 0.45},
        {-0.22, 0.22, -0.22, 0.22, 0.35, 1.20},
        {-0.65, 0.65, -0.55, 0.55, 1.05, 1.85},
    };

    const auto poses = loadPoses(poses_path);
    CloudType::Ptr map_cloud(new CloudType);
    size_t input_points = 0;
    size_t body_filtered_points = 0;
    size_t world_z_filtered_points = 0;

    for (const auto &pose : poses) {
      const auto patch_path = patches_dir / pose.patch_name;
      CloudType::Ptr patch(new CloudType);
      if (pcl::io::loadPCDFile<PointType>(patch_path.string(), *patch) != 0) {
        throw std::runtime_error("Failed to load patch: " + patch_path.string());
      }
      input_points += patch->size();

      CloudType::Ptr body_filtered = filterBodyCloud(patch, self_filter_boxes);
      body_filtered_points += patch->size() - body_filtered->size();

      CloudType::Ptr world_cloud(new CloudType);
      pcl::transformPointCloud(*body_filtered, *world_cloud, pose.translation, pose.rotation);

      for (const auto &point : world_cloud->points) {
        if (point.z >= options.map_z_min && point.z <= options.map_z_max) {
          map_cloud->push_back(point);
        } else {
          ++world_z_filtered_points;
        }
      }
    }

    map_cloud->width = map_cloud->points.size();
    map_cloud->height = 1;
    map_cloud->is_dense = false;

    if (!options.backup_path.empty() && std::filesystem::exists(options.output_path)) {
      std::filesystem::copy_file(
          options.output_path, options.backup_path,
          std::filesystem::copy_options::overwrite_existing);
    }

    if (pcl::io::savePCDFileBinary(options.output_path.string(), *map_cloud) != 0) {
      throw std::runtime_error("Failed to save output: " + options.output_path.string());
    }

    std::cout << "Rebuilt map from " << poses.size() << " patches\n";
    std::cout << "Input points: " << input_points << "\n";
    std::cout << "Removed by self filter: " << body_filtered_points << "\n";
    std::cout << "Removed by world Z filter: " << world_z_filtered_points << "\n";
    std::cout << "Output points: " << map_cloud->size() << "\n";
    std::cout << "Output: " << options.output_path << "\n";
    if (!options.backup_path.empty()) {
      std::cout << "Backup: " << options.backup_path << "\n";
    }
  } catch (const std::exception &e) {
    std::cerr << "[rebuild_filtered_map] " << e.what() << "\n";
    return 1;
  }

  return 0;
}
