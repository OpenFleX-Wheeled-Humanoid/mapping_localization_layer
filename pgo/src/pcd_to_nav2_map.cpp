#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<PointType>;

enum class CellState : int8_t
{
    Unknown = -1,
    Free = 0,
    Occupied = 100
};

struct Options
{
    std::string pcd_path;
    std::string out_dir;
    std::string poses_path;
    double resolution = 0.05;
    double z_min = 0.00;
    double z_max = 2.30;
    double obstacle_z_min = 0.10;
    double obstacle_z_max = 1.40;
    double inflation_radius = 0.0;
    double padding = 0.3;
    int obstacle_min_hits = 3;
    int sor_mean_k = 24;           // 0 = disabled
    double sor_stddev_thresh = 1.0;
    int min_obstacle_cluster = 8;  // 0 = disabled; remove occupied blobs < N cells
};

struct Bounds
{
    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;
};

void printUsage()
{
    std::cout
        << "Usage:\n"
        << "  ros2 run pgo pcd_to_nav2_map \\\n"
        << "    --pcd /path/to/map.pcd \\\n"
        << "    --out-dir /path/to/output_dir [options]\n\n"
        << "Options:\n"
        << "  --resolution <m>         Grid resolution (default: 0.05)\n"
        << "  --z-min <m>              Min Z for observed/free projection (default: 0.00)\n"
        << "  --z-max <m>              Max Z for observed/free projection (default: 2.30)\n"
        << "  --obstacle-z-min <m>     Min obstacle Z (default: 0.10)\n"
        << "  --obstacle-z-max <m>     Max obstacle Z (default: 1.40)\n"
        << "  --inflation-radius <m>   Inflate occupied cells (default: 0.0)\n"
        << "  --padding <m>            Extra border around bounds (default: 0.3)\n"
        << "  --obstacle-min-hits <n>  Min hits to mark occupied (default: 3)\n"
        << "  --sor-mean-k <n>        SOR neighbors to analyze (default: 24)\n"
        << "  --sor-stddev <f>        SOR stddev multiplier threshold (default: 1.0)\n"
        << "  --min-obstacle-cluster <n>  Remove occupied blobs < N cells (default: 8)\n"
        << "  --poses <path>             Sensor poses file for raycasting (format: name x y z qw qx qy qz)\n"
        << "  --help                     Show this help\n";
}

std::string requireValue(int &i, int argc, char **argv, const std::string &option)
{
    if (i + 1 >= argc)
    {
        throw std::invalid_argument("Missing value for option: " + option);
    }
    ++i;
    return argv[i];
}

Options parseOptions(int argc, char **argv)
{
    Options options;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg(argv[i]);
        if (arg == "--help" || arg == "-h")
        {
            printUsage();
            std::exit(0);
        }
        if (arg == "--pcd")
        {
            options.pcd_path = requireValue(i, argc, argv, arg);
            continue;
        }
        if (arg == "--out-dir")
        {
            options.out_dir = requireValue(i, argc, argv, arg);
            continue;
        }
        if (arg == "--resolution")
        {
            options.resolution = std::stod(requireValue(i, argc, argv, arg));
            continue;
        }
        if (arg == "--z-min")
        {
            options.z_min = std::stod(requireValue(i, argc, argv, arg));
            continue;
        }
        if (arg == "--z-max")
        {
            options.z_max = std::stod(requireValue(i, argc, argv, arg));
            continue;
        }
        if (arg == "--obstacle-z-min")
        {
            options.obstacle_z_min = std::stod(requireValue(i, argc, argv, arg));
            continue;
        }
        if (arg == "--obstacle-z-max")
        {
            options.obstacle_z_max = std::stod(requireValue(i, argc, argv, arg));
            continue;
        }
        if (arg == "--inflation-radius")
        {
            options.inflation_radius = std::stod(requireValue(i, argc, argv, arg));
            continue;
        }
        if (arg == "--padding")
        {
            options.padding = std::stod(requireValue(i, argc, argv, arg));
            continue;
        }
        if (arg == "--obstacle-min-hits")
        {
            options.obstacle_min_hits = std::stoi(requireValue(i, argc, argv, arg));
            continue;
        }
        if (arg == "--sor-mean-k")
        {
            options.sor_mean_k = std::stoi(requireValue(i, argc, argv, arg));
            continue;
        }
        if (arg == "--sor-stddev")
        {
            options.sor_stddev_thresh = std::stod(requireValue(i, argc, argv, arg));
            continue;
        }
        if (arg == "--min-obstacle-cluster")
        {
            options.min_obstacle_cluster = std::stoi(requireValue(i, argc, argv, arg));
            continue;
        }
        if (arg == "--poses")
        {
            options.poses_path = requireValue(i, argc, argv, arg);
            continue;
        }
        throw std::invalid_argument("Unknown option: " + arg);
    }

    if (options.pcd_path.empty() || options.out_dir.empty())
    {
        throw std::invalid_argument("Both --pcd and --out-dir are required");
    }
    if (options.resolution <= 0.0)
    {
        throw std::invalid_argument("--resolution must be > 0");
    }
    if (options.z_min >= options.z_max)
    {
        throw std::invalid_argument("--z-min must be < --z-max");
    }
    if (options.obstacle_z_min >= options.obstacle_z_max)
    {
        throw std::invalid_argument("--obstacle-z-min must be < --obstacle-z-max");
    }
    if (options.obstacle_min_hits < 1)
    {
        throw std::invalid_argument("--obstacle-min-hits must be >= 1");
    }
    return options;
}

bool loadCloud(const std::string &pcd_path, CloudType::Ptr &cloud)
{
    cloud.reset(new CloudType);
    if (pcl::io::loadPCDFile<PointType>(pcd_path, *cloud) == 0)
    {
        std::vector<int> valid_indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, valid_indices);
        return !cloud->empty();
    }

    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, cloud_xyz) != 0)
    {
        return false;
    }

    cloud->clear();
    cloud->reserve(cloud_xyz.size());
    for (const auto &point : cloud_xyz.points)
    {
        PointType p{};
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        p.intensity = 1.0f;
        cloud->push_back(p);
    }
    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = false;
    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, valid_indices);
    return !cloud->empty();
}

bool isFinite(const PointType &p)
{
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

bool isObservedPoint(const PointType &p, const Options &options)
{
    // Any point within [z_min, z_max] counts as "observed" for that 2D cell,
    // including ceiling and ground points — if we can see a point at (x,y),
    // the cell is at least partially observable.
    return p.z >= options.z_min && p.z <= options.z_max;
}

// A "ceiling" point is high up (above obstacle band) — cells with only ceiling
// points and no obstacle points should be marked free, since the robot can
// pass under them.
bool isCeilingPoint(const PointType &p, const Options &options)
{
    return p.z > options.obstacle_z_max && p.z <= options.z_max;
}

bool isObstaclePoint(const PointType &p, const Options &options)
{
    return p.z >= options.obstacle_z_min && p.z <= options.obstacle_z_max;
}

Bounds computeBounds(const CloudType::Ptr &cloud, const Options &options)
{
    Bounds bounds;
    bounds.min_x = std::numeric_limits<double>::max();
    bounds.min_y = std::numeric_limits<double>::max();
    bounds.max_x = std::numeric_limits<double>::lowest();
    bounds.max_y = std::numeric_limits<double>::lowest();

    for (const auto &point : cloud->points)
    {
        if (!isFinite(point) || !isObservedPoint(point, options))
        {
            continue;
        }
        bounds.min_x = std::min(bounds.min_x, static_cast<double>(point.x));
        bounds.min_y = std::min(bounds.min_y, static_cast<double>(point.y));
        bounds.max_x = std::max(bounds.max_x, static_cast<double>(point.x));
        bounds.max_y = std::max(bounds.max_y, static_cast<double>(point.y));
    }

    if (bounds.min_x == std::numeric_limits<double>::max())
    {
        throw std::runtime_error("No valid observed points within [z_min, z_max]");
    }

    bounds.min_x -= options.padding;
    bounds.min_y -= options.padding;
    bounds.max_x += options.padding;
    bounds.max_y += options.padding;
    return bounds;
}

size_t flattenIndex(int x, int y, int width)
{
    return static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x);
}

bool worldToGrid(const PointType &point, const Bounds &bounds, const Options &options, int width, int height, int &gx, int &gy)
{
    gx = static_cast<int>(std::floor((static_cast<double>(point.x) - bounds.min_x) / options.resolution));
    gy = static_cast<int>(std::floor((static_cast<double>(point.y) - bounds.min_y) / options.resolution));
    return gx >= 0 && gx < width && gy >= 0 && gy < height;
}

void inflateOccupied(std::vector<CellState> &grid, int width, int height, int radius_cells)
{
    if (radius_cells <= 0)
    {
        return;
    }

    const int r2 = radius_cells * radius_cells;
    std::vector<CellState> inflated = grid;

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            if (grid[flattenIndex(x, y, width)] != CellState::Occupied)
            {
                continue;
            }
            for (int dy = -radius_cells; dy <= radius_cells; ++dy)
            {
                for (int dx = -radius_cells; dx <= radius_cells; ++dx)
                {
                    if (dx * dx + dy * dy > r2)
                    {
                        continue;
                    }
                    const int nx = x + dx;
                    const int ny = y + dy;
                    if (nx < 0 || nx >= width || ny < 0 || ny >= height)
                    {
                        continue;
                    }
                    inflated[flattenIndex(nx, ny, width)] = CellState::Occupied;
                }
            }
        }
    }
    grid.swap(inflated);
}

size_t removeSmallClusters(std::vector<CellState> &grid, int width, int height, int min_cluster_size)
{
    if (min_cluster_size <= 0)
    {
        return 0;
    }

    const size_t total = static_cast<size_t>(width) * static_cast<size_t>(height);
    std::vector<bool> visited(total, false);
    size_t removed_cells = 0;
    const int dx[] = {1, -1, 0, 0};
    const int dy[] = {0, 0, 1, -1};

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            const size_t idx = flattenIndex(x, y, width);
            if (visited[idx] || grid[idx] != CellState::Occupied)
            {
                continue;
            }

            // BFS to find connected occupied cluster
            std::vector<size_t> cluster;
            std::queue<std::pair<int, int>> q;
            q.push({x, y});
            visited[idx] = true;

            while (!q.empty())
            {
                auto [cx, cy] = q.front();
                q.pop();
                cluster.push_back(flattenIndex(cx, cy, width));

                for (int d = 0; d < 4; ++d)
                {
                    const int nx = cx + dx[d];
                    const int ny = cy + dy[d];
                    if (nx < 0 || nx >= width || ny < 0 || ny >= height)
                    {
                        continue;
                    }
                    const size_t nidx = flattenIndex(nx, ny, width);
                    if (!visited[nidx] && grid[nidx] == CellState::Occupied)
                    {
                        visited[nidx] = true;
                        q.push({nx, ny});
                    }
                }
            }

            // Remove cluster if too small
            if (static_cast<int>(cluster.size()) < min_cluster_size)
            {
                for (const size_t cidx : cluster)
                {
                    grid[cidx] = CellState::Free;
                }
                removed_cells += cluster.size();
            }
        }
    }
    return removed_cells;
}

void writePGM(const std::filesystem::path &pgm_path, const std::vector<CellState> &grid, int width, int height)
{
    std::ofstream ofs(pgm_path, std::ios::binary);
    if (!ofs.is_open())
    {
        throw std::runtime_error("Failed to open " + pgm_path.string());
    }
    ofs << "P5\n";
    ofs << "# CREATOR: pcd_to_nav2_map\n";
    ofs << width << " " << height << "\n";
    ofs << "255\n";

    std::vector<uint8_t> row(static_cast<size_t>(width), 205U);
    for (int y = height - 1; y >= 0; --y)
    {
        for (int x = 0; x < width; ++x)
        {
            const auto cell = grid[flattenIndex(x, y, width)];
            uint8_t value = 205U;
            if (cell == CellState::Occupied)
            {
                value = 0U;
            }
            else if (cell == CellState::Free)
            {
                value = 254U;
            }
            row[static_cast<size_t>(x)] = value;
        }
        ofs.write(reinterpret_cast<const char *>(row.data()), static_cast<std::streamsize>(row.size()));
    }
}

void writeYAML(const std::filesystem::path &yaml_path, const Bounds &bounds, const Options &options)
{
    std::ofstream ofs(yaml_path);
    if (!ofs.is_open())
    {
        throw std::runtime_error("Failed to open " + yaml_path.string());
    }
    ofs << std::fixed << std::setprecision(6);
    ofs << "image: map.pgm\n";
    ofs << "mode: trinary\n";
    ofs << "resolution: " << options.resolution << "\n";
    ofs << "origin: [" << bounds.min_x << ", " << bounds.min_y << ", 0.0]\n";
    ofs << "negate: 0\n";
    ofs << "occupied_thresh: 0.65\n";
    ofs << "free_thresh: 0.25\n";
}

std::vector<Eigen::Vector2d> loadPoses(const std::string &poses_path)
{
    std::vector<Eigen::Vector2d> poses;
    std::ifstream ifs(poses_path);
    if (!ifs.is_open())
    {
        throw std::runtime_error("Failed to open poses file: " + poses_path);
    }

    std::string line;
    while (std::getline(ifs, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }
        std::istringstream iss(line);
        std::string name;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        // Format: name x y z qw qx qy qz (we only need x, y)
        if (!(iss >> name >> x >> y >> z))
        {
            continue;
        }
        poses.emplace_back(x, y);
    }

    if (poses.empty())
    {
        throw std::runtime_error("No valid poses found in: " + poses_path);
    }
    return poses;
}

size_t findNearestPose(double px, double py, const std::vector<Eigen::Vector2d> &poses)
{
    size_t best = 0;
    double best_dist2 = std::numeric_limits<double>::max();
    for (size_t i = 0; i < poses.size(); ++i)
    {
        const double dx = px - poses[i].x();
        const double dy = py - poses[i].y();
        const double d2 = dx * dx + dy * dy;
        if (d2 < best_dist2)
        {
            best_dist2 = d2;
            best = i;
        }
    }
    return best;
}

void bresenhamRaycast(int x0, int y0, int x1, int y1,
                      std::vector<CellState> &grid, int width, int height)
{
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    // Walk from origin toward target, but stop BEFORE the target cell
    // (the target cell is the observed point — it may be occupied)
    while (true)
    {
        // Stop before reaching the target
        if (x0 == x1 && y0 == y1)
        {
            break;
        }

        // Mark current cell as free if it's unknown
        if (x0 >= 0 && x0 < width && y0 >= 0 && y0 < height)
        {
            const size_t idx = flattenIndex(x0, y0, width);
            if (grid[idx] == CellState::Unknown)
            {
                grid[idx] = CellState::Free;
            }
        }

        int e2 = 2 * err;
        if (e2 > -dy)
        {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx)
        {
            err += dx;
            y0 += sy;
        }
    }
}

int main(int argc, char **argv)
{
    try
    {
        const Options options = parseOptions(argc, argv);
        if (!std::filesystem::exists(options.pcd_path))
        {
            throw std::runtime_error("PCD file does not exist: " + options.pcd_path);
        }

        std::filesystem::create_directories(options.out_dir);

        CloudType::Ptr cloud;
        if (!loadCloud(options.pcd_path, cloud))
        {
            throw std::runtime_error("Failed to load PCD file: " + options.pcd_path);
        }

        // SOR filtering: remove isolated noise points in the obstacle Z band
        if (options.sor_mean_k > 0)
        {
            // Extract obstacle-band points
            CloudType::Ptr obstacle_cloud(new CloudType);
            CloudType::Ptr non_obstacle_cloud(new CloudType);
            for (const auto &p : cloud->points)
            {
                if (isFinite(p) && p.z >= options.obstacle_z_min && p.z <= options.obstacle_z_max)
                {
                    obstacle_cloud->push_back(p);
                }
                else
                {
                    non_obstacle_cloud->push_back(p);
                }
            }

            const size_t before = obstacle_cloud->size();

            // Apply Statistical Outlier Removal
            pcl::StatisticalOutlierRemoval<PointType> sor;
            sor.setInputCloud(obstacle_cloud);
            sor.setMeanK(options.sor_mean_k);
            sor.setStddevMulThresh(options.sor_stddev_thresh);
            CloudType::Ptr filtered(new CloudType);
            sor.filter(*filtered);

            const size_t after = filtered->size();
            std::cout << "SOR filter (k=" << options.sor_mean_k
                      << ", stddev=" << options.sor_stddev_thresh
                      << "): " << before << " -> " << after
                      << " obstacle-band points ("
                      << (before - after) << " removed)\n";

            // Recombine: filtered obstacle points + non-obstacle points
            *filtered += *non_obstacle_cloud;
            cloud = filtered;
        }

        const Bounds bounds = computeBounds(cloud, options);
        const int width = static_cast<int>(std::ceil((bounds.max_x - bounds.min_x) / options.resolution)) + 1;
        const int height = static_cast<int>(std::ceil((bounds.max_y - bounds.min_y) / options.resolution)) + 1;

        if (width <= 1 || height <= 1)
        {
            throw std::runtime_error("Projected map is too small");
        }

        std::vector<int> observed_hits(static_cast<size_t>(width) * static_cast<size_t>(height), 0);
        std::vector<int> obstacle_hits(static_cast<size_t>(width) * static_cast<size_t>(height), 0);
        std::vector<int> ceiling_hits(static_cast<size_t>(width) * static_cast<size_t>(height), 0);

        for (const auto &point : cloud->points)
        {
            if (!isFinite(point))
            {
                continue;
            }
            int gx = 0;
            int gy = 0;
            if (!worldToGrid(point, bounds, options, width, height, gx, gy))
            {
                continue;
            }
            const size_t idx = flattenIndex(gx, gy, width);
            if (isObservedPoint(point, options))
            {
                observed_hits[idx] += 1;
            }
            if (isObstaclePoint(point, options))
            {
                obstacle_hits[idx] += 1;
            }
            if (isCeilingPoint(point, options))
            {
                ceiling_hits[idx] += 1;
            }
        }

        std::vector<CellState> grid(static_cast<size_t>(width) * static_cast<size_t>(height), CellState::Unknown);
        for (size_t i = 0; i < grid.size(); ++i)
        {
            // A cell is "free" if any point was observed there (ground, wall top, ceiling)
            if (observed_hits[i] > 0)
            {
                grid[i] = CellState::Free;
            }
            // A cell with only ceiling points (no obstacle-height points) is free —
            // the robot can pass underneath
            if (ceiling_hits[i] > 0 && obstacle_hits[i] == 0)
            {
                grid[i] = CellState::Free;
            }
            // Override to occupied if enough obstacle-height points exist
            if (obstacle_hits[i] >= options.obstacle_min_hits)
            {
                grid[i] = CellState::Occupied;
            }
        }

        // Raycasting: mark cells along rays from sensor poses to observed points as free
        if (!options.poses_path.empty())
        {
            const auto poses = loadPoses(options.poses_path);
            std::cout << "Loaded " << poses.size() << " sensor poses for raycasting\n";

            size_t rays_cast = 0;
            for (const auto &point : cloud->points)
            {
                if (!isFinite(point) || !isObservedPoint(point, options))
                {
                    continue;
                }

                int gx = 0;
                int gy = 0;
                if (!worldToGrid(point, bounds, options, width, height, gx, gy))
                {
                    continue;
                }

                // Find nearest sensor pose
                const double px = static_cast<double>(point.x);
                const double py = static_cast<double>(point.y);
                const size_t pose_idx = findNearestPose(px, py, poses);
                const auto &pose = poses[pose_idx];

                // Convert pose to grid coordinates
                const int ox = static_cast<int>(std::floor((pose.x() - bounds.min_x) / options.resolution));
                const int oy = static_cast<int>(std::floor((pose.y() - bounds.min_y) / options.resolution));

                bresenhamRaycast(ox, oy, gx, gy, grid, width, height);
                ++rays_cast;
            }
            std::cout << "Raycasting: " << rays_cast << " rays cast from "
                      << poses.size() << " sensor poses\n";
        }

        // Remove small isolated occupied blobs (noise), then inflate remaining obstacles
        const size_t removed = removeSmallClusters(grid, width, height, options.min_obstacle_cluster);
        if (removed > 0)
        {
            std::cout << "Cluster filter (min=" << options.min_obstacle_cluster
                      << "): removed " << removed << " noise cells\n";
        }

        const int inflation_cells = static_cast<int>(std::ceil(options.inflation_radius / options.resolution));
        inflateOccupied(grid, width, height, inflation_cells);

        size_t free_cells = 0;
        size_t occupied_cells = 0;
        size_t unknown_cells = 0;
        for (const auto cell : grid)
        {
            if (cell == CellState::Free)
            {
                ++free_cells;
            }
            else if (cell == CellState::Occupied)
            {
                ++occupied_cells;
            }
            else
            {
                ++unknown_cells;
            }
        }

        const std::filesystem::path out_dir(options.out_dir);
        const std::filesystem::path pgm_path = out_dir / "map.pgm";
        const std::filesystem::path yaml_path = out_dir / "map.yaml";
        writePGM(pgm_path, grid, width, height);
        writeYAML(yaml_path, bounds, options);

        const double total_cells = static_cast<double>(grid.size());
        const double known_ratio = (total_cells > 0.0) ? (1.0 - static_cast<double>(unknown_cells) / total_cells) : 0.0;
        const double occ_ratio = (free_cells + occupied_cells > 0)
                                     ? static_cast<double>(occupied_cells) / static_cast<double>(free_cells + occupied_cells)
                                     : 0.0;

        std::cout << "Input cloud: " << options.pcd_path << "\n";
        std::cout << "Output map: " << yaml_path << "\n";
        std::cout << "Grid size: " << width << " x " << height << " (" << options.resolution << " m/cell)\n";
        std::cout << "Bounds: x[" << bounds.min_x << ", " << bounds.max_x << "], "
                  << "y[" << bounds.min_y << ", " << bounds.max_y << "]\n";
        std::cout << "Cells: free=" << free_cells
                  << " occupied=" << occupied_cells
                  << " unknown=" << unknown_cells << "\n";
        std::cout << std::fixed << std::setprecision(3)
                  << "Known ratio=" << known_ratio
                  << ", Occupied ratio(known)=" << occ_ratio << "\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << "[pcd_to_nav2_map] " << e.what() << "\n";
        return 1;
    }
    return 0;
}
