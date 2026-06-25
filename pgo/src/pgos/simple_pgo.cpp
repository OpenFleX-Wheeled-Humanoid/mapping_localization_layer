#include "simple_pgo.h"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

SimplePGO::SimplePGO(const Config &config) : m_config(config)
{
    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = 0.01;
    isam2_params.relinearizeSkip = 1;
    m_isam2 = std::make_shared<gtsam::ISAM2>(isam2_params);
    m_initial_values.clear();
    m_graph.resize(0);
    m_r_offset.setIdentity();
    m_t_offset.setZero();

    m_icp.setMaximumIterations(50);
    m_icp.setMaxCorrespondenceDistance(10);
    m_icp.setTransformationEpsilon(1e-6);
    m_icp.setEuclideanFitnessEpsilon(1e-6);
    m_icp.setRANSACIterations(0);

    // Initialize NDT
    m_ndt.setResolution(m_config.ndt_resolution);
    m_ndt.setStepSize(m_config.ndt_step_size);
    m_ndt.setMaximumIterations(m_config.ndt_max_iterations);
    m_ndt.setTransformationEpsilon(m_config.ndt_epsilon);

    // Initialize ScanContext
    m_sc_manager.setSCdistThres(m_config.sc_dist_thresh);
    m_sc_manager.setMaximumRadius(m_config.sc_max_radius);
}

bool SimplePGO::isKeyPose(const PoseWithTime &pose)
{
    if (m_key_poses.size() == 0)
        return true;
    const KeyPoseWithCloud &last_item = m_key_poses.back();
    double delta_trans = (pose.t - last_item.t_local).norm();
    double delta_deg = Eigen::Quaterniond(pose.r).angularDistance(Eigen::Quaterniond(last_item.r_local)) * 57.324;
    if (delta_trans > m_config.key_pose_delta_trans || delta_deg > m_config.key_pose_delta_deg)
        return true;
    return false;
}
bool SimplePGO::addKeyPose(const CloudWithPose &cloud_with_pose)
{
    bool is_key_pose = isKeyPose(cloud_with_pose.pose);
    if (!is_key_pose)
        return false;
    size_t idx = m_key_poses.size();
    M3D init_r = m_r_offset * cloud_with_pose.pose.r;
    V3D init_t = m_r_offset * cloud_with_pose.pose.t + m_t_offset;
    // 添加初始值
    m_initial_values.insert(idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)));
    if (idx == 0)
    {
        // 添加先验约束
        gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * 1e-12);
        m_graph.add(gtsam::PriorFactor<gtsam::Pose3>(idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)), noise));
    }
    else
    {
        // 添加里程计约束
        const KeyPoseWithCloud &last_item = m_key_poses.back();
        M3D r_between = last_item.r_local.transpose() * cloud_with_pose.pose.r;
        V3D t_between = last_item.r_local.transpose() * (cloud_with_pose.pose.t - last_item.t_local);
        gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-6).finished());
        m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(idx - 1, idx, gtsam::Pose3(gtsam::Rot3(r_between), gtsam::Point3(t_between)), noise));
    }
    KeyPoseWithCloud item;
    item.time = cloud_with_pose.pose.second;
    item.r_local = cloud_with_pose.pose.r;
    item.t_local = cloud_with_pose.pose.t;
    item.body_cloud = cloud_with_pose.cloud;
    item.r_global = init_r;
    item.t_global = init_t;
    m_key_poses.push_back(item);

    // Build ScanContext descriptor for this keyframe
    m_sc_manager.makeAndSaveScancontextAndKeys(*cloud_with_pose.cloud);

    return true;
}

CloudType::Ptr SimplePGO::getSubMap(int idx, int half_range, double resolution)
{
    assert(idx >= 0 && idx < static_cast<int>(m_key_poses.size()));
    int min_idx = std::max(0, idx - half_range);
    int max_idx = std::min(static_cast<int>(m_key_poses.size()) - 1, idx + half_range);

    CloudType::Ptr ret(new CloudType);
    for (int i = min_idx; i <= max_idx; i++)
    {

        CloudType::Ptr body_cloud = m_key_poses[i].body_cloud;
        CloudType::Ptr global_cloud(new CloudType);
        pcl::transformPointCloud(*body_cloud, *global_cloud, m_key_poses[i].t_global, Eigen::Quaterniond(m_key_poses[i].r_global));
        *ret += *global_cloud;
    }
    if (resolution > 0)
    {
        pcl::VoxelGrid<PointType> voxel_grid;
        voxel_grid.setLeafSize(resolution, resolution, resolution);
        voxel_grid.setInputCloud(ret);
        voxel_grid.filter(*ret);
    }
    return ret;
}

void SimplePGO::searchForLoopPairs()
{
    if (m_key_poses.size() < 10)
        return;
    if (m_config.min_loop_detect_duration > 0.0)
    {
        if (m_history_pairs.size() > 0)
        {
            double current_time = m_key_poses.back().time;
            double last_time = m_key_poses[m_history_pairs.back().second].time;
            if (current_time - last_time < m_config.min_loop_detect_duration)
                return;
        }
    }

    size_t cur_idx = m_key_poses.size() - 1;

    // Use ScanContext to find loop closure candidate
    auto sc_result = m_sc_manager.detectLoopClosureID();
    int sc_loop_idx = sc_result.first;
    float sc_yaw_diff = sc_result.second;

    if (sc_loop_idx < 0)
        return;

    // Enforce temporal separation from candidate loop keyframe.
    if (m_config.loop_time_tresh > 0.0)
    {
        double dt = std::fabs(m_key_poses[cur_idx].time - m_key_poses[sc_loop_idx].time);
        if (dt < m_config.loop_time_tresh)
            return;
    }

    // Reject loop candidates that are too far in odometry-local XY.
    if (m_config.loop_search_radius > 0.0)
    {
        double xy_dist = (m_key_poses[cur_idx].t_local.head<2>() - m_key_poses[sc_loop_idx].t_local.head<2>()).norm();
        if (xy_dist > m_config.loop_search_radius)
            return;
    }

    // Build target submap around the SC-detected candidate
    CloudType::Ptr target_cloud = getSubMap(sc_loop_idx, m_config.loop_submap_half_range, m_config.submap_resolution);
    CloudType::Ptr source_cloud = getSubMap(m_key_poses.size() - 1, 0, m_config.submap_resolution);

    // Compute initial guess from SC yaw difference
    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    // Apply SC yaw correction as rotation around Z axis
    Eigen::AngleAxisf yaw_correction(sc_yaw_diff, Eigen::Vector3f::UnitZ());
    init_guess.block<3, 3>(0, 0) = yaw_correction.toRotationMatrix();
    // translation is zero since source cloud is already in global coords near the target

    // Stage 1: NDT coarse alignment
    CloudType::Ptr ndt_aligned(new CloudType);
    m_ndt.setInputSource(source_cloud);
    m_ndt.setInputTarget(target_cloud);
    m_ndt.align(*ndt_aligned, init_guess);

    if (!m_ndt.hasConverged() || m_ndt.getFitnessScore() > m_config.ndt_score_thresh)
    {
        // NDT failed, try ICP directly with identity guess
        CloudType::Ptr icp_aligned(new CloudType);
        m_icp.setInputSource(source_cloud);
        m_icp.setInputTarget(target_cloud);
        m_icp.align(*icp_aligned);
        if (!m_icp.hasConverged() || m_icp.getFitnessScore() > m_config.loop_score_tresh)
            return;

        M4F loop_transform = m_icp.getFinalTransformation();
        LoopPair one_pair;
        one_pair.source_id = cur_idx;
        one_pair.target_id = sc_loop_idx;
        one_pair.score = m_icp.getFitnessScore();
        M3D r_refined = loop_transform.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].r_global;
        V3D t_refined = loop_transform.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].t_global + loop_transform.block<3, 1>(0, 3).cast<double>();
        one_pair.r_offset = m_key_poses[sc_loop_idx].r_global.transpose() * r_refined;
        one_pair.t_offset = m_key_poses[sc_loop_idx].r_global.transpose() * (t_refined - m_key_poses[sc_loop_idx].t_global);
        m_cache_pairs.push_back(one_pair);
        m_history_pairs.emplace_back(one_pair.target_id, one_pair.source_id);
        std::cout << "[PGO-SC] Loop closed (ICP only): " << cur_idx << " -> " << sc_loop_idx
                  << " score=" << one_pair.score << std::endl;
        return;
    }

    // Stage 2: ICP fine alignment using NDT result as initial guess
    M4F ndt_transform = m_ndt.getFinalTransformation();
    CloudType::Ptr icp_aligned(new CloudType);
    m_icp.setInputSource(source_cloud);
    m_icp.setInputTarget(target_cloud);
    m_icp.align(*icp_aligned, ndt_transform);

    if (!m_icp.hasConverged() || m_icp.getFitnessScore() > m_config.loop_score_tresh)
        return;

    M4F loop_transform = m_icp.getFinalTransformation();

    LoopPair one_pair;
    one_pair.source_id = cur_idx;
    one_pair.target_id = sc_loop_idx;
    one_pair.score = m_icp.getFitnessScore();
    M3D r_refined = loop_transform.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].r_global;
    V3D t_refined = loop_transform.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].t_global + loop_transform.block<3, 1>(0, 3).cast<double>();
    one_pair.r_offset = m_key_poses[sc_loop_idx].r_global.transpose() * r_refined;
    one_pair.t_offset = m_key_poses[sc_loop_idx].r_global.transpose() * (t_refined - m_key_poses[sc_loop_idx].t_global);
    m_cache_pairs.push_back(one_pair);
    m_history_pairs.emplace_back(one_pair.target_id, one_pair.source_id);
    std::cout << "[PGO-SC] Loop closed (NDT+ICP): " << cur_idx << " -> " << sc_loop_idx
              << " ndt_score=" << m_ndt.getFitnessScore()
              << " icp_score=" << one_pair.score << std::endl;
}

void SimplePGO::smoothAndUpdate()
{
    bool has_loop = !m_cache_pairs.empty();
    // 添加回环因子
    if (has_loop)
    {
        for (LoopPair &pair : m_cache_pairs)
        {
            m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(pair.target_id, pair.source_id,
                                                           gtsam::Pose3(gtsam::Rot3(pair.r_offset),
                                                                        gtsam::Point3(pair.t_offset)),
                                                           gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * pair.score)));
        }
        std::vector<LoopPair>().swap(m_cache_pairs);
    }
    // smooth and mapping
    m_isam2->update(m_graph, m_initial_values);
    m_isam2->update();
    if (has_loop)
    {
        m_isam2->update();
        m_isam2->update();
        m_isam2->update();
        m_isam2->update();
    }
    m_graph.resize(0);
    m_initial_values.clear();

    // update key poses
    gtsam::Values estimate_values = m_isam2->calculateBestEstimate();
    for (size_t i = 0; i < m_key_poses.size(); i++)
    {
        gtsam::Pose3 pose = estimate_values.at<gtsam::Pose3>(i);
        m_key_poses[i].r_global = pose.rotation().matrix().cast<double>();
        m_key_poses[i].t_global = pose.translation().matrix().cast<double>();
    }
    // update offset
    const KeyPoseWithCloud &last_item = m_key_poses.back();
    m_r_offset = last_item.r_global * last_item.r_local.transpose();
    m_t_offset = last_item.t_global - m_r_offset * last_item.t_local;
}

bool SimplePGO::saveScanContextDatabase(const std::string &dir_path)
{
    std::filesystem::path sc_dir(dir_path);
    std::filesystem::create_directories(sc_dir);

    size_t N = m_sc_manager.polarcontexts_.size();
    if (N == 0)
    {
        std::cerr << "[PGO-SC] No ScanContext data to save." << std::endl;
        return false;
    }

    int rows = m_sc_manager.PC_NUM_RING;
    int cols = m_sc_manager.PC_NUM_SECTOR;

    // Save metadata
    {
        std::ofstream f(sc_dir / "metadata.txt");
        f << N << " " << rows << " " << cols << std::endl;
        f.close();
    }

    // Save polarcontexts (N matrices of rows x cols doubles)
    {
        std::ofstream f(sc_dir / "polarcontexts.bin", std::ios::binary);
        for (size_t i = 0; i < N; i++)
        {
            const Eigen::MatrixXd &mat = m_sc_manager.polarcontexts_[i];
            f.write(reinterpret_cast<const char *>(mat.data()), rows * cols * sizeof(double));
        }
        f.close();
    }

    // Save invkeys (N matrices of rows x 1 doubles)
    {
        std::ofstream f(sc_dir / "invkeys.bin", std::ios::binary);
        for (size_t i = 0; i < N; i++)
        {
            const Eigen::MatrixXd &mat = m_sc_manager.polarcontext_invkeys_[i];
            f.write(reinterpret_cast<const char *>(mat.data()), rows * sizeof(double));
        }
        f.close();
    }

    // Save vkeys (N matrices of 1 x cols doubles)
    {
        std::ofstream f(sc_dir / "vkeys.bin", std::ios::binary);
        for (size_t i = 0; i < N; i++)
        {
            const Eigen::MatrixXd &mat = m_sc_manager.polarcontext_vkeys_[i];
            f.write(reinterpret_cast<const char *>(mat.data()), cols * sizeof(double));
        }
        f.close();
    }

    // Save invkeys_mat (KeyMat: N vectors of rows floats)
    {
        std::ofstream f(sc_dir / "invkeys_mat.bin", std::ios::binary);
        for (size_t i = 0; i < N; i++)
        {
            const std::vector<float> &vec = m_sc_manager.polarcontext_invkeys_mat_[i];
            f.write(reinterpret_cast<const char *>(vec.data()), rows * sizeof(float));
        }
        f.close();
    }

    std::cout << "[PGO-SC] Saved ScanContext database: " << N << " keyframes to " << dir_path << std::endl;
    return true;
}
