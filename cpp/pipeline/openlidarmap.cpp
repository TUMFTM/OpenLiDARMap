#include "pipeline/openlidarmap.hpp"

#include <guik/viewer/async_light_viewer.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <memory>
#include <fstream>

#include "io/loader_factory.hpp"
#include "io/point_cloud_saver.hpp"
#include "utils/helpers.hpp"

namespace openlidarmap::pipeline {

Pipeline::Pipeline(const config::Config &config)
    : config_(config),
      scan2scan_config_(config),
      scan2map_config_(config),
      last_save_distance_(0.0),
      submap_counter_(0) {
    preprocess_ = std::make_unique<Preprocess>(config_);
    scan2map_config_.registration_.removal_horizon = 1e9;
    scan2map_config_.registration_.max_num_points_in_cell = 100;
    scan2map_config_.preprocess_.min_range = -1e9;
    scan2map_config_.preprocess_.max_range = 1e9;
    scan2map_registration_ = std::make_unique<Registration>(scan2map_config_);
    scan2scan_registration_ = std::make_unique<Registration>(scan2scan_config_);
    pose_graph_ = std::make_unique<PoseGraph>(config_, poses_);
}

Pipeline::~Pipeline() {
    stop_requested_ = true;
    paused_ = false;
    pause_cv_.notify_all();

    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }
}

bool Pipeline::initialize(const std::string &map_path,
                          const std::string &scans_dir,
                          const std::string &output_path,
                          const std::string &imu_dir,
                          const Vector7d &initial_pose) {
    output_path_ = output_path;
    imu_dir_ = imu_dir;

    // Load and initialize map
    auto map_cloud = io::LoaderFactory::loadPointCloud(scan2map_config_, map_path);
    if (!scan2map_registration_->initialize(map_cloud, Eigen::Isometry3d::Identity())) {
        return false;
    }

    // Get sorted scan files
    scan_files_ = utils::FileUtils::getFiles(scans_dir);

    // Get sorted IMU files if directory is provided
    if (!imu_dir.empty()) {
        imu_files_ = utils::FileUtils::getFiles(imu_dir);
        std::sort(imu_files_.begin(), imu_files_.end());
    }

    // Initialize first two poses
    if (!initializeFirstPoses(initial_pose)) {
        return false;
    }

    return true;
}

bool Pipeline::initializeFirstPoses(const Vector7d &initial_pose) {
    // First frame scan2map
    auto first_frame = io::LoaderFactory::loadPointCloud(config_, scan_files_[0]);
    small_gicp::estimate_covariances_tbb(*first_frame, config_.preprocess_.num_neighbors);
    auto first_frame_processed = preprocess_->preprocess_cloud(first_frame);
    small_gicp::estimate_covariances_tbb(*first_frame_processed, config_.preprocess_.num_neighbors);

    auto init_result = scan2map_registration_->register_frame(
        first_frame_processed, utils::PoseUtils::poseVectorToIsometry(initial_pose));

    // Initialize with scan2map result
    auto first_aligned_pose = utils::PoseUtils::isometryToPoseVector(init_result.T_target_source);
    addPose(first_aligned_pose);
    pose_graph_->addConstraint(0, 0, first_aligned_pose, true);

    kitti_poses_.reserve(scan_files_.size());
    kitti_poses_.emplace_back(first_aligned_pose);

    if (!scan2scan_registration_->initialize(
            first_frame, utils::PoseUtils::poseVectorToIsometry(first_aligned_pose))) {
        return false;
    }

    // Second frame scan2scan
    auto second_frame = io::LoaderFactory::loadPointCloud(config_, scan_files_[1]);
    auto second_frame_processed = preprocess_->preprocess_cloud(second_frame);
    small_gicp::estimate_covariances_tbb(*second_frame_processed,
                                         config_.preprocess_.num_neighbors);

    auto scan2scan_result = scan2scan_registration_->register_frame(
        second_frame_processed, utils::PoseUtils::poseVectorToIsometry(first_aligned_pose));

    auto second_aligned_pose =
        utils::PoseUtils::isometryToPoseVector(scan2scan_result.T_target_source);
    addPose(second_aligned_pose);

    return true;
}

bool Pipeline::run() {
    if (scan_files_.empty()) {
        std::cerr << "No scan files loaded" << std::endl;
        return false;
    }

    timer_.start();
    guik::LightViewer *async_viewer = nullptr;

    try {
        if (config_.pipeline_.visualize) {
            async_viewer = guik::LightViewer::instance();
            if (!async_viewer) {
                std::cerr << "Failed to initialize visualization" << std::endl;
                return false;
            }

            auto viewer = guik::viewer();
            viewer->set_clear_color({0.1f, 0.1f, 0.1f, 1.0f});
            if (!viewer) {
                std::cerr << "Guik viewer is null immediately after LightViewer init" << std::endl;
                return false;
            }

            Eigen::Vector2f z_range(0.0f, 100.0f);
            async_viewer->invoke([=] {
                auto viewer = guik::viewer();
                if (viewer) {
                    viewer->shader_setting().add("z_range", z_range);
                }
            });

            // Map visualization
            // auto map = scan2map_registration_->get_map();
            // auto voxel_cloud = small_gicp::traits::voxel_points(*map);
            // auto viewer_ptr = std::shared_ptr<guik::LightViewer>(async_viewer,
            // [](guik::LightViewer *) {}); std::vector<Eigen::Vector4d>
            // points_copy(voxel_cloud.begin(), voxel_cloud.end()); viewer_ptr->invoke(
            //     [points = std::move(points_copy), pose = Eigen::Isometry3d::Identity(), viewer =
            //     viewer_ptr]() { viewer->update_points("map", points,
            //         guik::FlatWhite(Eigen::Isometry3d::Identity()));
            //     });
        }

        processing_thread_ = std::thread(&Pipeline::processingLoop, this);

        while (!stop_requested_) {
            if (config_.pipeline_.visualize && async_viewer) {
                if (!async_viewer->spin_once()) {
                    stop_requested_ = true;
                    break;
                }
                handleVisualizationControls(async_viewer);
            } else {
                utils::printProgressBar(progress_, current_processing_time_);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }

        return !stop_requested_;

    } catch (const std::exception &e) {
        std::cerr << "Fatal error in run(): " << e.what() << std::endl;
        stop_requested_ = true;
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
        return false;
    }
}

void Pipeline::processingLoop() {
    try {
        for (size_t i = 1; i < scan_files_.size() && !stop_requested_; ++i) {
            waitIfPaused();
            if (stop_requested_) break;

            auto frame = io::LoaderFactory::loadPointCloud(config_, scan_files_[i]);
            if (!processFrame(frame, i)) {
                stop_requested_ = true;
                break;
            }

            if (config_.pipeline_.visualize) {
                updateVisualization(frame);
            }

            timer_.lap();
            current_processing_time_ = timer_.msec();
            progress_ = static_cast<float>(++frame_count_) / scan_files_.size();

            // std::cout << "Frame processing time [ms]: " << current_processing_time_ << std::endl;
        }

        writeResults();

    } catch (const std::exception &e) {
        std::cerr << "Processing error: " << e.what() << std::endl;
        stop_requested_ = true;
    }
}

bool Pipeline::processFrame(small_gicp::PointCloud::Ptr &frame, size_t frame_idx) {
    if (!frame || frame->empty()) {
        return false;
    }

    auto initial_guess = utils::PoseUtils::poseVectorToIsometry(poses_[pose_index_]);
    auto processed_frame = preprocess_->preprocess_cloud(frame);

    // Scan-to-scan registration
    auto scan2scan_result = scan2scan_registration_->register_frame(processed_frame, initial_guess);

    if (!utils::PoseUtils::isMoving(scan2scan_result.T_target_source, poses_[pose_index_ - 1],
                                    config_.pipeline_.translation_threshold,
                                    config_.pipeline_.rotation_threshold)) {
        kitti_poses_.emplace_back(
            utils::PoseUtils::isometryToPoseVector(scan2scan_result.T_target_source));
        return true;
    }

    // Scan-to-map registration
    auto scan2map_result = scan2map_registration_->register_frame(processed_frame, initial_guess);

    // Update pose graph
    updatePoseGraph(scan2map_result, scan2scan_result);
    
    // Add IMU constraint if available
    if (!imu_files_.empty()) {
        addIMUConstraint(frame_idx);
    }
    
    // Optimize
    if (!pose_graph_->optimize()) {
        return false;
    }

    // Update scan2scan target with optimized pose
    scan2scan_registration_->get_map()->distance_insert(
        *processed_frame, utils::PoseUtils::poseVectorToIsometry(poses_[pose_index_]));

    // Adaptive kernel of KISS-ICP
    const auto deviation =
        initial_guess.inverse() * utils::PoseUtils::poseVectorToIsometry(poses_[pose_index_]);
    const double model_error = [&]() {
        const double theta = Eigen::AngleAxisd(deviation.rotation().matrix()).angle();
        const double delta_rot = 2.0 * 100.0 * std::sin(theta / 2.0);
        const double delta_trans = deviation.translation().norm();
        return delta_trans + delta_rot;
    }();
    scan2scan_config_.kernel_.model_sse += model_error * model_error;
    scan2scan_config_.kernel_.num_samples++;
    scan2scan_config_.kernel_.sigma =
        std::sqrt(scan2scan_config_.kernel_.model_sse / scan2scan_config_.kernel_.num_samples) /
        3.0;
    scan2scan_config_.registration_.max_dist_sq =
        (9.0 * scan2scan_config_.kernel_.sigma) * (9.0 * scan2scan_config_.kernel_.sigma);

    scan2scan_registration_->update_config(scan2scan_config_);

    // Save submap
    if (config_.pipeline_.save_submaps) {
        if (!current_submap_) {
            constexpr auto resolution = 0.25;
            current_submap_ =
                std::make_shared<small_gicp::IncrementalVoxelMap<small_gicp::FlatContainerPoints>>(
                    resolution);
            current_submap_->voxel_setting.max_num_points_in_cell = 100;
            current_submap_->removal_horizon = 1e9;
        }

        current_submap_->distance_insert(
            *frame, utils::PoseUtils::poseVectorToIsometry(poses_[pose_index_]));

        double total_distance =
            calculateTravelledDistance(poses_[pose_index_], poses_[pose_index_ - 1]);
        last_save_distance_ += total_distance;

        if (last_save_distance_ >= config_.pipeline_.map_save_interval) {
            saveSubmap();
            last_save_distance_ = 0.0;
        }
    }

    // Predict next pose
    addPose(predictNextPose());

    kitti_poses_.emplace_back(poses_[pose_index_]);
    pose_index_++;

    return true;
}

void Pipeline::writeResults() const {
    utils::FileUtils::writePosesToCSV(output_path_ + "/poses.txt", kitti_poses_);
}

void Pipeline::updatePoseGraph(const small_gicp::RegistrationResult &scan2map_result,
                               const small_gicp::RegistrationResult &scan2scan_result) {
    pose_graph_->addConstraint(
        pose_index_ - 1, pose_index_,
        utils::PoseUtils::isometryToPoseVector(scan2scan_result.T_target_source), false);

    if (scan2map_result.num_inliers > config_.registration_.map_overlap) {
        pose_graph_->addConstraint(
            pose_index_, pose_index_,
            utils::PoseUtils::isometryToPoseVector(scan2map_result.T_target_source), true);
    }
}

void Pipeline::addPose(const Vector7d &pose) { poses_.emplace_back(pose); }

Vector7d Pipeline::predictNextPose() {
    return ConstantDistancePredictor::predict(poses_[pose_index_], poses_[pose_index_ - 1]);
}

void Pipeline::updateVisualization(const small_gicp::PointCloud::Ptr &cloud) {
    std::lock_guard<std::mutex> lock(visualization_mutex_);
    if (!config_.pipeline_.visualize || !cloud) {
        return;
    }

    auto async_viewer = guik::LightViewer::instance();
    if (!async_viewer) {
        return;
    }

    auto viewer_ptr = std::shared_ptr<guik::LightViewer>(async_viewer, [](guik::LightViewer *) {});

    auto map = scan2scan_registration_->get_map();
    if (!map || pose_index_ >= poses_.size()) {
        return;
    }

    auto voxel_cloud = small_gicp::traits::voxel_points(*map);
    if (voxel_cloud.empty()) {
        return;
    }

    std::vector<Eigen::Vector4d> points_copy(voxel_cloud.begin(), voxel_cloud.end());
    auto current_pose = poses_[pose_index_ - 1];
    auto pose_isometry = utils::PoseUtils::poseVectorToIsometry(current_pose);

    viewer_ptr->invoke(
        [points = std::move(points_copy), pose = pose_isometry, viewer = viewer_ptr]() {
            try {
                auto gui_viewer = guik::viewer();
                if (!gui_viewer) return;

                // Update view parameters
                Eigen::Vector2f z_range;
                z_range[0] = pose.translation().z() - 3.0f;
                z_range[1] = pose.translation().z() + 10.0f;
                gui_viewer->shader_setting().add("z_range", z_range);

                // Update points
                if (!points.empty()) {
                    viewer->update_points("scan2scan", points,
                                          guik::Rainbow(Eigen::Isometry3d::Identity()));
                }

                // Update camera
                viewer->lookat(pose.translation().cast<float>());
                viewer->update_coord(guik::anon(), guik::VertexColor(pose).scale(0.5f));

            } catch (const std::exception &e) {
                std::cerr << "Visualization update failed: " << e.what() << std::endl;
            }
        });
}

void Pipeline::handleVisualizationControls(guik::LightViewer *viewer) {
    std::lock_guard<std::mutex> lock(viewer_mutex_);
    if (!viewer) return;

    viewer->invoke([this] {
        ImGui::Begin("Pipeline Status");
        ImGui::Text("Processing Time: %.2f ms", current_processing_time_);
        ImGui::ProgressBar(progress_, ImVec2(-1, 0),
                           (std::to_string(static_cast<int>(progress_ * 100)) + "%").c_str());

        if (ImGui::Button(paused_ ? "Resume" : "Pause")) {
            std::unique_lock<std::mutex> lock(pause_mutex_);
            paused_ = !paused_;
            if (!paused_) {
                pause_cv_.notify_one();
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Stop")) {
            stop_requested_ = true;
            paused_ = false;
            pause_cv_.notify_one();
        }

        ImGui::End();
    });
}

void Pipeline::waitIfPaused() {
    std::unique_lock<std::mutex> lock(pause_mutex_);
    while (paused_ && !stop_requested_) {
        pause_cv_.wait(lock);
    }
}

void Pipeline::saveSubmap() {
    if (!current_submap_) return;

    auto voxel_points = small_gicp::traits::voxel_points(*current_submap_);
    auto cloud = small_gicp::PointCloud(voxel_points);

    std::string filename = output_path_ + "/submap_" + std::to_string(submap_counter_++) + ".pcd";

    if (!io::PointCloudSaver::savePCD(filename, cloud)) {
        std::cerr << "Failed to save submap to " << filename << std::endl;
    }

    // Reset submap
    constexpr auto resolution = 0.25;
    current_submap_ =
        std::make_shared<small_gicp::IncrementalVoxelMap<small_gicp::FlatContainerPoints>>(
            resolution);
    current_submap_->voxel_setting.max_num_points_in_cell = 100;
    current_submap_->removal_horizon = 1e9;
}

double Pipeline::calculateTravelledDistance(const Vector7d &pose1, const Vector7d &pose2) {
    return (pose1.head<3>() - pose2.head<3>()).norm();
}

IMUData Pipeline::getIMUDataForFrame(size_t idx) const {
    // Check if we have valid IMU files available
    if (imu_files_.empty() || idx >= imu_files_.size()) {
        return IMUData(); // Return default IMU data
    }
    
    try {
        return IMUData::fromFile(imu_files_[idx]);
    } catch (const std::exception& e) {
        std::cerr << "Warning: " << e.what() << std::endl;
        return IMUData(); // Return default IMU data if file can't be read
    }
}

void Pipeline::addIMUConstraint(size_t idx) {
    if (idx >= scan_files_.size() || imu_files_.empty() || !pose_graph_) {
        return;
    }
    
    // Get IMU data corresponding to this frame index
    IMUData imu_data = getIMUDataForFrame(idx);
    pose_graph_->addIMUConstraint(idx, imu_data);
}

}  // namespace openlidarmap::pipeline
