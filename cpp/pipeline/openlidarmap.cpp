#include "pipeline/openlidarmap.hpp"

#include <guik/viewer/async_light_viewer.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <memory>

namespace openlidarmap::pipeline {

Pipeline::Pipeline(const config::Config &config)
    : config_(config), scan2scan_config_(config), scan2map_config_(config) {
    scan2scan_config_.registration_.removal_horizon = 100;
    scan2map_config_.registration_.max_num_points_in_cell = 20;
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
                          const Vector7d &initial_pose) {
    output_path_ = output_path;

    // Load and initialize map
    auto map_cloud = io::loadPCD_map(map_path);
    if (!scan2map_registration_->initialize(map_cloud, Eigen::Isometry3d::Identity())) {
        return false;
    }

    // Get sorted scan files
    scan_files_ = utils::FileUtils::getFiles(scans_dir);

    // Initialize first two poses
    if (!initializeFirstPoses(initial_pose)) {
        return false;
    }

    return true;
}

bool Pipeline::initializeFirstPoses(const Vector7d &initial_pose) {
    // First frame scan2map
    auto first_frame = io::loadBIN_kitti(config_, scan_files_[0]);

    auto init_result = scan2map_registration_->register_frame(
        first_frame, utils::PoseUtils::poseVectorToIsometry(initial_pose));

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
    auto second_frame = io::loadBIN_kitti(config_, scan_files_[1]);

    auto scan2scan_result = scan2scan_registration_->register_frame(
        second_frame, utils::PoseUtils::poseVectorToIsometry(first_aligned_pose));

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
        if (visualization_enabled_) {
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
        }

        processing_thread_ = std::thread(&Pipeline::processingLoop, this);

        while (!stop_requested_) {
            if (visualization_enabled_ && async_viewer) {
                if (!async_viewer->spin_once()) {
                    stop_requested_ = true;
                    break;
                }
                handleVisualizationControls(async_viewer);
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

            auto frame = io::loadBIN_kitti(config_, scan_files_[i]);
            if (!processFrame(frame)) {
                stop_requested_ = true;
                break;
            }

            if (visualization_enabled_) {
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

bool Pipeline::processFrame(const small_gicp::PointCloud::Ptr &frame) {
    if (!frame || frame->empty()) {
        return false;
    }

    // Scan-to-scan registration
    auto scan2scan_result = scan2scan_registration_->register_frame(
        frame, utils::PoseUtils::poseVectorToIsometry(poses_[pose_index_]));

    if (!utils::PoseUtils::isMoving(scan2scan_result.T_target_source, poses_[pose_index_ - 1],
                                    config_.pipeline_.translation_threshold,
                                    config_.pipeline_.rotation_threshold)) {
        kitti_poses_.emplace_back(utils::PoseUtils::isometryToPoseVector(scan2scan_result.T_target_source));
        return true;
    }

    // Scan-to-map registration
    auto scan2map_result = scan2map_registration_->register_frame(
        frame, utils::PoseUtils::poseVectorToIsometry(poses_[pose_index_]));

    // Update pose graph
    updatePoseGraph(scan2map_result, scan2scan_result);

    // Optimize
    if (!pose_graph_->optimize()) {
        return false;
    }

    // Update scan2scan target with optimized pose
    scan2scan_registration_->get_map()->distance_insert(
        *frame, utils::PoseUtils::poseVectorToIsometry(poses_[pose_index_]));

    // Predict next pose
    addPose(predictNextPose());
    pose_index_++;

    kitti_poses_.emplace_back(poses_[pose_index_]);

    return true;
}

void Pipeline::writeResults() const {
    utils::FileUtils::writePosesToCSV(output_path_, kitti_poses_);
}

void Pipeline::updatePoseGraph(const small_gicp::RegistrationResult &scan2map_result,
                               const small_gicp::RegistrationResult &scan2scan_result) {
    pose_graph_->addConstraint(
        pose_index_ - 1, pose_index_,
        utils::PoseUtils::isometryToPoseVector(scan2scan_result.T_target_source), false);

    if (scan2map_result.num_inliers > 50) {
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
    if (!visualization_enabled_ || !cloud) {
        return;
    }

    auto async_viewer = guik::LightViewer::instance();
    if (!async_viewer) {
        return;
    }

    std::lock_guard<std::mutex> lock(visualization_mutex_);

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
    auto current_pose = poses_[pose_index_];
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

}  // namespace openlidarmap::pipeline
