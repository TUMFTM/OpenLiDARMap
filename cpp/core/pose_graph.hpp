// core/pose_graph.hpp
#pragma once

#include "config/config.hpp"
#include "config/types.hpp"
#include "core/imu_data.hpp"
#include <ceres/ceres.h>
#include <deque>
#include <utility>

namespace openlidarmap {
class PoseGraph {
public:
    explicit PoseGraph(const config::Config &config, std::deque<Vector7d> &poses);

    void addConstraint(size_t idx_from, size_t idx_to, const Vector7d &measurement, bool absolute);
    void addIMUConstraint(size_t idx, const IMUData& imu_data);
    bool optimize();

private:
    void manageSlidingWindow();

    ceres::Problem problem_{};
    ceres::Solver::Options solver_options_{};
    const config::Config &config_;
    std::deque<Vector7d> &poses_;
    std::deque<size_t> window_indices_{};
    ceres::Manifold *pose_manifold_{};
    ceres::Solver::Summary summary_{};
    std::deque<std::pair<ceres::ResidualBlockId, ceres::ResidualBlockId>> constraints_{};
    std::vector<ceres::ResidualBlockId> imu_constraints_{};
    int current_fixed_pose_{-1};
};

}  // namespace openlidarmap