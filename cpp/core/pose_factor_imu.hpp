#pragma once

#include "config/types.hpp"
#include "core/imu_data.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace openlidarmap {

struct IMUError {
    IMUError(const IMUData& imu_data) : imu_data_(imu_data) {}

    template <typename T>
    bool operator()(const T* const pose, T* residual) const {       
        constexpr double ROT_STDDEV = 1.0;

        // Extract rotation from the pose quaternion
        Eigen::Quaternion<T> q_pose(pose[6], pose[3], pose[4], pose[5]);
        q_pose.normalize();
        
        // Convert IMU roll, pitch, yaw to quaternion
        const T roll = T(imu_data_.roll);
        const T pitch = T(imu_data_.pitch);
        const T yaw = T(imu_data_.yaw);
        
        // Create rotation matrices for roll, pitch, yaw
        Eigen::AngleAxis<T> rollAngle(roll, Eigen::Matrix<T, 3, 1>::UnitX());
        Eigen::AngleAxis<T> pitchAngle(pitch, Eigen::Matrix<T, 3, 1>::UnitY());
        Eigen::AngleAxis<T> yawAngle(yaw, Eigen::Matrix<T, 3, 1>::UnitZ());
        
        // Combine rotations
        Eigen::Quaternion<T> q_imu = yawAngle * pitchAngle * rollAngle;
        q_imu.normalize();
        
        // Compute rotation error
        Eigen::Quaternion<T> q_error = q_pose * q_imu.inverse();
        q_error.normalize();
        
        // First 3 residuals not used (we don't use IMU position)
        residual[0] = T(0);
        residual[1] = T(0);
        residual[2] = T(0);
        
        // Rotation error (just use roll and pitch from IMU, yaw can drift)
        residual[3] = T(2.0) * q_error.x() / T(ROT_STDDEV);
        residual[4] = T(2.0) * q_error.y() / T(ROT_STDDEV);
        residual[5] = T(2.0) * q_error.z() / T(ROT_STDDEV);
        
        return true;
    }

private:
    const IMUData imu_data_;
};
} // namespace openlidarmap