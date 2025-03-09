#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

namespace openlidarmap {

struct IMUData {
    double timestamp = 0.0;
    
    // Position (from GPS)
    double lat = 0.0, lon = 0.0, alt = 0.0;
    
    // Orientation (roll, pitch, yaw in radians)
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    
    // Linear velocities
    double vn = 0.0, ve = 0.0, vf = 0.0, vl = 0.0, vu = 0.0;
    
    // Accelerations
    double ax = 0.0, ay = 0.0, az = 0.0, af = 0.0, al = 0.0, au = 0.0;
    
    // Angular rates
    double wx = 0.0, wy = 0.0, wz = 0.0, wf = 0.0, wl = 0.0, wu = 0.0;
    
    // Accuracy and status
    double pos_accuracy = 0.0, vel_accuracy = 0.0;
    int navstat = -1, numsats = 0;
    int posmode = -1, velmode = -1, orimode = -1;
    
    // Convenience functions
    Eigen::Vector3d getAcceleration() const { return Eigen::Vector3d(ax, ay, az); }
    Eigen::Vector3d getAngularVelocity() const { return Eigen::Vector3d(wx, wy, wz); }
    Eigen::Vector3d getRPY() const { return Eigen::Vector3d(roll, pitch, yaw); }
    
    static IMUData fromKITTI(const std::string& line);
    static IMUData fromFile(const std::string& filename);
};
} // namespace openlidarmap
