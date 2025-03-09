#include "core/imu_data.hpp"
#include <sstream>
#include <fstream>

namespace openlidarmap {

IMUData IMUData::fromKITTI(const std::string& line) {
    std::istringstream iss(line);
    IMUData data;
    
    iss >> data.lat >> data.lon >> data.alt;
    iss >> data.roll >> data.pitch >> data.yaw;
    iss >> data.vn >> data.ve >> data.vf >> data.vl >> data.vu;
    iss >> data.ax >> data.ay >> data.az >> data.af >> data.al >> data.au;
    iss >> data.wx >> data.wy >> data.wz >> data.wf >> data.wl >> data.wu;
    iss >> data.pos_accuracy >> data.vel_accuracy;
    iss >> data.navstat >> data.numsats >> data.posmode >> data.velmode >> data.orimode;
    
    return data;
}

IMUData IMUData::fromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open IMU file: " + filename);
    }
    
    std::string line;
    if (std::getline(file, line) && !line.empty()) {
        return fromKITTI(line);
    }
    
    return IMUData();
}

} // namespace openlidarmap