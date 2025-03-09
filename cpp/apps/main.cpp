#include <iostream>

#include "config/config.hpp"
#include "pipeline/openlidarmap.hpp"

void printUsage() {
    std::cout << "Usage: ./OpenLiDARMap <map_path> <scans_dir> <output_path> "
              << "[x y z qx qy qz qw]" << std::endl;
}

int main(int argc, char **argv) {
    if (argc != 4 && argc != 11) {
        printUsage();
        return 1;
    }

    std::string map_path = argv[1];
    std::string scans_dir = argv[2];
    std::string output_path = argv[3];
    std::string imu_dir = ""; //"/media/Volume/kitti_raw/2011_10_03_drive_0027_sync/oxts/data";

    // Default or provided initial pose
    openlidarmap::Vector7d initial_pose{};
    if (argc == 11) {
        for (int i = 0; i < 7; ++i) {
            initial_pose[i] = std::stod(argv[i + 4]);
        }
    } else {
        initial_pose << 0, 0, 0, 0, 0, 0, 1;
    }

    // Create configuration
    openlidarmap::config::Config config{};

    // Create and run pipeline
    auto pipeline = std::make_unique<openlidarmap::pipeline::Pipeline>(config);
    if (!pipeline->initialize(map_path, scans_dir, output_path, imu_dir, initial_pose)) {
        std::cerr << "Failed to initialize pipeline" << std::endl;
        return 1;
    }

    if (!pipeline->run()) {
        std::cerr << "Pipeline execution stopped" << std::endl;
        return 1;
    }

    return 0;
}
