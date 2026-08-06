#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

#include "System.h"
#include "rgbd-inertial-node.hpp"

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam3 rgbd-inertial path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    if (const char *working_directory = std::getenv("ORB_SLAM3_WORKING_DIRECTORY"))
    {
        if (chdir(working_directory) != 0)
        {
            std::cerr << "Cannot enter ORB-SLAM3 map directory: "
                      << working_directory << std::endl;
            return 2;
        }
    }

    std::string settings_path = argv[2];
    const char *load_atlas = std::getenv("ORB_SLAM3_LOAD_ATLAS");
    const char *save_atlas = std::getenv("ORB_SLAM3_SAVE_ATLAS");
    std::string generated_settings;
    if ((load_atlas && *load_atlas) || (save_atlas && *save_atlas))
    {
        generated_settings = "/tmp/bxi_orbslam3_settings_" +
            std::to_string(static_cast<long long>(getpid())) + ".yaml";
        std::ifstream input(settings_path);
        std::ofstream output(generated_settings);
        if (!input.good() || !output.good())
        {
            std::cerr << "Cannot prepare ORB-SLAM3 atlas settings" << std::endl;
            return 2;
        }
        std::string line;
        while (std::getline(input, line))
        {
            if (line.find("System.LoadAtlasFromFile") == std::string::npos &&
                line.find("System.SaveAtlasToFile") == std::string::npos)
                output << line << '\n';
        }
        if (load_atlas && *load_atlas)
            output << "System.LoadAtlasFromFile: \"" << load_atlas << "\"\n";
        if (save_atlas && *save_atlas)
            output << "System.SaveAtlasToFile: \"" << save_atlas << "\"\n";
        output.close();
        settings_path = generated_settings;
    }

    bool visualization = true;
    if (const char *viewer_env = std::getenv("ORB_SLAM3_VIEWER"))
    {
        const std::string viewer_value(viewer_env);
        visualization = viewer_value == "1" || viewer_value == "true" || viewer_value == "TRUE" || viewer_value == "on";
    }
    ORB_SLAM3::System::eSensor sensor = ORB_SLAM3::System::IMU_RGBD;
    if (const char *sensor_mode = std::getenv("ORB_SLAM3_SENSOR_MODE"))
    {
        const std::string value(sensor_mode);
        if (value == "rgbd" || value == "RGBD")
            sensor = ORB_SLAM3::System::RGBD;
    }
    ORB_SLAM3::System SLAM(argv[1], settings_path, sensor, visualization);
    if (const char *localization_only = std::getenv("ORB_SLAM3_LOCALIZATION_ONLY"))
    {
        const std::string value(localization_only);
        if (value == "1" || value == "true" || value == "TRUE")
            SLAM.ActivateLocalizationMode();
    }

    auto node = std::make_shared<RgbdInertialNode>(&SLAM);
    std::cout << "============================" << std::endl;

    rclcpp::spin(node);

    // The node owns the RGB-D synchronization thread. Destroy it first so no
    // TrackRGBD call can race with ORB-SLAM3 shutdown and atlas serialization.
    node.reset();
    rclcpp::shutdown();

    if (!generated_settings.empty())
        std::remove(generated_settings.c_str());

    return 0;
}
