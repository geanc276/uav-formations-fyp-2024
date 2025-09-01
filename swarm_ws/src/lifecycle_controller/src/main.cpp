#include <rclcpp/rclcpp.hpp>
#include "lifecycle_controller/lifecycle_class.hpp"
#include "lifecycle_controller/config_reader.hpp"
#include "lifecycle_controller/input.hpp"
#include "lifecycle_controller/action_map.hpp"

#include "logging/logging.hpp"




#include <iostream>
#include <unistd.h>
#include <limits.h>
#include <stdexcept>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    char hostname[256];
    gethostname(hostname, sizeof(hostname));
    std::string hostname_str(hostname);
    std::string name = hostname_str;

    u_int16_t id;
    std::ifstream config_file("/etc/drone_config.conf");
    if (!config_file.is_open())
    {
        throw std::runtime_error("Unable to open /etc/drone_config.conf");
    }
    std::string line;
    while (std::getline(config_file, line))
    {
        if (line.find("DRONE_ID=") == 0)
        {
            
            id = std::stoi(line.substr(9));
        }
    }
    config_file.close();

    printf("Drone name: %s\n", name.c_str());

    try
    {
        // other_drones.erase(std::remove(other_drones.begin(), other_drones.end(), name), other_drones.end());

        // auto lifecycle = Lifecycle::create(name, id);
        auto lifecycle = Lifecycle::create(name, id);
        printf("Lifecycle created\n");
        auto user_input = std::make_shared<UserInput>(lifecycle);
        printf("UserInput created\n");
        auto action_map = std::make_shared<ActionMap>(lifecycle, lifecycle->get_state_config_file_path());
        printf("ActionMap created\n");

        LOG_MISSION_INFO(lifecycle,"Lifecycle Controller for %s", name.c_str());
        LOG_MISSION_INFO(lifecycle,"Lifecycle Controller for %s", name.c_str());
        LOG_MISSION_INFO(lifecycle,"Lifecycle Controller for %s", name.c_str());
        LOG_MISSION_INFO(lifecycle,"Lifecycle Controller for %s", name.c_str());

        rclcpp::Rate rate(15);
        LOG_SETUP_INFO(lifecycle, "RATE SET");
        RCLCPP_DEBUG(lifecycle->get_logger(), "RATE SET %s", name.c_str());

        while (!rclcpp::ok())
        {
            LOG_SETUP_DEBUG(lifecycle, "Waiting for rclcpp to be ok");
            usleep(1000000);
        }
        while (rclcpp::ok())
        {
            LOG_SETUP_DEBUG(lifecycle, "Running lifecycle controller for drone %s", name.c_str());
            lifecycle->run();
            rclcpp::spin_some(lifecycle);
            rate.sleep();
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error initializing lifecycle controller: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}