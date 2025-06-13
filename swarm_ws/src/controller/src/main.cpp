#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <fstream>
#include <unistd.h>



#include "controller/controller_node.hpp"

/**
 *  @brief  ROS 2 controller node set up.
 */
int main(int argc, char **argv)
{



    char hostname[256];
    gethostname(hostname, sizeof(hostname));
    std::string hostname_str(hostname);
    std::string name = hostname_str;

    int id;
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
    

    rclcpp::init(argc, argv);
    // Create a ROS node
    std::shared_ptr<ControllerNode> controller_node;
    controller_node = std::make_shared<ControllerNode>(id);
    // Start the ROS executor
    rclcpp::spin(controller_node);

    rclcpp::shutdown();
    return 0;
}
