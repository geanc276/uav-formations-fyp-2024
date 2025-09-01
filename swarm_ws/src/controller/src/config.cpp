/*
 *   Constraint class. Contains flight constraints for a drone.
 *
 *   File: constraints.cpp
 *   Date: 3-6-2024
 *   Author: Benjamin Ireland, Finlay Cross
 */

#include <vector>
#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include "controller/config.hpp"
#include "controller/global_pose.hpp"


//  RP CONFIG CONSTRUCTORS  //


RPConfig::RPConfig(const std::string &file_path)
{
    // const std::filesystem::path config_path{file_path};
    drone_constraints_ = Constraints(file_path);
}

RPConfig::RPConfig()
{
    drone_constraints_ = Constraints();
}

Constraints RPConfig::get_drone_constraint(void) const
{
    return drone_constraints_;
} 

RPConfig::~RPConfig()
{

}


//  CONSTRAINTS CONSTRUCTORS  //


Constraints::Constraints(const std::string &file_path)
{
    std::ifstream file(file_path);
    if (!file)
    {
        throw std::invalid_argument("Could not open file");
    }


    nlohmann::json config;

    try
    {
        file >> config;
    }
    catch (nlohmann::json::parse_error &e)
    {
        throw std::invalid_argument("Error parsing JSON file: " + std::string(e.what()));
    }

    try
    {
        // Drone information
        id_ = config["id"].get<int>();
        name_ = "drone_" + std::to_string(id_);

        // Define Altitude Information
        alt_.min_alt = config["alt_constraint"]["min_altitude"].get<double>();
        alt_.height_displacement = config["alt_constraint"]["height_displacement"].get<double>();

        // Define Position Information
        position_.dist = config["pos_constraint"]["distance"].get<double>();
        position_.pos_angle = config["pos_constraint"]["angle"].get<double>();
        position_.dist_x = position_.dist * cos(DEG_TO_RAD(position_.pos_angle));
        position_.dist_y = position_.dist * sin(DEG_TO_RAD(position_.pos_angle));

        // Define Heading Information
        heading_ = config["heading_constraint"]["angle"].get<double>();

        // Gains
        std::array<double, 3> temp_alt_gains = config["alt_constraint"]["gains"].get<std::array<double, 3>>();
        alt_gains_ = control_toolbox::Pid::Gains(temp_alt_gains[0], temp_alt_gains[1], temp_alt_gains[2],
                                                 INTEGRAL_MAX, INTEGRAL_MIN, ANTI_WINDUP);

        std::array<double, 3> temp_pos_gains = config["pos_constraint"]["gains"].get<std::array<double, 3>>();
        pos_gains_ = control_toolbox::Pid::Gains(temp_pos_gains[0], temp_pos_gains[1], temp_pos_gains[2],
                                                 INTEGRAL_MAX, INTEGRAL_MIN, ANTI_WINDUP);

        std::array<double, 3> temp_ang_gains = config["heading_constraint"]["gains"].get<std::array<double, 3>>();
        ang_gains_ = control_toolbox::Pid::Gains(temp_ang_gains[0], temp_ang_gains[1], temp_ang_gains[2],
                                                 INTEGRAL_MAX, INTEGRAL_MIN, ANTI_WINDUP);
    }
    catch (nlohmann::json::exception &e)
    {
        throw std::invalid_argument("Error accessing JSON fields: " + std::string(e.what()));
    }
}

Constraints::Constraints()
{
    id_ = 0;
    alt_ = {0, 0};
    position_ = {0, 0, 0, 0, 0};
    heading_ = 0;
    alt_gains_ = control_toolbox::Pid::Gains(0, 0, 0, INTEGRAL_MAX, INTEGRAL_MIN, ANTI_WINDUP);
    pos_gains_ = control_toolbox::Pid::Gains(0, 0, 0, INTEGRAL_MAX, INTEGRAL_MIN, ANTI_WINDUP);
    ang_gains_ = control_toolbox::Pid::Gains(0, 0, 0, INTEGRAL_MAX, INTEGRAL_MIN, ANTI_WINDUP);
}


//  GETTERS/SETTERS  //


control_toolbox::Pid::Gains Constraints::get_alt_gains(void) const
{
    return alt_gains_;
}

control_toolbox::Pid::Gains Constraints::get_pos_gains(void) const
{
    return pos_gains_;
}

control_toolbox::Pid::Gains Constraints::get_ang_gains(void) const
{
    return ang_gains_;
}

int Constraints::get_id(void) const
{
    return id_;
}

altitude_config_t Constraints::get_alt_cnstr(void) const
{
    return alt_;
}

position_config_t Constraints::get_pos_cnstr(void) const
{
    return position_;
}

double Constraints::get_heading_cnstr(void) const
{
    return heading_;
}

control_toolbox::Pid::Gains Constraints::get_gains(pid_type_t type) const
{
    control_toolbox::Pid::Gains gains;
    switch (type)
    {
    case ALTITUDE:
        gains = alt_gains_;
        break;
    case POSITION:
        gains = pos_gains_;
        break;
    case ANGULAR:
        gains = ang_gains_;
        break;
    default:
        // Handle unexpected cases, possibly by throwing an exception or setting default gains
        throw std::invalid_argument("Unknown PID type");
    }
    return gains;
}


//  DESTRUCTOR  //


Constraints::~Constraints() = default;