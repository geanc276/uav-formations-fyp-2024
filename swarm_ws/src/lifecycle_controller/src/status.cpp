#include "lifecycle_controller/status.hpp"




Status::Status(std::string name, int id, std::string state)
{
    name_ = name;
    id_ = id;
    state_ = state;
    offboard_ = 0;
    mode_ = 0;
    armed_ = 0;
    battery_percentage_ = 0;
    battery_voltage_ = 0;
    battery_current_ = 0;
}

Status::Status()
{
    name_ = "";
    id_ = 0;
    state_ = "";
    offboard_ = 0;
    mode_ = 0;
    armed_ = 0;
    battery_percentage_ = 0;
    battery_voltage_ = 0;
    battery_current_ = 0;
}

Status::~Status()
{
}


void Status::set_publisher(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher)
{
    status_publisher_ = publisher;
}


void Status::set_state(std::string state)
{
    state_ = state;
}

void Status::set_nav_state(uint8_t nav_state)
{
    mode_ = nav_state;
}

void Status::set_armed(uint8_t armed)
{
    armed_ = armed;
}

void Status::set_battery_percentage(float_t percentage)
{
    battery_percentage_ = percentage;
}

void Status::set_battery_voltage(float_t voltage)
{
    battery_voltage_ = voltage;
}

void Status::set_battery_current(float_t current)
{
    battery_current_ = current;
}

void Status::print_status(void)
{
    std::cout << "Drone: " << name_ << std::endl;
    std::cout << "ID: " << id_ << std::endl;
    std::cout << "State: " << state_ << std::endl;
    std::cout << "Offboard: " << offboard_ << std::endl;
    std::cout << "Mode: " << mode_ << std::endl;
    std::cout << "Armed: " << armed_ << std::endl;
    std::cout << "Battery Percentage: " << battery_percentage_ << std::endl;
    std::cout << "Battery Voltage: " << battery_voltage_ << std::endl;
    std::cout << "Battery Current: " << battery_current_ << std::endl;
}


nlohmann::json Status::get_status(void)
{
    nlohmann::ordered_json status;
    status["name"] = name_;
    status["id"] = id_;
    status["state"] = state_;
    status["offboard"] = offboard_;
    status["mode"] = mode_;
    status["armed"] = armed_;
    status["battery_percentage"] = battery_percentage_;
    status["battery_voltage"] = battery_voltage_;
    status["battery_current"] = battery_current_;
    return status;
}


void Status::publish_status(void)
{
    auto msg = std_msgs::msg::String();
    msg.data = get_status().dump();
    status_publisher_->publish(msg);
}

void Status::update_vehicle_status(uint8_t nav_state, uint8_t armed)
{
    mode_ = nav_state;
    armed_ = armed;
}

void Status::update_battery_status(float_t percentage, float_t voltage, float_t current)
{
    battery_percentage_ = percentage;
    battery_voltage_ = voltage;
    battery_current_ = current;
}

