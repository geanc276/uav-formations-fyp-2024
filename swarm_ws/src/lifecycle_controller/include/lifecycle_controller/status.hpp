#ifndef STATUS_HPP
#define STATUS_HPP
#include <iostream>
#include <string>
#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>


class Status
{
private:
    std::string name_;
    int id_;
    std::string state_;
    uint8_t offboard_;
    uint8_t mode_;
    uint8_t armed_;

    float_t battery_percentage_;
    float_t battery_voltage_;
    float_t battery_current_;



    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;

public:
    Status(std::string name, int id, std::string state);
    Status();
    ~Status();

    void set_publisher(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher);

    void print_status(void);

    nlohmann::json get_status(void);


    void publish_status(void);

    void set_state(std::string state);
    void update_vehicle_status(uint8_t nav_state, uint8_t armed);

    void update_battery_status(float_t percentage,float_t voltage,float_t current);



    void set_nav_state(uint8_t);
    void set_armed(uint8_t);

    void set_battery_percentage(float_t);
    void set_battery_voltage(float_t);
    void set_battery_current(float_t);

};

#endif