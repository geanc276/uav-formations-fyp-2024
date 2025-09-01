#ifndef USER_INPUT_HPP
#define USER_INPUT_HPP

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <stdbool.h>
#include <string>
#include <stdint.h>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <controller_msgs/msg/pid_set.hpp>
#include <controller_msgs/msg/state_set.hpp>
#include <controller_msgs/msg/error_threshold_set.hpp>

#include <lifecycle_controller/lifecycle_class.hpp>
class Lifecycle;

//! THIS IS PID TYPE ENUM DEFINED IN CONTROLLER -- HERE AS TEMPORARY FIX
// ! IF NOT FIXED AT LEAST REMEMBER TO CHANGE ALONG SIDE
typedef enum
{
    ALTITUDE,
    POSITION,
    HEADING,
    ANGULAR
} pid_type_t;

class UserInput
{
private:
    std::shared_ptr<Lifecycle> lifecycle_;
    std::string drone_name_;
    // inits
    void init_subs();
    void init_pubs();
    // subs
    // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;

    rclcpp::Subscription<controller_msgs::msg::StateSet>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr coop_launch_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr coop_formation_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr coop_return_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;

    rclcpp::Subscription<controller_msgs::msg::ErrorThresholdSet>::SharedPtr error_threshold_sub_;
    rclcpp::Subscription<controller_msgs::msg::PidSet>::SharedPtr PID_sub_;

    // pubs
    // rclcpp::Publisher<controller_msgs::msg::ErrorThresholdSet>::SharedPtr error_threshold_pub_;
    rclcpp::Publisher<controller_msgs::msg::PidSet>::SharedPtr PID_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_ack_pub_;

    // callbacks
    void state_cb(const controller_msgs::msg::StateSet::SharedPtr msg);
    void coop_launch_cb(const std_msgs::msg::Bool::SharedPtr msg);
    void coop_formation_cb(const std_msgs::msg::Bool::SharedPtr msg);
    void coop_return_cb(const std_msgs::msg::Bool::SharedPtr msg);
    void command_cb(const std_msgs::msg::String::SharedPtr msg);

    void error_threshold_cb(const controller_msgs::msg::ErrorThresholdSet::SharedPtr msg);
    void PID_cb(const controller_msgs::msg::PidSet::SharedPtr msg);

    std::string user_input_;

public:
    UserInput(std::shared_ptr<Lifecycle> lifecycle);
};

class Command
{
};

#endif // USER_INPUT_HPP