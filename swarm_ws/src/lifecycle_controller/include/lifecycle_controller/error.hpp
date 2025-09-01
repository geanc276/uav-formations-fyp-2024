#ifndef ERROR_HPP
#define ERROR_HPP

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class Error
{
private:
    rclcpp::Node::SharedPtr node_; // Node pointer for node functionality
    std::string drone_name_;
    std::string topic_name_;
    std::map<std::string, rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr> error_subscribers_;
    rclcpp::TimerBase::SharedPtr convergence_timer_;
    std::map<std::string, double> error_targets_;
    std::map<std::string, double> epsilons_;
    std::map<std::string, bool> converged_values_;
    bool topic_converged_;

    void init_subs();
    void start_timer();
    void error_cb(const std::string &topic, const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void convergence_cb();

public:
    Error(rclcpp::Node::SharedPtr node, std::string, std::string, std::map<std::string, double>);
    Error(rclcpp::Node::SharedPtr node, std::string);
    Error(rclcpp::Node::SharedPtr node);

    ~Error();

    void init_error(std::string topic_name, std::map<std::string, double> error_targets);

    bool are_all_true();

    void reset_timer();
    bool converged();
};

#endif // ERROR_HPP