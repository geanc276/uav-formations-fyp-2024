#ifndef COOPERATIVE_HPP
#define COOPERATIVE_HPP

#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#define COOP_QUEUE_SIZE 10

class Cooperative
{
private:
    rclcpp::Node::SharedPtr node_; // Node pointer for node functionality
    const std::string drone_name_;
    const std::string topic_name_;
    std::vector<std::string> other_drones_;
    bool value;
    std::unordered_map<std::string, std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>>> subscribers_;
    std::unordered_map<std::string, bool> subscribers_sync_states_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;

    bool log_coop_flag_;
    bool input_only_flag_;

    void handle_subscribe_message(const std::string &drone, const std_msgs::msg::Bool::SharedPtr &msg);
    void init_subs(void);
    void init_pubs(void);

public:
    Cooperative(rclcpp::Node::SharedPtr node, std::string drone_name, std::string topic_name);
    Cooperative(rclcpp::Node::SharedPtr node);
    ~Cooperative() = default;

    // Move constructor
    Cooperative(Cooperative &&other) noexcept;

    // Move assignment operator
    Cooperative &operator=(Cooperative &&other) noexcept;

    // Deleted copy constructor and assignment operator
    Cooperative(const Cooperative &) = delete;
    Cooperative &operator=(const Cooperative &) = delete;

    void add_drone_to_coop(std::string drone_name);
    void add_sub_(std::string drone);

    std::vector<std::string> get_other_drones(void);


    bool synchronise(void);
    void set_value(bool v);
    void reset(void);
    void set_log_flag(bool flag);
    void set_input_only(bool flag);
    bool get_input_only(void);
};

#endif // COOPERATIVE_HPP