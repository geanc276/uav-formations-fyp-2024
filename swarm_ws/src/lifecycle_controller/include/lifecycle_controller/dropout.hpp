#ifndef DROPOUT_HPP
#define DROPOUT_HPP

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <boost/asio.hpp>
#include <string>
#include <unordered_map>
#include <vector>
#include <thread>

#include <std_srvs/srv/trigger.hpp>

class Dropout
{
public:
    // Dropout(std::shared_ptr<rclcpp::Node> node, std::string drone_name, std::vector<std::string> other_drones, double dropout_time_threshold);
    Dropout(std::shared_ptr<rclcpp::Node> node, std::string drone_name, std::vector<std::string> other_drones, int dropout_time_threshold, int check_interval);
    ~Dropout();
    bool getDropoutFlag();
    void setLogDropouts(bool log_flag);

private:
    void checkForDropouts();
    void initSubs();
    void initServiceClients();
    void dropoutCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg, const std::string &drone);
    void initService();
    void handleTrigger(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void handleDropout(const std::string &drone);

    std::shared_ptr<rclcpp::Node> node_;
    std::string drone_name_;
    std::vector<std::string> other_drones_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> subscribers_;
    std::unordered_map<std::string, double> log_times_;
    rclcpp::TimerBase::SharedPtr dropout_check_timer_;
    const int dropout_time_threshold_;
    int check_interval_;
    std::unordered_map<std::string, bool> dropout_flags_;
    bool dropout_flag_;

    bool log_flag_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;

    boost::asio::io_context io_context_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_;
    std::thread io_thread_;
    std::unordered_map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> alive_check_clients_;
};

#endif // DROPOUT_HPP