/*
 *  Controller Node. Used to control the flight of the drone
 *  based on the state of the drones lifecycle and the position of the reference point.
 *
 *  File: controller_node.hpp
 *  Date: 10-7-2024
 *  Author: Benjamin Ireland
 */

#ifndef CONTROLLER_NODE_HPP
#define CONTROLLER_NODE_HPP

// MACROS //
#define CONTROLLER_QUEUE_SIZE 10
#define COUNT_THRESHOLD 40
#define MAX_COUNT 80
// Error Thresholds in meters
#define POS_ERR_THRESHOLD_DEFAULT 2
#define ALT_ERR_THRESHOLD_DEFAULT 2
#define HEAD_ERR_THRESHOLD_DEFAULT 10
#include <vector>



#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/mode_completed.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/home_position.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>

#include <controller_msgs/msg/state_set.hpp>
#include <controller_msgs/msg/error_threshold_set.hpp>
#include <controller_msgs/msg/pid_set.hpp>
#include <controller_msgs/msg/ref_point_global_position.hpp>

#include "controller/pid_manager.hpp"
#include "controller/pose.hpp"
#include "controller/state.hpp"
#include "logging/logging.hpp"


class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode(const uint32_t id);
    
    void init_ros2();
    void run_PID();
    void rendezvous();
    void set_PID_gains(control_toolbox::Pid::Gains gains, pid_type_t type);
    void set_error_thresholds_cb(controller_msgs::msg::ErrorThresholdSet::SharedPtr msg);
    void set_error_thresholds(double pos_err, double alt_err, double head_err);
    state_t get_state() const;
    LocalPose convert_global_to_local(GlobalPose& global, const GlobalPose& origin);

    ~ControllerNode();

private:

    // ROS 2 CALLBACKS //
    // PX4 messages
    void status_cb(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void ctrl_state_cb(const std_msgs::msg::UInt8::SharedPtr msg);
    void global_pos_cb(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
    void local_pos_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void vehicle_command_ack_cb(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg);

    // Custom/standard ROS 2 messages
    void rp_pos_cb(const controller_msgs::msg::RefPointGlobalPosition::SharedPtr msg);
    void rp_established_cb(const std_msgs::msg::Bool::SharedPtr msg);
    void set_PID_cb(const controller_msgs::msg::PidSet::SharedPtr msg);
    
    
    void log_positions(const std::vector<Pose> &poses) const;

    // Overloaded function for variadic arguments
    template <typename... Poses>
    void log_positions(const Pose &first, const Poses &...rest) const;


    // PUBLISHERS //
    void pub_rendezvous_complete();
    void pub_rendezvous_incomplete();
    void publish_offboard_control_mode();
    void publish_vehicle_command(uint32_t command, float param1, float param2, float param3, float param4, float param5, float param6, float param7);
    bool in_position();

    // TIMER CALLBACKS //
    void control_loop();

    // MEMBER VARIABLES //
    int drone_id_;
    state_t state_;
    bool rp_established_;
    bool origin_set_;
    px4_msgs::msg::VehicleStatus drone_status_;
    double altitude_ = 0.0;

    double pos_err_threshold_ = POS_ERR_THRESHOLD_DEFAULT;
    double alt_err_threshold_ = ALT_ERR_THRESHOLD_DEFAULT;
    double head_err_threshold_ = HEAD_ERR_THRESHOLD_DEFAULT;




    // POSE VARIABLES //
    GlobalPose drone_pose_global_;
    LocalPose drone_pose_local_;
    GlobalPose ref_point_pose_global_;
    LocalPose ref_point_pose_local_;
    GlobalPose origin_;
    PidManager PID_;
    Constraints drone_constraints_;

    // SUBSCRIBERS //
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<controller_msgs::msg::RefPointGlobalPosition>::SharedPtr reference_point_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_drone_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_drone_pos_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr ctrl_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rp_established_sub_;
    
    rclcpp::Subscription<controller_msgs::msg::ErrorThresholdSet>::SharedPtr error_threshold_set_sub_;
    rclcpp::Subscription<controller_msgs::msg::PidSet>::SharedPtr pid_set_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicle_command_ack_sub_;

    // PUBLISHERS //
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rendezvous_status_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;

    // TIMERS //
    rclcpp::TimerBase::SharedPtr control_loop_timer_;
};

#endif // CONTROLLER_NODE_HPP
