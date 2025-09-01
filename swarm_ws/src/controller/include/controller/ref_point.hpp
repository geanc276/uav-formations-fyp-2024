/*
 *   The reference point node. Controls the location and movement of the
 *   virtual reference point at the centre of the swarm formation.
 *
 *   File: ref_point.hpp
 *   Date: 3-7-2024
 *   Author: Benjamin Ireland, Finlay Cross
 */

#ifndef REF_POINT_HPP
#define REF_POINT_HPP

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <controller_msgs/msg/ref_point_global_position.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

#include "controller/config.hpp"
#include "controller/global_pose.hpp"
#include "controller/state.hpp"
#include "controller/pose.hpp"


#include <controller_msgs/msg/register_drone.hpp>
#include <controller_msgs/msg/register_drone_ack.hpp>
#include <controller_msgs/msg/update_drone_list.hpp>


#define DEFAULT_DRONE_NUM 1
#define RP_QUEUE_SIZE 10

class ReferencePoint : public rclcpp::Node
{
public:
    // Instantiates reference point and establishes the position of the reference point.
    ReferencePoint();
    bool get_rp_established();

    GlobalPose get_drone_position(int drone_id);

    ~ReferencePoint();

private:
    void get_parameters();
    void validate_parameters();
    void init();
    // Set up ros2 publishers and subscribers.
    void init_ros2();

    /* CALLBACKS */
    // Updates global position of a given drone.
    void pos_cb(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
    void ctrl_state_cb(const std_msgs::msg::UInt8::SharedPtr msg);
    // Callback that superimposes reference point over the target for tracking.
    void target_cb(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);

    /* PUBLISHERS */
    void publish_rp_pos();

    void publish_home();
    void publish_rp_established();

    void drone_register_cb(const controller_msgs::msg::RegisterDrone::SharedPtr msg);
    void drone_register_ack(int id);

    void update_drone_list(void);

    std::unordered_map<int, std::string> drone_list_;

    // Establishes reference point from given drone position on startup.
    void set_rp(GlobalPose drone_pos, Constraints drone_constraint);


    /* Subs */
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_drone_pos_subscriber_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr ctrl_state_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr target_sub_;
    rclcpp::Subscription<controller_msgs::msg::RegisterDrone>::SharedPtr register_drone_sub_;

    /* Pubs */
    rclcpp::Publisher<controller_msgs::msg::RefPointGlobalPosition>::SharedPtr global_ref_point_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rp_establish_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr ref_point_home_pub_;
    rclcpp::Publisher<controller_msgs::msg::RegisterDroneAck>::SharedPtr register_drone_ack_pub_;
    rclcpp::Publisher<controller_msgs::msg::UpdateDroneList>::SharedPtr update_drone_list_pub_;
   
   
    /* Timers */
    rclcpp::TimerBase::SharedPtr rp_established_pub_timer_;
    rclcpp::TimerBase::SharedPtr global_rp_position_pub_timer_;

    GlobalPose global_drone_pose_;
    GlobalPose target_pos_;

    GlobalPose rp_pos_;

    bool rp_established_;

    state_t state_;

    RPConfig config_;
    std::string config_file_path_;
    uint32_t num_of_drones_;
    uint32_t drone_id_;
};

#endif // REF_POINT_HPP 