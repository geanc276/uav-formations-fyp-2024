/*
 *  PID manager utilizes the ros2_control library to run three PID controllers
 *  that maintain the drones configured position, altitude, and heading
 *  w.r.t the reference point.
 *
 *   File: pid_manager.hpp
 *   Date: 10-7-2024
 *   Author: Benjamin Ireland, Finlay Cross
 */

#ifndef PID_MANAGER_HPP
#define PID_MANAGER_HPP

#include <string>
#include <array>
#include <rclcpp/rclcpp.hpp>
#include "control_toolbox/pid.hpp"
#include "config.hpp"
#include "controller/global_pose.hpp"
#include "controller/local_pose.hpp"
#include "controller/pose.hpp"

class PidManager
{
public:
    PidManager(const Constraints &constraints);
    PidManager();
    void log_error(double error, std::string pid_name);

    void set_gains(struct control_toolbox::Pid::Gains gains, pid_type_t type);

    control_toolbox::Pid::Gains get_gains(pid_type_t type);

    std::array<double, 7> get_corrections(void);

    LocalPose get_error(LocalPose &drone_pose, LocalPose &goal_pose);
    void find_goal_position(LocalPose &goal_pose, LocalPose &ref_point_local_pos);
    double get_heading_error(GlobalPose &drone_pose, GlobalPose &ref_point);

    void update_pids(LocalPose &drone_pose, LocalPose &goal_pose);

    ~PidManager();

private:
    // MEMBER VARIABLES //
    rclcpp::Logger logger_;
    rclcpp::Clock clock_;
    rclcpp::Time prev_time_;
    Constraints constraints_;
    control_toolbox::Pid position_pid_;
    control_toolbox::Pid altitude_pid_;
    control_toolbox::Pid heading_pid_;
    std::array<double, 7> corrections_ = {0, 0, 0, 0, 0, 0, 0};

    // CLASS METHODS //
    void get_altitude_correction(LocalPose error_vector, long delta_t);
    void get_position_correction(LocalPose error_vector, long delta_t);
    void get_heading_correction(LocalPose error_vector, long delta_t);
};

#endif // PID_MANAGER_HPP