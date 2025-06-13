/*
 *  Controller Node. Used to control the flight of the drone
 *  based on the state of the drones lifecycle and the position of the reference point.
 *  Contains entry point for the controller process. 
 *
 *  File: controller_node.cpp
 *  Date: 10-7-2024
 *  Author: Benjamin Ireland
 */

#include <string>
#include <vector>
#include <chrono>
#include <nlohmann/json.hpp>  

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

// #include <rclcpp/rclcpp.hpp>

#include <controller_msgs/msg/pid_set.hpp>
#include <controller_msgs/msg/ref_point_global_position.hpp>
#include "controller/pid_manager.hpp"
#include "controller/pose.hpp"
#include "controller/state.hpp"
#include "controller/controller_node.hpp"
#include "logging/logging.hpp"

using namespace std::chrono_literals;
using json = nlohmann::json;
#define LOOP_SPEED 50ms // 20 Hz

// Add an offset to the target system for the simulation as required by px4_sitl.
#ifndef SYSTEM_OFFSET
#define SYSTEM_OFFSET 0
#endif


//   CONSTRUCTORS   //


ControllerNode::ControllerNode( const uint32_t id)
    : Node("drone_" +std::to_string(id) +"_controller_node")
{
    std::string config_file_path;
    config_file_path = "";
    this->declare_parameter<std::string>("config_file_path");
    this->get_parameter("config_file_path", config_file_path);
    Constraints constraints(config_file_path);

    drone_constraints_ = constraints;
    PID_ = PidManager(constraints);
 
 
    state_ = LANDED;
    drone_status_ = px4_msgs::msg::VehicleStatus();
    // drone_id_ = constraints.get_id();
    drone_id_= id;
    drone_pose_global_ = GlobalPose("drone_global");
    drone_pose_local_ = LocalPose("drone_local");
    ref_point_pose_global_ = GlobalPose("ref_point_global");
    ref_point_pose_local_ = LocalPose("ref_point_local");
    origin_ = GlobalPose();

    rp_established_ = false;
    origin_set_ = false;

    this->init_ros2();
    LOG_SETUP_DEBUG(this, "Controller node initialised for drone %d", drone_id_);
}

/**
 *  @brief  Helper function to initialise ros2 publishers and subscribers.
 */
void ControllerNode::init_ros2()
{
    /* Quality of Service set-up for PX4 topics. */
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    std::string drone_id_str = std::to_string(drone_id_);

    /* Subscribers */
    status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/px4_" + drone_id_str + "/fmu/out/vehicle_status", qos,
        std::bind(&ControllerNode::status_cb, this, std::placeholders::_1));

    ctrl_state_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/ctrl/state", CONTROLLER_QUEUE_SIZE, std::bind(&ControllerNode::ctrl_state_cb, this, std::placeholders::_1));

    global_drone_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        "/px4_" + drone_id_str + "/fmu/out/vehicle_global_position", qos,
        std::bind(&ControllerNode::global_pos_cb, this, std::placeholders::_1));

    vehicle_command_ack_sub_ = this->create_subscription<px4_msgs::msg::VehicleCommandAck>(
        "/px4_" + drone_id_str + "/fmu/out/vehicle_command_ack", qos,
        std::bind(&ControllerNode::vehicle_command_ack_cb, this, std::placeholders::_1));

    local_drone_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/px4_" + drone_id_str + "/fmu/out/vehicle_local_position", qos,
        std::bind(&ControllerNode::local_pos_cb, this, std::placeholders::_1));

    reference_point_sub_ = this->create_subscription<controller_msgs::msg::RefPointGlobalPosition>(
        "/reference_point/global_position", qos,
        std::bind(&ControllerNode::rp_pos_cb, this, std::placeholders::_1));

    rp_established_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/reference_point/rp_set", qos,
        std::bind(&ControllerNode::rp_established_cb, this, std::placeholders::_1));


    error_threshold_set_sub_ = this->create_subscription<controller_msgs::msg::ErrorThresholdSet>(
        "/ctrl/error_threshold_set", CONTROLLER_QUEUE_SIZE,
        std::bind(&ControllerNode::set_error_thresholds_cb, this, std::placeholders::_1));

    pid_set_sub_ = this->create_subscription<controller_msgs::msg::PidSet>(
        "/drone_" + drone_id_str + "/ctrl/pid_set", CONTROLLER_QUEUE_SIZE,
        std::bind(&ControllerNode::set_PID_cb, this, std::placeholders::_1));

    /* Publishers */
    trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/px4_" + drone_id_str + "/fmu/in/trajectory_setpoint", CONTROLLER_QUEUE_SIZE);

    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/px4_" + drone_id_str + "/fmu/in/vehicle_command", CONTROLLER_QUEUE_SIZE);

    rendezvous_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/px4_" + drone_id_str + "/rendezvous_status", CONTROLLER_QUEUE_SIZE);

    offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/px4_" + drone_id_str + "/fmu/in/offboard_control_mode", CONTROLLER_QUEUE_SIZE);

    /* Timers */
    control_loop_timer_ = this->create_wall_timer(LOOP_SPEED, std::bind(&ControllerNode::control_loop, this));
}


// GETTERS/SETTERS //

/**
 *  @brief  Takes user inputed gain values and applies them to the drone swarm.
 * 
 *  @param Gains   Represents P, I, D gain values.
 *  @param pid_type_t   Enum representing pid type to set.
 */
void ControllerNode::set_PID_gains(control_toolbox::Pid::Gains gains, pid_type_t type)
{

    PID_.set_gains(gains, type);
}

state_t ControllerNode::get_state() const
{
    return state_;
}


// CALLBACKS //


void ControllerNode::local_pos_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    LOG_MISSION_DEBUG(this, "%s", "LOCAL_POSE CALL BACK FOR DRONE");
    drone_pose_local_.set_position(msg->x, msg->y, msg->z);
    drone_pose_local_.set_heading(msg->heading);
}

void ControllerNode::status_cb(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{

    drone_status_ = *msg;
}

void ControllerNode::ctrl_state_cb(const std_msgs::msg::UInt8::SharedPtr msg)
{
    state_ = (state_t)msg->data;

}

void ControllerNode::global_pos_cb(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
    drone_pose_global_.set_position(msg->lat, msg->lon, msg->alt);
}

void ControllerNode::rp_pos_cb(const controller_msgs::msg::RefPointGlobalPosition::SharedPtr msg)
{

    //              msg->lat, msg->lon, msg->alt);
    ref_point_pose_global_.set_position(msg->lat, msg->lon, msg->alt);
    ref_point_pose_global_.set_heading(msg->heading);
    ref_point_pose_global_.set_terrain_offset(msg->terrain_offset);
    ref_point_pose_local_ = convert_global_to_local(ref_point_pose_global_, origin_);
    LOG_MISSION_DEBUG(this, "Reference point local: %s", ref_point_pose_local_.jsonify().dump().c_str());

}

/**
 *  @brief  This callback will set the local origin point to the first established RP position for 
 *          each drone.
 */
void ControllerNode::rp_established_cb(const std_msgs::msg::Bool::SharedPtr msg)
{
    rp_established_ = msg->data;

    if (rp_established_ && !origin_set_)
    {
        LOG_MISSION_DEBUG(this, "Setting origin");
        LOG_MISSION_DEBUG(this, "Origin position: %f, %f, %f", ref_point_pose_global_.get_position()[0],
                          ref_point_pose_global_.get_position()[1], ref_point_pose_global_.get_position()[2]);
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN,
                                NAN, NAN, NAN, NAN, ref_point_pose_global_.position_[0],
                                ref_point_pose_global_.position_[1], ref_point_pose_global_.position_[2]);
        origin_ = ref_point_pose_global_;
        origin_set_ = true;
    }
}

/**
 *  @brief  Used to alter PID gains during flight.
 */
void ControllerNode::set_PID_cb(const controller_msgs::msg::PidSet::SharedPtr msg)
{
    control_toolbox::Pid::Gains gains;
    gains.p_gain_ = msg->kp;
    gains.i_gain_ = msg->ki;
    gains.d_gain_ = msg->kd;
    char type = msg->type;
    pid_type_t pid_type;
    switch (type)
    {
    case 'p':
        pid_type = POSITION;
        break;
    case 'a':
        pid_type = ALTITUDE;
        break;
    case 'h':
        pid_type = HEADING;
        break;
    }
    set_PID_gains(gains, pid_type);
}

/**
 *  @brief  Used to confirm that each drone has set their origin to the same point.
 *  TODO: Currently does not work due to issues with the micro-dds-client setup.
 */
void ControllerNode::vehicle_command_ack_cb(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
{


    switch (msg->command)
    {
    case px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN:
        if (msg->result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED)
        {

            origin_set_ = true;
        }
        break;
    default:
        break;
    }
}


// PUBLISHERS //


void ControllerNode::pub_rendezvous_complete()
{

    std_msgs::msg::Bool msg{};
    msg.data = true;
    rendezvous_status_pub_->publish(msg);
}

void ControllerNode::pub_rendezvous_incomplete()
{
    std_msgs::msg::Bool msg{};
    msg.data = false;
    rendezvous_status_pub_->publish(msg);
}


// METHODS //


/**
 *  @brief  Compares the drones position to the reference point position. Once the drone
 *          is in its formation position it notifies the lifecycle node which uses this 
 *          to synchronise the swarm.
 */
void ControllerNode::rendezvous()
{
    // TODO Investigate putting rendezvous on a timer.
    static int count = 0;
    LOG_MISSION_DEBUG(this, "RP ESTABLISHED: %s", rp_established_ ? "true":"false");
    if (rp_established_)
    {
        // Run the PID to form initial formation.
        this->run_PID();

        // If drone is within threshold increase count, otherwise decrease count.
        if (this->in_position() && count < MAX_COUNT)
        {
            count++;
        }
        else if (count > 0)
        {
            count--;
        }

        LOG_MISSION_DEBUG(this, "IN POSITION COUNT: %d\tOUT OF %d", count, MAX_COUNT);
        if (count > COUNT_THRESHOLD)
        {
            this->pub_rendezvous_complete();

        }
        else
        {
            this->pub_rendezvous_incomplete();
        }
    }
    else
    {

    }
}


void ControllerNode::set_error_thresholds_cb(controller_msgs::msg::ErrorThresholdSet::SharedPtr msg)
{
    set_error_thresholds(msg->pos, msg->alt, msg->heading);
}

void ControllerNode::set_error_thresholds(double pos_err, double alt_err, double head_err)
{
    pos_err_threshold_ = pos_err;
    alt_err_threshold_ = alt_err;
    head_err_threshold_ = head_err;
    LOG_MISSION_DEBUG(this, "Error thresholds set to: %f, %f, %f", pos_err_threshold_, alt_err_threshold_, head_err_threshold_);
}

/**
 *  @brief  Checks the drones position error to see whether it is within postion error thresholds.
 *
 *  @return bool    True if within position thresholds, false otherwise.
 */
bool ControllerNode::in_position()
{
    bool pos_correct = 0;
    bool alt_correct = 0;
    LocalPose pos_err;
    LocalPose goal;

    PID_.find_goal_position(goal, ref_point_pose_local_);
    pos_err = PID_.get_error(drone_pose_local_, goal);

    pos_correct = abs(pos_err.position_[0]) < pos_err_threshold_ &&
                  abs(pos_err.position_[1]) < pos_err_threshold_;
    alt_correct = abs(pos_err.position_[2]) < alt_err_threshold_;
    return alt_correct && pos_correct;



}

/**
 *  @brief Converts a global pose to a local pose.
 *  
 *  @param  global     Reference to the global position to be converted.
 *  @param  origin     Reference to the global origin position in GPS.
 *  @return LocalPose  The converted local equivalent of the global pose.
 */
LocalPose ControllerNode::convert_global_to_local(GlobalPose &global, const GlobalPose &origin)
{
    // Convert degrees to radians
    double lat_ref_rad = origin.get_position_rad()[0];
    double lon_ref_rad = origin.get_position_rad()[1];
    double lat_rad = global.get_position_rad()[0];
    double lon_rad = global.get_position_rad()[1];

    // Differences in coordinates
    double d_lat = lat_rad - lat_ref_rad;
    double d_lon = lon_rad - lon_ref_rad;

    // Earth's radius at the reference latitude
    double R_N = EARTHS_RADIUS * (1 - FLATTENING) / pow(1 - FLATTENING * pow(sin(lat_ref_rad), 2), 1.5);
    double R_E = EARTHS_RADIUS / sqrt(1 - FLATTENING * pow(sin(lat_ref_rad), 2));

    // Convert to NED coordinates
    double north = d_lat * R_N;
    double east = d_lon * R_E * cos(lat_ref_rad);
    double down = global.get_position()[2] - origin.get_terrain_offset();
    return LocalPose(north, east, down, global.get_heading());
}


void ControllerNode::log_positions(const std::vector<Pose> &poses) const
{
    try
    {
        // Create a JSON array
        json combined_json = json::array();

        // Add each pose to the JSON array
        for (const auto &pose : poses)
        {
            combined_json.push_back(pose.jsonify());
        }

        // Log the JSON string
        LOG_MISSION_POSITION(this, "HERE: %s", combined_json.dump().c_str());
    }
    catch (const std::exception &e)
    {
        LOG_MISSION_ERROR(this, "Failed to log positions: %s", e.what());
    }
}

template <typename... Poses>
void ControllerNode::log_positions(const Pose &first, const Poses &...rest) const
{
    // Collect all arguments into a vector
    std::vector<Pose> poses = {first, rest...};

    // Call the original log_positions
    log_positions(poses);
}

/**
 * @brief Updates the drone's position based on corrections calculated by the PID controller.
 */
void ControllerNode::run_PID()
{

    LOG_MISSION_DEBUG(this, "%s", "running PID" );

    px4_msgs::msg::TrajectorySetpoint msg{};
    std::array<double, 7> corrections;
    static LocalPose goal_pos("goal_pos");


    // Determine the goal position based on the reference point
    PID_.find_goal_position(goal_pos, ref_point_pose_local_);

    // Update PID with current and goal positions
    log_positions(drone_pose_local_, goal_pos, ref_point_pose_local_);
    PID_.update_pids(drone_pose_local_, goal_pos);
    corrections = PID_.get_corrections();
    
    LOG_MISSION_DEBUG(this, "CORRECTIONS: %f, %f, %f, %f, %f, %f, %f",
                      corrections[0], corrections[1], corrections[2],
                      corrections[3], corrections[4], corrections[5],
                      corrections[6]);



    // Position Control - MUST BE SET TO NAN FOR VELOCITY CONTROL TO WORK
    msg.position = {NAN, NAN, NAN};
    msg.yaw = NAN;

    // Velocity Control
    msg.velocity = {static_cast<float>(corrections[0]),
                   static_cast<float>(corrections[1]),
                   static_cast<float>(corrections[2])};
    msg.yawspeed = static_cast<float>(corrections[6]);

    // Acceleration Control (Optional)
    // msg.acceleration = {static_cast<float>(corrections[3]),
    //                     static_cast<float>(corrections[4]),
    //                     static_cast<float>(corrections[5])};

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_pub_->publish(msg);
}


// FCU COMMANDS //


void ControllerNode::publish_offboard_control_mode()
{
    
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = false;
    msg.velocity = true;
    msg.acceleration = true;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief   Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void ControllerNode::publish_vehicle_command(uint32_t command, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
    msg.param6 = param6;
    msg.param7 = param7;
    msg.command = command;
    msg.target_system = drone_id_ + SYSTEM_OFFSET;
    msg.target_component = 1;
    msg.source_system = drone_id_;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_pub_->publish(msg);
}


// MAIN FLIGHT CONTROL LOOP //


/**
 *  @brief  Execution loop for the controller node.
 */
void ControllerNode::control_loop()
{
    state_t state;
    state = this->get_state();

    LOG_MISSION_DEBUG(this, "CONTROL LOOP State: %s", state_to_string(state).c_str());
    LOG_MISSION_DEBUG(this, "CONTROL LOOP %s", drone_pose_local_.jsonify().dump().c_str());
    switch (state)
    {
    case RENDEZVOUS:
        this->rendezvous();
        break;
    case MOVE:
        this->run_PID();
        break;
    case PAUSE:
        this->run_PID();
    /* Currently these states are implecitely handled by RP and Lifecycle node. */
    case CONFIG:
    case LANDED:
    /* Handled by Pixhawk flight controller. */
    case LAND:
    case TAKEOFF:
    case PX4_HOLD:
    case RETURN:
        break;
    default:
        break;
    }
}


ControllerNode::~ControllerNode() = default;
