#include "lifecycle_controller/fcu.hpp"
#include "logging/logging.hpp"
// TODO NEED TO MAKE SURE ID IS ACCURATE
// TODO vehicle command ack

// Add an offset to the target system for the simulation as required by px4_sitl.
#ifndef SYSTEM_OFFSET
#define SYSTEM_OFFSET 0
#endif

FCU::FCU(const std::shared_ptr<rclcpp::Node> &node, const std::string &drone_name, const int id)
    : node_(node), drone_name_(drone_name), current_armed_(false), id_(id)
{
    LOG_SETUP_DEBUG(node, "SYSTEM_OFFSET: %d", SYSTEM_OFFSET);
    // Log all environment variables
    extern char **environ;
    for (char **env = environ; *env != nullptr; ++env) {
        LOG_SETUP_DEBUG(node, "ENV VAR: %s", *env);
    }
    LOG_SETUP_DEBUG(node, "FCU:\n\tDrone name: %s\n\tID: %d", drone_name.c_str(), id);
    arming_timer_ = node_->create_wall_timer(std::chrono::seconds(1), std::bind(&FCU::publish_arm_timer_send, this));
    arming_timer_->cancel();
    init_pubs();
    init_subs();
    LOG_SETUP_INFO(node, "FCU for drone %s initialised", drone_name.c_str());
}

FCU::FCU(const std::shared_ptr<rclcpp::Node> &node)
    : node_(node), drone_name_(""), current_armed_(false)
{
}

FCU::~FCU() = default;

void FCU::pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    current_pose_ = *msg;
}

void FCU::init_subs(void)
{
    /* Quality of Service set-up for PX4 topics. */
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    std::string status_topic = "/px4_" + std::to_string(id_) + "/fmu/out/vehicle_status";
    status_subscriber_ = node_->create_subscription<px4_msgs::msg::VehicleStatus>(
        status_topic, qos, std::bind(&FCU::state_cb, this, std::placeholders::_1));

    battery_status_subscriber_ = node_->create_subscription<px4_msgs::msg::BatteryStatus>(
        "/px4_" + std::to_string(id_) + "/fmu/out/battery_status", qos, std::bind(&FCU::battery_status_cb, this, std::placeholders::_1));

    vehicle_command_ack_sub_ = node_->create_subscription<px4_msgs::msg::VehicleCommandAck>(
        "/px4_" + std::to_string(id_) + "/fmu/out/vehicle_command_ack", qos, std::bind(&FCU::vehicle_command_ack_cb, this, std::placeholders::_1));

    telemetry_status_subscriber_ = node_->create_subscription<px4_msgs::msg::TelemetryStatus>(
        "/px4_" + std::to_string(id_) + "/fmu/out/telemetry_status", qos, std::bind(&FCU::telemetry_status_cb, this, std::placeholders::_1));

    log_message_subscriber_ = node_->create_subscription<px4_msgs::msg::LogMessage>(
        "/px4_" + std::to_string(id_) + "/fmu/out/log_message", qos, std::bind(&FCU::log_message_cb, this, std::placeholders::_1));

    mavlink_log_subscriber_ = node_->create_subscription<px4_msgs::msg::MavlinkLog>(
        "/px4_" + std::to_string(id_) + "/fmu/out/mavlink_log", qos, std::bind(&FCU::mavlink_log_cb, this, std::placeholders::_1));

    offboard_control_mode_subscriber_ = node_->create_subscription<px4_msgs::msg::OffboardControlMode>(
        "/px4_" + std::to_string(id_) + "/fmu/out/offboard_control_mode", qos, std::bind(&FCU::offboard_control_mode_cb, this, std::placeholders::_1));


    LOG_SETUP_DEBUG(node_, "starting failsafe flags subscriber");
    failsafe_flags_subscriber_ = node_->create_subscription<px4_msgs::msg::FailsafeFlags>(
        "/px4_" + std::to_string(id_) + "/fmu/out/failsafe_flags", qos, std::bind(&FCU::failsafe_flags_cb, this, std::placeholders::_1));

    failure_detector_status_subscriber_ = node_->create_subscription<px4_msgs::msg::FailureDetectorStatus>(
        "/px4_" + std::to_string(id_) + "/fmu/out/failure_detector_status", qos, std::bind(&FCU::failure_detector_status_cb, this, std::placeholders::_1));

    event_subscriber_ = node_->create_subscription<px4_msgs::msg::Event>(
        "/px4_" + std::to_string(id_) + "/fmu/out/event", qos, std::bind(&FCU::event_cb, this, std::placeholders::_1));

    LOG_SETUP_DEBUG(node_, "FINISHED SUBSCRIBING TO ALL TOPICS");
}

void FCU::init_pubs(void)
{
    // Publisher setup
    vehicle_command_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>("/px4_" + std::to_string(id_) + "/fmu/in/vehicle_command", FCU_QUEUE_SIZE);
    off_board_control_mode_publisher_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>("/px4_" + std::to_string(id_) + "/fmu/in/offboard_control_mode", FCU_QUEUE_SIZE);
}

void FCU::state_cb(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    LOG_MISSION_DEBUG(node_, "FCU State CB: %d", msg->nav_state);
    LOG_MISSION_DEBUG(node_, "FCU Arming CB: %d", msg->arming_state);
    LOG_MISSION_DEBUG(node_, "FCU FAILURE CB: %d", msg->failure_detector_status);

    current_status_ = *msg;
    current_armed_ = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
}

void FCU::battery_status_cb(const px4_msgs::msg::BatteryStatus::SharedPtr msg)
{
    LOG_MISSION_DEBUG(node_, "FCU Status: remaining %d, voltage %d, current %d, predicted time %d", msg->remaining, msg->voltage_v, msg->current_a, msg->time_remaining_s);
    current_battery_status_ = *msg;
}

void FCU::telemetry_status_cb(const px4_msgs::msg::TelemetryStatus::SharedPtr msg)
{
    // LOG_MISSION_DEBUG(node_, "Telemetry: fixed=%s, ts=%llu, type=%d, link=%d%%, rssi=%d, remote_rssi=%d, tx=%d, rx=%d, noise=%d", msg->fixed ? "true" : "false", msg->timestamp, msg->type, msg->link_quality, msg->rssi, msg->remote_rssi, msg->tx_rate, msg->rx_rate, msg->noise);

    LOG_MISSION_DEBUG(
        node_,
        "Telemetry: ts=%llu, type=%u, mode=%u, flow_control=%s, forwarding=%s, mavlink_v2=%s, "
        "ftp=%s, streams=%u, data_rate=%.2f B/s, rate_multiplier=%.2f, "
        "tx_rate_avg=%.2f B/s, tx_error_rate_avg=%.2f B/s, tx_msg_count=%u, tx_buf_overruns=%u, "
        "rx_rate_avg=%.2f B/s, rx_msg_count=%u, rx_msg_lost_count=%u, rx_buf_overruns=%u, "
        "rx_parse_errors=%u, rx_pkt_drop_count=%u, rx_msg_lost_rate=%.2f",
        msg->timestamp,                       // timestamp
        msg->type,                            // link type
        msg->mode,                            // mode
        msg->flow_control ? "true" : "false", // flow control
        msg->forwarding ? "true" : "false",   // forwarding
        msg->mavlink_v2 ? "true" : "false",   // MAVLink v2
        msg->ftp ? "true" : "false",          // FTP support
        msg->streams,                         // stream count
        msg->data_rate,                       // configured max data rate
        msg->rate_multiplier,                 // rate multiplier
        msg->tx_rate_avg,                     // transmit rate average
        msg->tx_error_rate_avg,               // transmit error rate average
        msg->tx_message_count,                // total sent messages
        msg->tx_buffer_overruns,              // TX buffer overruns
        msg->rx_rate_avg,                     // receive rate average
        msg->rx_message_count,                // total received messages
        msg->rx_message_lost_count,           // total lost messages
        msg->rx_buffer_overruns,              // RX buffer overruns
        msg->rx_parse_errors,                 // RX parse errors
        msg->rx_packet_drop_count,            // packet drops
        msg->rx_message_lost_rate             // RX message lost rate
    );

    // LOG_MISSION_DEBUG(node_, "FCU Telemetry Status: %d", msg->telemetry_status);
}

void FCU::log_message_cb(const px4_msgs::msg::LogMessage::SharedPtr msg)
{
    LOG_MISSION_DEBUG(node_, "FCU Log Message: %d, %s", msg->severity, msg->text);
}

void FCU::mavlink_log_cb(const px4_msgs::msg::MavlinkLog::SharedPtr msg)
{

    LOG_MISSION_DEBUG(node_, "FCU MAVLink Log: %d, %s", msg->severity, msg->text);
}

void FCU::offboard_control_mode_cb(const px4_msgs::msg::OffboardControlMode::SharedPtr msg)
{
    LOG_MISSION_DEBUG(node_, "FCU Offboard Control Mode: %d, %d, %d, %d, %d", msg->position, msg->velocity, msg->acceleration, msg->attitude, msg->body_rate);
}

void FCU::failsafe_flags_cb(const px4_msgs::msg::FailsafeFlags::SharedPtr msg)
{
    LOG_MISSION_DEBUG(node_,"%s","  HERE HERE"); 
    LOG_MISSION_DEBUG(node_, "Failsafe Flags: timestamp=%llu, mode_req_angular_velocity=%u, mode_req_attitude=%u, mode_req_local_alt=%u, mode_req_local_position=%u, mode_req_local_position_relaxed=%u, mode_req_global_position=%u, mode_req_mission=%u, mode_req_offboard_signal=%u, mode_req_home_position=%u, mode_req_wind_and_flight_time_compliance=%u, mode_req_prevent_arming=%u, mode_req_manual_control=%u, mode_req_other=%u, angular_velocity_invalid=%s, attitude_invalid=%s, local_altitude_invalid=%s, local_position_invalid=%s, local_position_invalid_relaxed=%s, local_velocity_invalid=%s, global_position_invalid=%s, auto_mission_missing=%s, offboard_control_signal_lost=%s, home_position_invalid=%s, manual_control_signal_lost=%s, gcs_connection_lost=%s, battery_warning=%u, battery_low_remaining_time=%s, battery_unhealthy=%s, geofence_breached=%s, mission_failure=%s, vtol_fixed_wing_system_failure=%s, wind_limit_exceeded=%s, flight_time_limit_exceeded=%s, local_position_accuracy_low=%s, fd_critical_failure=%s, fd_esc_arming_failure=%s, fd_imbalanced_prop=%s, fd_motor_failure=%s",
                      msg->timestamp,
                      msg->mode_req_angular_velocity,
                      msg->mode_req_attitude,
                      msg->mode_req_local_alt,
                      msg->mode_req_local_position,
                      msg->mode_req_local_position_relaxed,
                      msg->mode_req_global_position,
                      msg->mode_req_mission,
                      msg->mode_req_offboard_signal,
                      msg->mode_req_home_position,
                      msg->mode_req_wind_and_flight_time_compliance,
                      msg->mode_req_prevent_arming,
                      msg->mode_req_manual_control,
                      msg->mode_req_other,
                      msg->angular_velocity_invalid ? "true" : "false",
                      msg->attitude_invalid ? "true" : "false",
                      msg->local_altitude_invalid ? "true" : "false",
                      msg->local_position_invalid ? "true" : "false",
                      msg->local_position_invalid_relaxed ? "true" : "false",
                      msg->local_velocity_invalid ? "true" : "false",
                      msg->global_position_invalid ? "true" : "false",
                      msg->auto_mission_missing ? "true" : "false",
                      msg->offboard_control_signal_lost ? "true" : "false",
                      msg->home_position_invalid ? "true" : "false",
                      msg->manual_control_signal_lost ? "true" : "false",
                      msg->gcs_connection_lost ? "true" : "false",
                      msg->battery_warning,
                      msg->battery_low_remaining_time ? "true" : "false",
                      msg->battery_unhealthy ? "true" : "false",
                      msg->primary_geofence_breached ? "true" : "false",
                      msg->mission_failure ? "true" : "false",
                      msg->vtol_fixed_wing_system_failure ? "true" : "false",
                      msg->wind_limit_exceeded ? "true" : "false",
                      msg->flight_time_limit_exceeded ? "true" : "false",
                      msg->local_position_accuracy_low ? "true" : "false",
                      msg->fd_critical_failure ? "true" : "false",
                      msg->fd_esc_arming_failure ? "true" : "false",
                      msg->fd_imbalanced_prop ? "true" : "false",
                      msg->fd_motor_failure ? "true" : "false");
    // LOG_MISSION_DEBUG(node_, "FCU Failsafe Flags: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", msg->rc_lost, msg->rc_lost_remote, msg->rc_loss_failsafe, msg->rc_loss_timeout, msg->rc_loss_rc, msg->rc_loss_data, msg->rc_loss_gps, msg->rc_loss_battery, msg->rc_loss_mavlink, msg->rc_loss_linkquality);
}

void FCU::failure_detector_status_cb(const px4_msgs::msg::FailureDetectorStatus::SharedPtr msg)
{
    LOG_MISSION_DEBUG(node_, "Failure Detector Status: timestamp=%llu, fd_roll=%s, fd_pitch=%s, fd_alt=%s, fd_ext=%s, fd_arm_escs=%s, fd_battery=%s, fd_imbalanced_prop=%s, fd_motor=%s, imbalanced_prop_metric=%.2f, motor_failure_mask=%u",
                      msg->timestamp,
                      msg->fd_roll ? "true" : "false",
                      msg->fd_pitch ? "true" : "false",
                      msg->fd_alt ? "true" : "false",
                      msg->fd_ext ? "true" : "false",
                      msg->fd_arm_escs ? "true" : "false",
                      msg->fd_battery ? "true" : "false",
                      msg->fd_imbalanced_prop ? "true" : "false",
                      msg->fd_motor ? "true" : "false",
                      msg->imbalanced_prop_metric,
                      msg->motor_failure_mask);
    // LOG_MISSION_DEBUG(node_, "FCU Failure Detector Status: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", msg->rc_input, msg->gps, msg->battery, msg->acceleration, msg->gyroscope, msg->magnetometer, msg->barometer, msg->airspeed, msg->distance_sensor, msg->optical_flow);
}

void FCU::event_cb(const px4_msgs::msg::Event::SharedPtr msg)
{
    LOG_MISSION_DEBUG(node_, "Vehicle Event: timestamp=%llu, id=%u, event_sequence=%u, arguments=%s, log_levels=%u",
                      msg->timestamp,
                      msg->id,
                      msg->event_sequence,
                      std::string(msg->arguments.begin(), msg->arguments.end()).c_str(),
                      msg->log_levels);
    // LOG_MISSION_DEBUG(node_, "FCU Vehicle Event: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", msg->timestamp, msg->event, msg->data, msg->reason, msg->action, msg->info, msg->id, msg->hash, msg->hash_id, msg->hash_instance);
}

void FCU::set_takeoff_height(double height)
{

    takeoff_height_ = height;
    // node_->set_parameter(rclcpp::Parameter("MIS_TAKEOFF_ALT", height));
}

geometry_msgs::msg::Point FCU::get_drone_position() const
{
    return current_pose_.pose.position;
}

geometry_msgs::msg::Quaternion FCU::get_drone_orientation() const
{
    return current_pose_.pose.orientation;
}

uint8_t FCU::get_mode() const
{
    return current_status_.nav_state;
}

bool FCU::is_armed() const
{
    return current_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
}

double FCU::get_takeoff_height() const
{
    double value;
    node_->get_parameter("MIS_TAKEOFF_ALT", value);
    return value;
}

/*
 *
 */
void FCU::vehicle_command_ack_cb(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
{
    if (msg->result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED)
    {
        LOG_MISSION_INFO(node_, "Vehicle command accepted %d", msg->command);
    }
    else
    {
        LOG_MISSION_ERROR(node_, "Vehicle command rejected %d", msg->command);
    }
    if (msg->command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM)
    {
        if (msg->result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED)
        {
            LOG_MISSION_INFO(node_, "Vehicle armed accepted");
        }
        else
        {
            LOG_MISSION_ERROR(node_, "Vehicle armed rejected");
        }
    }
}

void FCU::publish_off_board_control_mode()
{
    // velocity control is active, i.e. true
    auto msg = px4_msgs::msg::OffboardControlMode();
    msg.position = false;
    msg.velocity = true;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    off_board_control_mode_publisher_->publish(msg);
}

void FCU::publish_vehicle_command(uint16_t command, float param1 = NAN, float param2 = NAN, float param3 = NAN, float param4 = NAN, float param5 = NAN, float param6 = NAN, float param7 = NAN)
{

    // px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH; // Return to land
    // px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND; // Land
    // px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF; // Takeoff
    // px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_REPOSITION; // Hold
    LOG_MISSION_DEBUG(node_, "Publishing vehicle command %d", command);
    LOG_MISSION_DEBUG(node_, "id: %d\t System_offset: %d\t target_system: %d", id_, SYSTEM_OFFSET, id_ + SYSTEM_OFFSET);
    auto msg = px4_msgs::msg::VehicleCommand();
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
    msg.param6 = param6;
    msg.param7 = param7;
    msg.command = command;
    msg.target_system = id_ + SYSTEM_OFFSET;
    msg.target_component = 1;
    msg.source_system = id_;
    msg.source_component = 1; // Component within the source system

    vehicle_command_pub_->publish(msg);
}

void FCU::publish_hold()
{
    this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_REPOSITION, NAN, NAN, 0, NAN, NAN, NAN, 0);
}

void FCU::publish_takeoff()
{
    this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 0.0, NAN, NAN, NAN, NAN, NAN, takeoff_height_);
}

void FCU::publish_land()
{
    this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0, 0.0, NAN, NAN, NAN, NAN, NAN);
}

void FCU::publish_return()
{
    this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH, NAN, NAN, NAN, NAN, NAN, NAN, NAN);
}

void FCU::publish_off_board()
{

    this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6, NAN, NAN, NAN, NAN, NAN);
}

void FCU::publish_arm()
{
    LOG_MISSION_DEBUG(node_, "PUBLISHING ARMING RECEIVED");
    if (arming_timer_->is_canceled())
    {
        // armed_timer_ = node_->create_wall_timer(std::chrono::seconds(1), std::bind(&FCU::publish_arm_timer_send, this));
        arming_timer_->reset();
    }
}

void FCU::publish_arm_timer_send()
{
    LOG_MISSION_DEBUG(node_, "SENDING ARM COMMAND");
    if (current_armed_)
    {
        LOG_MISSION_DEBUG(node_, "ALREADY armed");
        arming_timer_->cancel();
        return;
    }
    publish_off_board();

    this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1, 0.0, NAN, NAN, NAN, NAN, NAN);
}

void FCU::publish_disarm()
{
    this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0, NAN, NAN, NAN, NAN, NAN);
}

void FCU::get_battery_status(float_t &battery_percentage, float_t &battery_voltage, float_t &battery_current)
{
    battery_percentage = current_battery_status_.remaining;
    battery_voltage = current_battery_status_.voltage_v;
    battery_current = current_battery_status_.current_a;
}

void FCU::get_vehicle_status(uint8_t &nav_state, uint8_t &armed)
{
    nav_state = current_status_.nav_state;
    armed = current_status_.arming_state;
}

// void publish_auto_takeoff_command()
// {
//     // TODO i think target and source should be id of the drone
//     auto msg = px4_msgs::msg::VehicleCommand();
//     msg.timestamp = this->now().nanoseconds() / 1000;
//     msg.param1 = 1.0; // Example parameter
//     msg.param2 = 0.0; // Example parameter
//     msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
//     msg.target_system = id_;    // Example system ID
//     msg.target_component = id_; // Example component ID

//     command_publisher_->publish(msg);

//     vehicle_command_pub_->publish(message);
// }

// px4_msgs::msg::VehicleCommand hold_command;
// hold_command.timestamp = /* current timestamp in microseconds */;
// hold_command.param1 = NAN;
// hold_command.param2 = NAN;
// hold_command.param3 = 10.0; // Altitude (relative or absolute, depending on param7)
// hold_command.param4 = NAN;  // Yaw angle (set to NaN to use current yaw heading)
// hold_command.param5 = NAN;  // Latitude (set to NaN to hold current position)
// hold_command.param6 = NAN;  // Longitude (set to NaN to hold current position)
// hold_command.param7 = 0;    // Altitude type (0 for relative, 1 for absolute)
// hold_command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_REPOSITION;
// hold_command.target_system = 1;
// hold_command.target_component = 1;
// hold_command.source_system = 255;  // Ground station or sending system
// hold_command.source_component = 1; // Component within the source system

// px4_msgs::msg::VehicleCommand takeoff_command;
// takeoff_command.timestamp = /* current timestamp in microseconds */;
// takeoff_command.param1 = 0.0; // Minimum pitch (set to 0 for multicopters)
// takeoff_command.param2 = NAN;
// takeoff_command.param3 = NAN;
// takeoff_command.param4 = NAN;  // Yaw angle (set to NaN to use current yaw heading)
// takeoff_command.param5 = NAN;  // Latitude (set to NaN to use current position)
// takeoff_command.param6 = NAN;  // Longitude (set to NaN to use current position)
// takeoff_command.param7 = 10.0; // Altitude (relative to home position)
// takeoff_command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
// takeoff_command.target_system = 1;
// takeoff_command.target_component = 1;
// takeoff_command.source_system = 255;  // Ground station or sending system
// takeoff_command.source_component = 1; // Component within the source system

// px4_msgs::msg::VehicleCommand land_command;
// land_command.timestamp = /* current timestamp in microseconds */;
// land_command.param1 = 0.0; // Abort Alt (set to 0 to use default)
// land_command.param2 = 0.0; // Precision land mode (set to 0 for normal landing)
// land_command.param3 = NAN;
// land_command.param4 = NAN; // Yaw angle (set to NaN to use current yaw heading)
// land_command.param5 = NAN; // Latitude (set to NaN to land at current position)
// land_command.param6 = NAN; // Longitude (set to NaN to land at current position)
// land_command.param7 = NAN; // Altitude (set to NaN to use current altitude)
// land_command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
// land_command.target_system = 1;
// land_command.target_component = 1;
// land_command.source_system = 255;  // Ground station or sending system
// land_command.source_component = 1; // Component within the source system

// px4_msgs::msg::VehicleCommand return_to_land_command;
// return_to_land_command.timestamp = /* current timestamp in microseconds */;
// return_to_land_command.param1 = NAN;
// return_to_land_command.param2 = NAN;
// return_to_land_command.param3 = NAN;
// return_to_land_command.param4 = NAN;
// return_to_land_command.param5 = NAN;
// return_to_land_command.param6 = NAN;
// return_to_land_command.param7 = NAN;
// return_to_land_command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH;
// return_to_land_command.target_system = 1;
// return_to_land_command.target_component = 1;
// return_to_land_command.source_system = 255;  // Ground station or sending system
// return_to_land_command.source_component = 1; // Component within the source system