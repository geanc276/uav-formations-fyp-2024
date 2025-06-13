#ifndef FCU_H
#define FCU_H
// TODO fcu should be in px4_node
// ? actually idk, probably good here

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/telemetry_status.hpp>
#include <px4_msgs/msg/log_message.hpp>
#include <px4_msgs/msg/mavlink_log.hpp>
#include <px4_msgs/msg/failsafe_flags.hpp>
#include <px4_msgs/msg/failure_detector_status.hpp>
#include <px4_msgs/msg/event.hpp>



#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/mode_completed.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>

#include <px4_msgs/msg/vehicle_command_ack.hpp>

#define FCU_QUEUE_SIZE 10
#define FCU_DEFAULT_TAKEOFF_HEIGHT 5.0

class FCU
{
private:
    std::shared_ptr<rclcpp::Node> node_;
    std::string drone_name_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_status_subscriber_;

    rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicle_command_ack_sub_;
    rclcpp::Subscription<px4_msgs::msg::TelemetryStatus>::SharedPtr telemetry_status_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::LogMessage>::SharedPtr log_message_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::MavlinkLog>::SharedPtr mavlink_log_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_subscriber_;
    
    rclcpp::Subscription<px4_msgs::msg::FailsafeFlags>::SharedPtr failsafe_flags_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::FailureDetectorStatus>::SharedPtr failure_detector_status_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::Event>::SharedPtr event_subscriber_;

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr off_board_control_mode_publisher_;

    geometry_msgs::msg::PoseStamped current_pose_;
    px4_msgs::msg::VehicleStatus current_status_;
    px4_msgs::msg::BatteryStatus current_battery_status_;


    bool current_armed_;
    // TODO standardise the takeoff height, and probably the drone id with ros params
    double takeoff_height_;

    int id_;

    void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void state_cb(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void battery_status_cb(const px4_msgs::msg::BatteryStatus::SharedPtr msg);
    void telemetry_status_cb(const px4_msgs::msg::TelemetryStatus::SharedPtr msg);
    void log_message_cb(const px4_msgs::msg::LogMessage::SharedPtr msg);
    void mavlink_log_cb(const px4_msgs::msg::MavlinkLog::SharedPtr msg);
    void offboard_control_mode_cb(const px4_msgs::msg::OffboardControlMode::SharedPtr msg);

    void failsafe_flags_cb(const px4_msgs::msg::FailsafeFlags::SharedPtr msg);
    void failure_detector_status_cb(const px4_msgs::msg::FailureDetectorStatus::SharedPtr msg);
    void event_cb(const px4_msgs::msg::Event::SharedPtr msg);


public:
    FCU(const std::shared_ptr<rclcpp::Node> &node, const std::string &drone_name, int id);
    FCU(const std::shared_ptr<rclcpp::Node> &node);
    ~FCU();


    void init_subs(void);
    void init_pubs(void);
    
    /*
     *
     */
    void vehicle_command_ack_cb(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg);
    void publish_off_board_control_mode(void);
    void publish_off_board(void);
    void publish_takeoff(void);
    void publish_hold(void);
    void publish_return(void);
    void publish_land(void);
    void publish_arm(void);
    void publish_arm_timer_send(void);
    rclcpp::TimerBase::SharedPtr arming_timer_;
    void publish_disarm(void);

    bool set_mode(const std::string &new_mode);
    bool set_armed(bool arming_value);
    void set_takeoff_height(double height);

    geometry_msgs::msg::Point get_drone_position() const;
    geometry_msgs::msg::Quaternion get_drone_orientation() const;
    uint8_t get_mode() const;
    bool is_armed() const;
    double get_takeoff_height() const;
    // void publish_vehicle_command(uint16_t command, float param1, float param2);
    void publish_vehicle_command(uint16_t command, float param1, float param2, float param3, float param4, float param5, float param6, float param7);



    void get_battery_status(float_t&, float_t&, float_t&);
    void get_vehicle_status(uint8_t&, uint8_t&);
    
};

#endif /* FCU_H */