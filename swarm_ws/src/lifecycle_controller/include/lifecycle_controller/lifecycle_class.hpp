#ifndef LIFECYCLE_HPP
#define LIFECYCLE_HPP

#include <string>
#include <unordered_map>
#include <functional>
#include <vector>
#include <fstream>
#include <iostream>
#include <chrono>
#include <stdexcept>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

#include <px4_msgs/msg/battery_status.hpp>

#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/mode_completed.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <lifecycle_controller/state.hpp>
#include <lifecycle_controller/error.hpp>
#include <lifecycle_controller/cooperative.hpp>
#include <lifecycle_controller/dropout.hpp>
#include <lifecycle_controller/fcu.hpp>
#include <lifecycle_controller/status.hpp>


#include <controller_msgs/msg/state_set.hpp>
#include <controller_msgs/msg/error_threshold_set.hpp>
#include <controller_msgs/msg/pid_set.hpp>


#include <controller_msgs/msg/register_drone.hpp>
#include <controller_msgs/msg/register_drone_ack.hpp>
#include <controller_msgs/msg/update_drone_list.hpp>

#define LIFECYCLE_QUEUE_SIZE 10

class UserInput;
class ActionMap;

typedef enum
{
    CONFIG = 1,
    LANDED,
    TAKEOFF,
    RENDEZVOUZ,
    MOVE,
    PAUSE,
    LAND
} state_t;

class Lifecycle : public rclcpp::Node
{
public:
    // static std::shared_ptr<Lifecycle> create(const std::string &config_file, const std::string &drone_name, std::vector<std::string> other_drones, const int id, double takeoff_height);
    Lifecycle(const std::string &drone_name, int id = 0);
    static std::shared_ptr<Lifecycle> create(const std::string &drone_name, int id = 0);

    void run();
    bool transition(const std::string &nextState, bool force = false);
    std::string get_current_state() const;
    std::string get_drone_name() const;
    double get_takeoff_height() const;
    std::string get_state_config_file_path() const;
    void init();

private:
    // Lifecycle(const std::string &config_file, const std::string &drone_name, std::vector<std::string> other_drones, const int id, double takeoff_height);

    void get_parameters();
    /************ Initialization ************/

    void init_subs();
    void init_pubs();

    void init_status();
    void init_coops();
    void init_errors();
    void init_dropout();
    void init_FCU();

    /************ Callbacks ************/
    void gather_status(void);


    void add_drone_to_coops(const std::string &drone_name);

    void register_drone_timer_cb(void);
    void register_drone_ack_cb(const controller_msgs::msg::RegisterDroneAck::SharedPtr msg);
    void update_drone_list_cb(const controller_msgs::msg::UpdateDroneList::SharedPtr msg);


    void battery_status_cb(const px4_msgs::msg::BatteryStatus::SharedPtr msg);

    void global_position_cb(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);

    void state_cb(const px4_msgs::msg::VehicleStatus::SharedPtr msg);

    void rendezvous_done_cb(const std_msgs::msg::Bool::SharedPtr msg);

    void mode_complete_cb(const px4_msgs::msg::ModeCompleted::SharedPtr msg);

    void vehicle_command_ack_cb(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg);

    /************ Publishing Functions ************/
    void publish_ctrl_state(const std::string &state);
    void publish_altitude_bias(double bias);
    // void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

    /************ Helper Functions ************/

    bool is_any_data_stale();

    /************ Member Variables ************/
    State *current_state_;
    std::unordered_map<std::string, State> states_;

    bool off_board_;

    State *get_state_by_name(const std::string &name);
    std::string drone_name_;
    std::string drone_hostname_;
    uint16_t id_;
    // std::vector<std::string> other_drones_;
    std::unordered_map<uint8_t, std::string> other_drones_;

    double takeoff_height_;

    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_status_subscriber_;

    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_position_subscriber_;

    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_subscriber_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rendezvous_done_sub_;
    rclcpp::Subscription<px4_msgs::msg::ModeCompleted>::SharedPtr mode_complete_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicle_command_ack_sub_;

    rclcpp::Subscription<controller_msgs::msg::RegisterDroneAck>::SharedPtr register_drone_ack_sub_;
    rclcpp::Subscription<controller_msgs::msg::UpdateDroneList>::SharedPtr update_drone_list_sub_;
   
   
   
   
    /***************/
    rclcpp::Publisher<controller_msgs::msg::RegisterDrone>::SharedPtr register_drone_pub_;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr ctrl_state_pub_;

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;


    rclcpp::Publisher<controller_msgs::msg::ErrorThresholdSet>::SharedPtr error_threshold_pub_;
    rclcpp::Publisher<controller_msgs::msg::PidSet>::SharedPtr PID_pub_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;

    std::unique_ptr<Cooperative> launch_coop_;
    std::unique_ptr<Cooperative> formation_coop_;
    std::unique_ptr<Cooperative> return_coop_;

    std::unique_ptr<Error> takeoff_error_;
    std::unique_ptr<Error> land_error_;
    std::unique_ptr<Error> ang_error_;

    std::unique_ptr<Dropout> dropout_checker_;
    std::unique_ptr<FCU> fcu_;

    int dropout_time_threshold_seconds_;
    bool log_flag_;

    int last_state_;

    // config_file;
    nlohmann::json config_file_;
    std::string config_file_path_;

    
    std::string state_config_file_path_;


    std::string num_drones_str_;
    int num_drones_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr register_drone_timer_;


    Status status;
    friend class UserInput;
    friend class ActionMap;
};

#endif /* LIFECYCLE_HPP */
