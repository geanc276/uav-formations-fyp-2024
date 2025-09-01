#ifndef USER_INPUT_HPP
#define USER_INPUT_HPP


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <controller_msgs/msg/pid_set.hpp>
#include <controller_msgs/msg/error_threshold_set.hpp>
#include <controller_msgs/msg/state_set.hpp>

#include <iostream>
#include <sstream>
#include <thread>
#include <mutex>
#include <vector>
#include <map>
#include <functional>
#include <algorithm>

// For readline
#include <readline/readline.h>
#include <readline/history.h>

#ifdef _WIN32
  #define CLEAR_COMMAND "cls"
#else
  #define CLEAR_COMMAND "clear"
#endif


#define POS_ERR_THRESHOLD_DEFAULT 1
#define ALT_ERR_THRESHOLD_DEFAULT 1
#define HEAD_ERR_THRESHOLD_DEFAULT 10


// ------------------------------------------------------------------
// A small struct to capture the results of parsing the user's input
// ------------------------------------------------------------------
struct ParsedInput {
    std::string command;             // e.g. "start", "drone", "pid", etc.
    std::vector<std::string> args;   // positional arguments
    std::vector<std::string> flags;  // tokens beginning with "--"
};

// ------------------------------------------------------------------
// Command structure for self-documenting help
// ------------------------------------------------------------------
struct Command {
    std::string name;        // e.g. "start"
    std::string usage;       // e.g. "start - Begin the mission"
    // Handler: parse the tokens (both args & flags) after reading the first command
    std::function<void(const ParsedInput &)> handler;
};

class UserInputNode : public rclcpp::Node
{
public:
    UserInputNode(int num_drones);
    ~UserInputNode();

private:
    // Publishers
    rclcpp::Publisher<controller_msgs::msg::StateSet>::SharedPtr user_input_state_pub_;
    rclcpp::Publisher<controller_msgs::msg::PidSet>::SharedPtr user_input_pid_pub_;
    rclcpp::Publisher<controller_msgs::msg::ErrorThresholdSet>::SharedPtr user_input_error_thresholds_pub_;


    // One set of publishers per drone
    std::vector<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> user_input_launch_pubs_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> user_input_formation_pubs_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> user_input_return_pubs_;
    std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> user_input_command_pubs_;
    


    // Subscriber for acknowledging state
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr user_input_state_ack_sub_;

    // Timer for re-publishing states until ack is received
    rclcpp::TimerBase::SharedPtr state_send_timer_;
    bool ack_received_;
    std::string current_sending_state_;
    bool current_state_force_bool_;

    // Number of drones
    int num_drones_;

    // Thread for user input
    std::thread input_thread_;

    // Command registry
    std::map<std::string, Command> command_map_;

private:
    // Thread function
    void user_input_thread();
    void publish_user_input();  // Where we read from readline
    void process_input(const std::string &input);

    // Parse tokens into a ParsedInput object (split out flags)
    ParsedInput parse_tokens(const std::vector<std::string> &tokens);

    // ---- Timer & Subscriber callbacks ----
    void timer_send_state();
    void ack_callback(const std_msgs::msg::Bool::SharedPtr msg);

    // ---- Helper methods for sending states, etc. ----
    bool is_force_state(const std::vector<std::string> &flags);
    void send_state(const std::string &state, const bool force);
    void set_pid_message(std::shared_ptr<controller_msgs::msg::PidSet> msg,
                         char type, float kp, float ki, float kd);

    // Drone subcommand logic
    void process_coop(int drone_index, const std::string &coop_type, bool coop_value);
    void process_coop_all(const std::string &coop_type, bool coop_value);
    int  get_coop_type_index(const std::string &coop_string);

    // ---- Actual command handlers used by the dictionary ----
    void handle_start(const ParsedInput &parsed);
    void handle_pause(const ParsedInput &parsed);
    void handle_hold(const ParsedInput &parsed);
    void handle_return(const ParsedInput &parsed);
    void handle_land(const ParsedInput &parsed);
    void handle_move(const ParsedInput &parsed);
    void handle_error_thresholds(const ParsedInput &parsed);
    void handle_pid(const ParsedInput &parsed);
    void handle_drone(const ParsedInput &parsed);
    void handle_help(const ParsedInput &parsed);
    void handle_clear(const ParsedInput &parsed);

    // Helper to build the command map
    void build_command_map();
};

#endif // USER_INPUT_HPP
