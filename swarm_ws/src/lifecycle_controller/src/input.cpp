#include "lifecycle_controller/input.hpp"
#include "logging/logging.hpp"
UserInput::UserInput(std::shared_ptr<Lifecycle> lifecycle) : lifecycle_(lifecycle)
{
    drone_name_ = lifecycle_->drone_name_;
    user_input_ = "";
    init_subs();
    init_pubs();
}

void UserInput::init_subs()
{
    // Subscribe to the default user input state topic
    state_sub_ = lifecycle_->create_subscription<controller_msgs::msg::StateSet>(
        "/user_input/state", 10, std::bind(&UserInput::state_cb, this, std::placeholders::_1));

    // Subscribe to the specific coop types for lifecycle_ drone
    coop_launch_sub_ = lifecycle_->create_subscription<std_msgs::msg::Bool>(
        drone_name_ + "/user_input/coop_launch", 10, std::bind(&UserInput::coop_launch_cb, this, std::placeholders::_1));

    coop_formation_sub_ = lifecycle_->create_subscription<std_msgs::msg::Bool>(
        drone_name_ + "/user_input/coop_formation", 10, std::bind(&UserInput::coop_formation_cb, this, std::placeholders::_1));

    coop_return_sub_ = lifecycle_->create_subscription<std_msgs::msg::Bool>(
        drone_name_ + "/user_input/coop_return", 10, std::bind(&UserInput::coop_return_cb, this, std::placeholders::_1));

    command_sub_ = lifecycle_->create_subscription<std_msgs::msg::String>(
        drone_name_ + "/user_input/command", 10, std::bind(&UserInput::command_cb, this, std::placeholders::_1));

    error_threshold_sub_ = lifecycle_->create_subscription<controller_msgs::msg::ErrorThresholdSet>(
        "/user_input/error_threshold", 10, std::bind(&UserInput::error_threshold_cb, this, std::placeholders::_1));

    PID_sub_ = lifecycle_->create_subscription<controller_msgs::msg::PidSet>(
        "/user_input/PID", 10, std::bind(&UserInput::PID_cb, this, std::placeholders::_1));
}

void UserInput::init_pubs()
{
    state_ack_pub_ = lifecycle_->create_publisher<std_msgs::msg::Bool>(
        "/user_input/state_ack", 10);
}

static inline std::string trim(std::string s)
{
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch)
                                    { return !std::isspace(ch); }));
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch)
                         { return !std::isspace(ch); })
                .base(),
            s.end());
    return s;
}

void UserInput::state_cb(const controller_msgs::msg::StateSet::SharedPtr msg)
{
    LOG_MISSION_INFO(lifecycle_, "------------------------RECEIVED STATE %s  Force: %d---------------", msg->state_name.c_str(), msg->force);
    std_msgs::msg::Bool ack{};

    user_input_ = msg->state_name;

    bool force = msg->force;
    // Check if the user input contains '--force'
    if (lifecycle_->states_.find(user_input_) != lifecycle_->states_.end())
    {
        lifecycle_->transition(user_input_, force);
    }
    else
    {
        std::string warning_message = "State doesn't exist: " + user_input_ + "\nValid states are:\n";
        for (const auto &state : lifecycle_->states_)
        {
            warning_message += "\033[33m" + state.first + "\033[0m\n";
        }


    }

    ack.data = true;
    state_ack_pub_->publish(ack);
}

void UserInput::coop_launch_cb(const std_msgs::msg::Bool::SharedPtr msg)
{


    lifecycle_->launch_coop_->set_value(msg->data);
}

void UserInput::coop_formation_cb(const std_msgs::msg::Bool::SharedPtr msg)
{

    lifecycle_->formation_coop_->set_value(msg->data);
}

void UserInput::coop_return_cb(const std_msgs::msg::Bool::SharedPtr msg)
{

    lifecycle_->return_coop_->set_value(msg->data);
}


void UserInput::command_cb(const std_msgs::msg::String::SharedPtr msg)
{
    auto command = msg->data;

    // Define a map of command handlers
    static const std::unordered_map<std::string, std::function<void()>> command_handlers = {
        {"hello", [&]() {
            LOG_MISSION_DEBUG(lifecycle_, "Command received: hello, no action taken.");
        }},
        {"disable_log_dropouts", [&]() {
            lifecycle_->dropout_checker_->setLogDropouts(0);
            LOG_MISSION_DEBUG(lifecycle_, "Command: disable_log_dropouts executed.");
        }},
        {"enable_log_dropouts", [&]() {
            lifecycle_->dropout_checker_->setLogDropouts(1);
            LOG_MISSION_DEBUG(lifecycle_, "Command: enable_log_dropouts executed.");
        }},
        {"enable_log_coops", [&]() {
            lifecycle_->launch_coop_->set_log_flag(true);
            lifecycle_->formation_coop_->set_log_flag(true);
            lifecycle_->return_coop_->set_log_flag(true);
            LOG_MISSION_DEBUG(lifecycle_, "Command: enable_log_coops executed.");
        }},
        {"disable_log_coops", [&]() {
            lifecycle_->launch_coop_->set_log_flag(false);
            lifecycle_->formation_coop_->set_log_flag(false);
            lifecycle_->return_coop_->set_log_flag(false);
            LOG_MISSION_DEBUG(lifecycle_, "Command: disable_log_coops executed.");
        }},
        {"reset_launch_coop", [&]() {
            lifecycle_->launch_coop_->reset();
            LOG_MISSION_DEBUG(lifecycle_, "Command: reset_launch_coop executed.");
        }},
        {"reset_formation_coop", [&]() {
            lifecycle_->formation_coop_->reset();
            LOG_MISSION_DEBUG(lifecycle_, "Command: reset_formation_coop executed.");
        }},
        {"reset_return_coop", [&]() {
            lifecycle_->return_coop_->reset();
            LOG_MISSION_DEBUG(lifecycle_, "Command: reset_return_coop executed.");
        }},
        {"reset_all_coops", [&]() {
            lifecycle_->launch_coop_->reset();
            lifecycle_->formation_coop_->reset();
            lifecycle_->return_coop_->reset();
            LOG_MISSION_DEBUG(lifecycle_, "Command: reset_all_coops executed.");
        }},
        {"launch_coop_input_only_enable", [&]() {
            lifecycle_->launch_coop_->set_input_only(true);
            LOG_MISSION_DEBUG(lifecycle_, "Command: launch_coop_input_only_enable executed.");
        }},
        {"launch_coop_input_only_disable", [&]() {
            lifecycle_->launch_coop_->set_input_only(false);
            LOG_MISSION_DEBUG(lifecycle_, "Command: launch_coop_input_only_disable executed.");
        }},
        {"formation_coop_input_only_enable", [&]() {
            lifecycle_->formation_coop_->set_input_only(true);
            LOG_MISSION_DEBUG(lifecycle_, "Command: formation_coop_input_only_enable executed.");
        }},
        {"formation_coop_input_only_disable", [&]() {
            lifecycle_->formation_coop_->set_input_only(false);
            LOG_MISSION_DEBUG(lifecycle_, "Command: formation_coop_input_only_disable executed.");
        }},
        {"return_coop_input_only_enable", [&]() {
            lifecycle_->return_coop_->set_input_only(true);
            LOG_MISSION_DEBUG(lifecycle_, "Command: return_coop_input_only_enable executed.");
        }},
        {"return_coop_input_only_disable", [&]() {
            lifecycle_->return_coop_->set_input_only(false);
            LOG_MISSION_DEBUG(lifecycle_, "Command: return_coop_input_only_disable executed.");
        }},
        {"reset_takeoff_error", [&]() {
            lifecycle_->takeoff_error_->reset_timer();
            LOG_MISSION_DEBUG(lifecycle_, "Command: reset_takeoff_error executed.");
        }},
        {"reset_land_error", [&]() {
            lifecycle_->land_error_->reset_timer();
            LOG_MISSION_DEBUG(lifecycle_, "Command: reset_land_error executed.");
        }},
        {"reset_ang_error", [&]() {
            lifecycle_->ang_error_->reset_timer();
            LOG_MISSION_DEBUG(lifecycle_, "Command: reset_ang_error executed.");
        }},
        {"reset_all_errors", [&]() {
            lifecycle_->takeoff_error_->reset_timer();
            lifecycle_->land_error_->reset_timer();
            lifecycle_->ang_error_->reset_timer();
            LOG_MISSION_DEBUG(lifecycle_, "Command: reset_all_errors executed.");
        }},
        {"disable_all_logs", [&]() {
            lifecycle_->dropout_checker_->setLogDropouts(0);
            lifecycle_->launch_coop_->set_log_flag(false);
            lifecycle_->formation_coop_->set_log_flag(false);
            lifecycle_->return_coop_->set_log_flag(false);
            lifecycle_->log_flag_ = false;
            LOG_MISSION_DEBUG(lifecycle_, "Command: disable_all_logs executed.");
        }},
        {"enable_all_logs", [&]() {
            lifecycle_->dropout_checker_->setLogDropouts(1);
            lifecycle_->launch_coop_->set_log_flag(true);
            lifecycle_->formation_coop_->set_log_flag(true);
            lifecycle_->return_coop_->set_log_flag(true);
            lifecycle_->log_flag_ = true;
            LOG_MISSION_DEBUG(lifecycle_, "Command: enable_all_logs executed.");
        }}
    };

    // Find and execute the command handler
    auto it = command_handlers.find(command);
    if (it != command_handlers.end())
    {
        it->second(); // Execute the handler
    }
    else
    {
        LOG_MISSION_DEBUG(lifecycle_, "Invalid command received: %s", command.c_str());
    }
}


void UserInput::error_threshold_cb(const controller_msgs::msg::ErrorThresholdSet::SharedPtr msg)
{

    LOG_MISSION_DEBUG(lifecycle_, "Received error thresholds: pos: %f, alt: %f, head: %f", msg->pos, msg->alt, msg->heading);
    lifecycle_->error_threshold_pub_->publish(*msg);
}

void UserInput::PID_cb(const controller_msgs::msg::PidSet::SharedPtr msg)
{

    lifecycle_->PID_pub_->publish(*msg);
}
