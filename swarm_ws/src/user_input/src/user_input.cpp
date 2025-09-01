#include "user_input/user_input.hpp"  // Adjust to your project

// For readline
#include <readline/readline.h>
#include <readline/history.h>
// ------------------------------------------------------------------
// Implementation
// ------------------------------------------------------------------
UserInputNode::UserInputNode(int num_drones)
    : Node("user_input"), ack_received_(false), num_drones_(num_drones)
{
    // Publishers
    user_input_state_pub_ = create_publisher<controller_msgs::msg::StateSet>("/user_input/state", 10);
    user_input_pid_pub_   = create_publisher<controller_msgs::msg::PidSet>("/user_input/PID", 10);
    user_input_error_thresholds_pub_ = create_publisher<controller_msgs::msg::ErrorThresholdSet>("/user_input/error_threshold", 10);

    for (int i = 1; i <= num_drones_; ++i)
    {
        user_input_launch_pubs_.push_back(
            create_publisher<std_msgs::msg::Bool>("/drone_" + std::to_string(i) + "/user_input/coop_launch", 10));
        user_input_formation_pubs_.push_back(
            create_publisher<std_msgs::msg::Bool>("/drone_" + std::to_string(i) + "/user_input/coop_formation", 10));
        user_input_return_pubs_.push_back(
            create_publisher<std_msgs::msg::Bool>("/drone_" + std::to_string(i) + "/user_input/coop_return", 10));
        user_input_command_pubs_.push_back(
            create_publisher<std_msgs::msg::String>("/drone_" + std::to_string(i) + "/user_input/command", 10));
    }

    // Subscriber
    user_input_state_ack_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/user_input/state_ack", 10,
        std::bind(&UserInputNode::ack_callback, this, std::placeholders::_1));

    // Timer
    state_send_timer_ = create_wall_timer(std::chrono::milliseconds(1000),
                                          std::bind(&UserInputNode::timer_send_state, this));
    state_send_timer_->cancel();

    // Build the command map (populates commands & usage)
    build_command_map();

    // Launch user input thread
    input_thread_ = std::thread(&UserInputNode::user_input_thread, this);
}

UserInputNode::~UserInputNode()
{
    if (input_thread_.joinable())
    {
        input_thread_.join();
    }
}

// ------------------------------------------------------------------
// Populate command_map_ with top-level commands
// ------------------------------------------------------------------
void UserInputNode::build_command_map()
{
    command_map_["start"]  = {
        "start",
        "start [flags...] - Begin the mission (state: 'start')",
        std::bind(&UserInputNode::handle_start, this, std::placeholders::_1)
    };
    command_map_["pause"]  = {
        "pause",
        "pause [flags...] - Pause the drones (state: 'pause')",
        std::bind(&UserInputNode::handle_pause, this, std::placeholders::_1)
    };
    command_map_["hold"]   = {
        "hold",
        "hold [flags...] - Put drones into px4_hold",
        std::bind(&UserInputNode::handle_hold, this, std::placeholders::_1)
    };
    command_map_["return"] = {
        "return",
        "return [flags...] - Send drones to 'return' state",
        std::bind(&UserInputNode::handle_return, this, std::placeholders::_1)
    };
    command_map_["land"]   = {
        "land",
        "land [flags...] - Land the drones",
        std::bind(&UserInputNode::handle_land, this, std::placeholders::_1)
    };
    command_map_["move"]   = {
        "move",
        "move [flags...] - Send drones to 'move' state",
        std::bind(&UserInputNode::handle_move, this, std::placeholders::_1)
    };

    command_map_["error_thresholds"] = {
        "error_thresholds",
        "error_thresholds <pos> <alt> <head> - Set error thresholds for position, altitude, and heading",
        std::bind(&UserInputNode::handle_error_thresholds, this, std::placeholders::_1)
    };

    command_map_["pid"]    = {
        "pid",
        "pid <type> <kp> <ki> <kd> [flags...] - Publish PID update (type in {p,a,h})",
        std::bind(&UserInputNode::handle_pid, this, std::placeholders::_1)
    };
    command_map_["drone"]  = {
        "drone",
        "drone <id|all> <coop|command> <args...> [flags...] - Send commands to drone(s)",
        std::bind(&UserInputNode::handle_drone, this, std::placeholders::_1)
    };
    command_map_["help"]   = {
        "help",
        "help - Show all available commands",
        std::bind(&UserInputNode::handle_help, this, std::placeholders::_1)
    };
    command_map_["clear"]  = {
        "clear",
        "clear - Clear the terminal screen",
        std::bind(&UserInputNode::handle_clear, this, std::placeholders::_1)
    };
}

// ------------------------------------------------------------------
// Timer & Subscriber Callbacks
// ------------------------------------------------------------------
void UserInputNode::timer_send_state()
{
    if (!ack_received_)
    {
        auto msg = std::make_shared<controller_msgs::msg::StateSet>();
        msg->state_name = current_sending_state_;
        msg->force = current_state_force_bool_;
        user_input_state_pub_->publish(*msg);
        RCLCPP_INFO(get_logger(), "Re-published state: '%s'", msg->state_name.c_str());
    }
    else
    {
        if (!state_send_timer_->is_canceled())
        {
            state_send_timer_->cancel();
        }
    }
}

void UserInputNode::ack_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "STATE ACK received: %s", msg->data ? "true" : "false");
    if (msg->data)
    {
        ack_received_ = true;
    }
}

// ------------------------------------------------------------------
// User Input Thread
// ------------------------------------------------------------------
void UserInputNode::user_input_thread()
{
    while (rclcpp::ok())
    {
        publish_user_input();
    }
}

void UserInputNode::publish_user_input()
{
    char *input = readline("Enter command: ");
    if (input && *input)
    {
        add_history(input);
        std::string input_str(input);
        free(input);

        process_input(input_str);
    }
    else
    {
        if (input)
            free(input);
    }
}

ParsedInput UserInputNode::parse_tokens(const std::vector<std::string> &tokens)
{
    ParsedInput result;

    if (tokens.empty())
        return result;

    // First token is the command, if it isn't a flag
    if (!tokens[0].empty() && tokens[0].rfind("--", 0) == 0)
    {
        result.command = ""; // Treat as invalid if the first token is a flag
        result.flags.push_back(tokens[0]);
    }
    else
    {
        result.command = tokens[0];
    }

    // Process the remaining tokens
    for (size_t i = 1; i < tokens.size(); ++i)
    {
        const auto &tk = tokens[i];

        // Validate token characters
        if (!std::all_of(tk.begin(), tk.end(), [](unsigned char c) {
                return std::isalnum(c) || c == '-' || c == '_';
            }))
        {
            RCLCPP_ERROR(get_logger(), "Invalid characters in token: '%s'", tk.c_str());
            continue;
        }

        if (tk.rfind("--", 0) == 0)
        {
            // It's a flag
            result.flags.push_back(tk);
        }
        else
        {
            // It's an arg
            result.args.push_back(tk);
        }
    }

    return result;
}


// ------------------------------------------------------------------
// process_input: tokenizes and does command lookup
// ------------------------------------------------------------------
void UserInputNode::process_input(const std::string &input)
{
    // Tokenize by whitespace
    std::istringstream iss(input);
    std::vector<std::string> tokens;
    std::string token;
    while (iss >> token)
    {
        tokens.push_back(token);
    }
    if (tokens.empty())
    {
        return;
    }

    // Convert the tokens to a ParsedInput
    ParsedInput parsed = parse_tokens(tokens);

    // If there's no recognized command at all:
    if (parsed.command.empty())
    {
        RCLCPP_ERROR(get_logger(), "Could not parse command. Type 'help' for usage.");
        return;
    }

    // Lookup the command in the registry
    auto it = command_map_.find(parsed.command);
    if (it != command_map_.end())
    {
        it->second.handler(parsed);
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Unknown command '%s'. Type 'help' for commands.", parsed.command.c_str());
    }
}

// ------------------------------------------------------------------
// Helper Methods for sending state, etc.
// ------------------------------------------------------------------
void UserInputNode::send_state(const std::string &state, const bool force)
{
    current_sending_state_ = state;
    current_state_force_bool_ = force; // reset
    ack_received_          = false; // reset

    if (state_send_timer_->is_canceled())
    {
        state_send_timer_->reset();
    }
}

void UserInputNode::set_pid_message(std::shared_ptr<controller_msgs::msg::PidSet> msg,
                                    char type, float kp, float ki, float kd)
{
    msg->type = type;
    msg->kp   = kp;
    msg->ki   = ki;
    msg->kd   = kd;
}

// ------------------------------------------------------------------
// Drone coop logic
// ------------------------------------------------------------------
void UserInputNode::process_coop(int drone_index, const std::string &coop_type, bool coop_value)
{
    int index = get_coop_type_index(coop_type);
    if (index < 0)
    {
        RCLCPP_ERROR(get_logger(), "Invalid coop type: '%s'", coop_type.c_str());
        return;
    }
    auto msg = std::make_shared<std_msgs::msg::Bool>();
    msg->data = coop_value;

    switch (index)
    {
    case 0: // launch
        user_input_launch_pubs_[drone_index]->publish(*msg);
        break;
    case 1: // formation
        user_input_formation_pubs_[drone_index]->publish(*msg);
        break;
    case 2: // return
        user_input_return_pubs_[drone_index]->publish(*msg);
        break;
    }
    RCLCPP_INFO(get_logger(), "[drone %d] coop %s = %s",
                drone_index + 1, coop_type.c_str(), coop_value ? "true" : "false");
}

void UserInputNode::process_coop_all(const std::string &coop_type, bool coop_value)
{
    int index = get_coop_type_index(coop_type);
    if (index < 0)
    {
        RCLCPP_ERROR(get_logger(), "Invalid coop type: '%s'", coop_type.c_str());
        return;
    }
    auto msg = std::make_shared<std_msgs::msg::Bool>();
    msg->data = coop_value;

    for (int i = 0; i < num_drones_; ++i)
    {
        switch (index)
        {
        case 0:
            user_input_launch_pubs_[i]->publish(*msg);
            break;
        case 1:
            user_input_formation_pubs_[i]->publish(*msg);
            break;
        case 2:
            user_input_return_pubs_[i]->publish(*msg);
            break;
        }
        RCLCPP_INFO(get_logger(), "[drone %d] coop %s = %s",
                    i + 1, coop_type.c_str(), coop_value ? "true" : "false");
    }
}

int UserInputNode::get_coop_type_index(const std::string &coop_string)
{
    // 0 => launch, 1 => formation, 2 => return
    if (coop_string == "launch")    return 0;
    if (coop_string == "formation") return 1;
    if (coop_string == "return")    return 2;
    return -1;
}

// ------------------------------------------------------------------
// Actual command handlers
// ------------------------------------------------------------------

bool UserInputNode::is_force_state(const std::vector<std::string> &flags)
{
    return std::find(flags.begin(), flags.end(), "--force") != flags.end();
}


void UserInputNode::handle_start(const ParsedInput &parsed)
{
    // Check if user provided any flags like '--fast', '--verbose', etc.
    bool fast_mode = std::find(parsed.flags.begin(), parsed.flags.end(), "--fast") != parsed.flags.end();
    if (fast_mode)
    {
        RCLCPP_INFO(get_logger(), "Starting in FAST mode (this is just an example).");
    }
    send_state("start",is_force_state(parsed.flags));
}

void UserInputNode::handle_pause(const ParsedInput &parsed)
{
    bool verbose = std::find(parsed.flags.begin(), parsed.flags.end(), "--verbose") != parsed.flags.end();
    if (verbose)
    {
        RCLCPP_INFO(get_logger(), "Pausing with verbose output...");
    }
    send_state("pause",is_force_state(parsed.flags));
}

void UserInputNode::handle_hold(const ParsedInput &parsed)
{

    send_state("px4_hold", is_force_state(parsed.flags));
}

void UserInputNode::handle_return(const ParsedInput &parsed)
{
    send_state("return",is_force_state(parsed.flags));
}

void UserInputNode::handle_land(const ParsedInput &parsed)
{
    send_state("land",is_force_state(parsed.flags));
}

void UserInputNode::handle_move(const ParsedInput &parsed)
{
    send_state("move",is_force_state(parsed.flags));
}



void UserInputNode::handle_error_thresholds(const ParsedInput &parsed)
{

    bool skip = false;
    double pos_err, alt_err, head_err;





    if (parsed.flags.size() > 0) {
        if (parsed.flags[0] == "--help") {
                    RCLCPP_INFO(get_logger(), "Usage: error_thresholds <pos> <alt> <head>");
            return;
        }
        else if(parsed.flags[0] == "--default") {
            pos_err = POS_ERR_THRESHOLD_DEFAULT;
            alt_err = ALT_ERR_THRESHOLD_DEFAULT;
            head_err = HEAD_ERR_THRESHOLD_DEFAULT;
            RCLCPP_INFO(get_logger(), "Reset error thresholds to default values");
            skip = true;
        } else {
            RCLCPP_ERROR(get_logger(), "Unknown flag: '%s'", parsed.flags[0].c_str());
            return;
        }
    }
        
    if(!skip) 
    {
        // parsed.args = [ <pos>, <alt>, <head> ]
        if (parsed.args.size() < 3)
        {
            RCLCPP_ERROR(get_logger(), "Usage: error_thresholds <pos> <alt> <head>");
            return;
        }


        try {
            pos_err = std::stod(parsed.args[0]);
            alt_err = std::stod(parsed.args[1]);
            head_err = std::stod(parsed.args[2]);
        } catch (std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Invalid float values for pos/alt/head error thresholds");
            return;
        }
    }
    auto msg = std::make_shared<controller_msgs::msg::ErrorThresholdSet>();
    msg->pos = pos_err;
    msg->alt = alt_err;
    msg->heading = head_err;
    user_input_error_thresholds_pub_->publish(*msg);

    RCLCPP_INFO(get_logger(), "Published error thresholds: pos=%.3f alt=%.3f head=%.3f", pos_err, alt_err, head_err);
}


void UserInputNode::handle_pid(const ParsedInput &parsed)
{
    // parsed.args = [ <type>, <kp>, <ki>, <kd> ]  (in simplest usage)
    if (parsed.args.size() < 4)
    {
        RCLCPP_ERROR(get_logger(), "Usage: pid <type> <kp> <ki> <kd> [flags...]");
        return;
    }

    char type = parsed.args[0][0];
    float kp, ki, kd;

    try {
        kp = std::stof(parsed.args[1]);
        ki = std::stof(parsed.args[2]);
        kd = std::stof(parsed.args[3]);
    } catch (std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Invalid float values for kp/ki/kd");
        return;
    }

    if (type != 'p' && type != 'a' && type != 'h')
    {
        RCLCPP_ERROR(get_logger(), "Unknown PID type: '%c' (expected p, a, or h)", type);
        return;
    }

    auto msg = std::make_shared<controller_msgs::msg::PidSet>();
    set_pid_message(msg, type, kp, ki, kd);
    user_input_pid_pub_->publish(*msg);

    RCLCPP_INFO(get_logger(), "Published PID: type=%c kp=%.3f ki=%.3f kd=%.3f", type, kp, ki, kd);

    // Check flags, e.g. --dry-run
    bool dry_run = std::find(parsed.flags.begin(), parsed.flags.end(), "--dry-run") != parsed.flags.end();
    if (dry_run)
    {
        RCLCPP_INFO(get_logger(), "Note: You specified --dry-run (example placeholder)");
    }
}

void UserInputNode::handle_drone(const ParsedInput &parsed)
{
    // parsed.args = [ <id|all>, <coop|command>, ... ]
    if (parsed.args.size() < 2)
    {
        RCLCPP_ERROR(get_logger(), "Usage: drone <id|all> <coop|command> <args...> [flags...]");
        return;
    }

    const std::string &drone_id_str = parsed.args[0];
    const std::string &subcmd      = parsed.args[1];

    if (subcmd == "coop")
    {
        // Format: drone <id|all> coop <launch|formation|return> <true|false>
        if (parsed.args.size() < 4)
        {
            RCLCPP_ERROR(get_logger(), "Usage: drone <id|all> coop <launch|formation|return> <true|false>");
            return;
        }
        const std::string &coop_type  = parsed.args[2];
        const std::string &coop_value = parsed.args[3];
        bool value_bool = (coop_value == "true" || coop_value == "1");

        if (drone_id_str == "all")
        {
            process_coop_all(coop_type, value_bool);
        }
        else
        {
            try {
                int drone_number = std::stoi(drone_id_str);
                if (drone_number < 1 || drone_number > num_drones_)
                {
                    RCLCPP_ERROR(get_logger(), "Drone number %d is out of range", drone_number);
                    return;
                }
                process_coop(drone_number - 1, coop_type, value_bool);
            } catch (...) {
                RCLCPP_ERROR(get_logger(), "Invalid drone ID: '%s'", drone_id_str.c_str());
            }
        }
    }
    else if (subcmd == "command")
    {
        // Format: drone <id|all> command <some-string...>
        if (parsed.args.size() < 3)
        {
            RCLCPP_ERROR(get_logger(), "Usage: drone <id|all> command <string...> [flags...]");
            return;
        }

        // Gather everything after "command" into one string
        std::ostringstream oss;
        for (size_t i = 2; i < parsed.args.size(); ++i)
        {
            if (i > 2) oss << " ";
            oss << parsed.args[i];
        }
        std::string msg_str = oss.str();

        auto msg = std::make_shared<std_msgs::msg::String>();
        msg->data = msg_str;

        if (drone_id_str == "all")
        {
            for (int i = 0; i < num_drones_; i++)
            {
                user_input_command_pubs_[i]->publish(*msg);
                RCLCPP_INFO(get_logger(), "[drone %d] Published command: '%s'", i + 1, msg_str.c_str());
            }
        }
        else
        {
            try {
                int drone_number = std::stoi(drone_id_str);
                if (drone_number < 1 || drone_number > num_drones_)
                {
                    RCLCPP_ERROR(get_logger(), "Drone number %d is out of range", drone_number);
                    return;
                }
                user_input_command_pubs_[drone_number - 1]->publish(*msg);
                RCLCPP_INFO(get_logger(), "[drone %d] Published command: '%s'", drone_number, msg_str.c_str());
            } catch (...) {
                RCLCPP_ERROR(get_logger(), "Invalid drone ID: '%s'", drone_id_str.c_str());
            }
        }

        // Example of checking a flag
        bool verbose = (std::find(parsed.flags.begin(), parsed.flags.end(), "--verbose") != parsed.flags.end());
        if (verbose)
        {
            RCLCPP_INFO(get_logger(), "Published command verbosely!");
        }
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Unknown subcommand '%s' for 'drone'", subcmd.c_str());
    }
}

void UserInputNode::handle_help(const ParsedInput &parsed)
{
    (void) parsed;  // Not used
    std::cout << "\nAvailable Commands:\n";
    for (auto &kv : command_map_)
    {
        // Each kv is (string -> Command)
        const Command &cmd = kv.second;
        // Print the usage line that we stored
        std::cout << "  " << cmd.usage << std::endl;
    }
    std::cout << "\nFlags can be added to many commands (e.g. '--verbose', '--fast').\n" << std::endl;
}

void UserInputNode::handle_clear(const ParsedInput &parsed)
{
    (void) parsed; // Not used
    // Cross-platform "clear screen" approach
    std::system(CLEAR_COMMAND);
    // Alternatively, you might just print many new lines, but
    // system("clear") or system("cls") is more accurate on typical terminals.
}

// ------------------------------------------------------------------
// main
// ------------------------------------------------------------------
int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cout << "Please provide the number of drones as a command line argument.\n";
        return 1;
    }

    int num_drones = std::stoi(argv[1]);
    std::cout << "Starting User Input Node with " << num_drones << " drones.\n";

    rclcpp::init(argc, argv);
    auto node = std::make_shared<UserInputNode>(num_drones);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
