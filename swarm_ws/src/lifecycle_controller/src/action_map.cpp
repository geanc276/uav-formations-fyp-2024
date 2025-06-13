#include "lifecycle_controller/action_map.hpp"
#include "logging/logging.hpp"

ActionMap::ActionMap(std::shared_ptr<Lifecycle> lifecycle, const std::string &config_file) : lifecycle_(lifecycle)
{
    make_action_map();
    read_states_from_config(config_file);
}

ActionMap::~ActionMap()
{
}

/************ State Configuration ************/
void ActionMap::read_states_from_config(const std::string &config_file)
{
    std::ifstream file(config_file);
    if (!file.is_open())
    {
        LOG_SETUP_FATAL(lifecycle_, "Unable to open file: %s", config_file.c_str());
        return;
    }
    nlohmann::json json_config;
    file >> json_config;
    std::string begin_state = "configure";
    for (const auto &item : json_config)
    {

        std::string state_name = item["state"];
        std::vector<std::string> transitions = item["valid"];

        if (item.find("begin") != item.end())
        {
            if (item["begin"] == true)
            {
                begin_state = state_name;
            }
        }
        std::string enter_action = item.value("onEnter", "None");
        std::string exit_action = item.value("onExit", "None");
        bool off_board = item.value("off_board", false);

        std::function<void()> enter_func = (enter_action != "None" && action_map.find(enter_action) != action_map.end()) ? action_map[enter_action] : nullptr;
        std::function<void()> exit_func = (exit_action != "None" && action_map.find(exit_action) != action_map.end()) ? action_map[exit_action] : nullptr;
        std::vector<std::function<void()>> enter_funcs;
        std::vector<std::function<void()>> exit_funcs;
        enter_funcs.push_back(enter_func);
        exit_funcs.push_back(exit_func);
        auto boundLogEnter = std::bind(&ActionMap::logEnter, this, std::placeholders::_1);
        auto boundLogExit = std::bind(&ActionMap::logExit, this, std::placeholders::_1);

        // Explicitly construct the State object
        State newState(state_name, transitions, enter_funcs, exit_funcs, boundLogEnter, boundLogExit, off_board);

        // Use emplace with the correct arguments
        lifecycle_->states_.emplace(state_name, std::move(newState));
        

    }

    if (!lifecycle_->states_.empty())
    {
        lifecycle_->current_state_ = lifecycle_->get_state_by_name(begin_state);
        if (lifecycle_->current_state_)
        {
            lifecycle_->current_state_->enter();
        }
    }
    std::string all_states_str = "";
    for (const auto &state : lifecycle_->states_)
    {
        all_states_str += state.first + ", ";
    }
    LOG_SETUP_DEBUG(lifecycle_, "States: %s", all_states_str.c_str());
    LOG_SETUP_DEBUG(lifecycle_, "Begin state: %s", begin_state.c_str());
}

void ActionMap::make_action_map()
{
    action_map["enter_launch"] = [this]()
    { enter_launch(); };
    action_map["exit_launch"] = [this]()
    { exit_launch(); };

    action_map["enter_rendezvous"] = [this]()
    { enter_rendezvous(); };
    action_map["exit_rendezvous"] = [this]()
    { exit_rendezvous(); };

    action_map["enter_pause"] = [this]()
    { enter_pause(); };
    action_map["exit_pause"] = [this]()
    { exit_pause(); };

    action_map["enter_configure"] = [this]()
    { enter_configure(); };
    action_map["exit_configure"] = [this]()
    { exit_configure(); };

    action_map["enter_start"] = [this]()
    { enter_start(); };
    action_map["exit_start"] = [this]()
    { exit_start(); };

    action_map["enter_takeoff"] = [this]()
    { enter_takeoff(); };
    action_map["exit_takeoff"] = [this]()
    { exit_takeoff(); };

    action_map["enter_move"] = [this]()
    { enter_move(); };
    action_map["exit_move"] = [this]()
    { exit_move(); };

    action_map["enter_px4_hold"] = [this]()
    { enter_px4_hold(); };
    action_map["exit_px4_hold"] = [this]()
    { exit_px4_hold(); };

    action_map["enter_return"] = [this]()
    { enter_return(); };
    action_map["exit_return"] = [this]()
    { exit_return(); };

    action_map["enter_land"] = [this]()
    { enter_land(); };
    action_map["exit_land"] = [this]()
    { exit_land(); };
}
void ActionMap::enter_launch()
{
    // fcu_->set_armed(true);
}

void ActionMap::exit_launch()
{
    lifecycle_->publish_ctrl_state("rendezvous");
}

void ActionMap::enter_rendezvous()
{
}

void ActionMap::exit_rendezvous()
{
    lifecycle_->launch_coop_->set_value(false);
}

void ActionMap::enter_pause()
{
    lifecycle_->formation_coop_->set_value(false);
    lifecycle_->takeoff_error_->reset_timer();
}

void ActionMap::exit_pause()
{
}

void ActionMap::enter_configure()
{
}

void ActionMap::exit_configure()
{
}


void ActionMap::enter_start()
{
    LOG_MISSION_DEBUG(lifecycle_, "ENTERED_START");
    lifecycle_->fcu_->publish_arm();
}

void ActionMap::exit_start()
{
}

void ActionMap::enter_takeoff()
{
    lifecycle_->fcu_->publish_takeoff();
}

void ActionMap::exit_takeoff()
{
}

void ActionMap::enter_move()
{
}

void ActionMap::exit_move()
{
}

void ActionMap::enter_px4_hold()
{
    // pfuc.publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4);
    // 7 is 0 = relative, 3 = hold height
    // fcu_->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_REPOSITION, param7 = get_takeoff_height(), param3 = 0);
    lifecycle_->fcu_->publish_hold();
}

void ActionMap::exit_px4_hold()
{
}

void ActionMap::enter_return()
{
    // fcu_->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
    lifecycle_->fcu_->publish_return();
}

void ActionMap::exit_return()
{
}

void ActionMap::enter_land()
{
    // fcu_->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND, param1 = 0, param2 = 0);
    lifecycle_->fcu_->publish_land();
}

void ActionMap::exit_land()
{
}

void ActionMap::logEnter(const std::string &state)
{
    LOG_MISSION_INFO(lifecycle_, "Entering state: %s", state.c_str());

}

void ActionMap::logExit(const std::string &state)
{
    LOG_MISSION_INFO(lifecycle_, "Exiting state: %s", state.c_str());
}
