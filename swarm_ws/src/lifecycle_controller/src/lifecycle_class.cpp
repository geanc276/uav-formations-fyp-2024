#include "lifecycle_controller/lifecycle_class.hpp"
#include "logging/logging.hpp"

// !
// !
// !
// ?
// classes names: Camel case, class functions: snake case
// class members: snake case, class private members snake case_

// to check:
// run_pid constantly needed? -- trajectory setpoint
// offboard_mode before arming?
// hard coded names?



Lifecycle::Lifecycle(const std::string &drone_name, int id)
    : Node("drone_" + std::to_string(id) + "_life_cycle")
{
    printf("Lifecycle for drone %s created\n", drone_name.c_str());
    LOG_SETUP_DEBUG(this, "Lifecycle for drone %s created", drone_name.c_str());
    id_ = id;
    drone_hostname_ = drone_name;
    drone_name_ = "drone_" + std::to_string(id_);
    other_drones_ = std::unordered_map<uint8_t, std::string>();
    this->get_parameters();
    LOG_SETUP_DEBUG(this, "Lifecycle for drone %s %u finished", drone_name.c_str(), id_);
}


void Lifecycle::get_parameters()
{
    // Declare and retrieve a string parameter
    LOG_SETUP_DEBUG(this, "Getting parameters for drone %s", drone_name_.c_str());


    LOG_SETUP_DEBUG(this, "Getting config file path");
    this->declare_parameter<std::string>("config_file_path", "default_config.yaml");
    this->get_parameter("config_file_path", config_file_path_);


    LOG_SETUP_DEBUG(this, "Getting State config file path");
    this->declare_parameter<std::string>("state_config_file_path", "default_state_config.yaml");
    this->get_parameter("state_config_file_path", state_config_file_path_); 


    LOG_SETUP_DEBUG(this, "Getting num_drones");
    num_drones_ = -999;
    try {
        this->declare_parameter<int>("num_drones", 1);
        this->get_parameter("num_drones", num_drones_);
    } 
    catch (const rclcpp::ParameterTypeException &e) {
    }


}

std::shared_ptr<Lifecycle> Lifecycle::create(const std::string &drone_name, int id)
{
    auto instance = std::make_shared<Lifecycle>(drone_name, id);
    instance->init();
    return instance;
}

void Lifecycle::init()
{
    std::vector<std::string> topics{"distance", "altitude", "angular"};

    std::string takeoff_str = "takeoff";
    std::string land_str = "land";
    std::string ang_str = "ang";

    double distance_error = 0.2;
    double alt_error = 0.2;
    double ang_error = 5;

    std::map<std::string, double> error_target_map;
    error_target_map[topics[0]] = distance_error;
    error_target_map[topics[1]] = alt_error;
    error_target_map[topics[2]] = ang_error;

    int dropout_time_threshold_seconds_ = 1;
    int dropout_check_interval_seconds = 2;




    try
    {
        launch_coop_ = std::make_unique<Cooperative>(shared_from_this(), drone_name_, "launch");
        formation_coop_ = std::make_unique<Cooperative>(shared_from_this(), drone_name_, "formation");
        return_coop_ = std::make_unique<Cooperative>(shared_from_this(), drone_name_, "return");
    //     dropout_checker_ = std::make_unique<Dropout>(shared_from_this(), drone_name_, other_drones_, dropout_time_threshold_seconds_, dropout_check_interval_seconds);
        fcu_ = std::make_unique<FCU>(shared_from_this(), drone_name_, id_);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error during initialization: eeeeeeeeeeeeeeeeee" << e.what() << std::endl;
        std::cerr << "Error during initialization: " << e.what() << std::endl;
        throw;
    }

    // if (!launch_coop_ || !formation_coop_ || !return_coop_ || !dropout_checker_ || !fcu_)
    // {
    //     throw std::runtime_error("Failed to initialize one or more components");
    // }

    fcu_->set_takeoff_height(takeoff_height_);

    this->init_subs();
    this->init_pubs();


    register_drone_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&Lifecycle::register_drone_timer_cb, this));

    off_board_ = false;
    auto timer_callback = [this]() -> void
    {
        LOG_SETUP_DEBUG(this, "Publishing off board control mode");
        
        // ! this is essentially a "im alive" publish to px4 to trust to switch teh offboard
        // ! must be at a rate of 2Hz of more otherwise px4 will think something is wrong and switch to landing mode
        fcu_->publish_off_board_control_mode();

    };
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);
    LOG_SETUP_INFO(this, "Lifecycle for drone %s initialized", drone_name_.c_str());

    this->init_status();

}





/************ Subscriptions ************/
void Lifecycle::init_subs()
{

    /* Quality of Service set-up for PX4 topics. */
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    status_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/px4_" + std::to_string(id_) + "/fmu/out/vehicle_status", qos, std::bind(&Lifecycle::state_cb, this, std::placeholders::_1));

    rendezvous_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/px4_" + std::to_string(id_) + "/rendezvous_status", qos, std::bind(&Lifecycle::rendezvous_done_cb, this, std::placeholders::_1));

    mode_complete_sub_ = this->create_subscription<px4_msgs::msg::ModeCompleted>(
        "/px4_" + std::to_string(id_) + "/fmu/out/mode_completed", qos, std::bind(&Lifecycle::mode_complete_cb, this, std::placeholders::_1));

    vehicle_command_ack_sub_ = this->create_subscription<px4_msgs::msg::VehicleCommandAck>(
        "/px4_" + std::to_string(id_) + "/fmu/out/vehicle_command_ack", qos, std::bind(&Lifecycle::vehicle_command_ack_cb, this, std::placeholders::_1));

    register_drone_ack_sub_ = this->create_subscription<controller_msgs::msg::RegisterDroneAck>(
        "/register_drone_ack", LIFECYCLE_QUEUE_SIZE, std::bind(&Lifecycle::register_drone_ack_cb, this, std::placeholders::_1));
    
    update_drone_list_sub_ = this->create_subscription<controller_msgs::msg::UpdateDroneList>(
        "/update_drone_list", LIFECYCLE_QUEUE_SIZE, std::bind(&Lifecycle::update_drone_list_cb, this, std::placeholders::_1));
    
    LOG_SETUP_INFO(this, "Subscriptions for drone %s initialized", drone_name_.c_str());
}


/************ Publishers ************/
void Lifecycle::init_pubs()
{

    register_drone_pub_ = this->create_publisher<controller_msgs::msg::RegisterDrone>(
        "/register_drone", LIFECYCLE_QUEUE_SIZE);

    ctrl_state_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
        "/ctrl/state", LIFECYCLE_QUEUE_SIZE);

    error_threshold_pub_ = this->create_publisher<controller_msgs::msg::ErrorThresholdSet>(
        "/ctrl/error_threshold_set", 10);

    PID_pub_ = this->create_publisher<controller_msgs::msg::PidSet>(
        "/" + drone_name_ + "/ctrl/pid_set", 10);

    status_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "/status", 10);



    LOG_SETUP_INFO(this, "Publishers for drone %s initialized", drone_name_.c_str());

}







void Lifecycle::register_drone_timer_cb(void)
{
    LOG_SETUP_DEBUG(this, "Registering drone %s", drone_name_.c_str());
    auto msg = controller_msgs::msg::RegisterDrone();
    msg.px4_id = id_;
    msg.name = drone_name_;
    register_drone_pub_->publish(msg);
}



void Lifecycle::register_drone_ack_cb(const controller_msgs::msg::RegisterDroneAck::SharedPtr msg)
{
    LOG_SETUP_INFO(this, "Drone %s registered", drone_name_.c_str());
    if (msg->id == id_)
    {
        register_drone_timer_->cancel();
    }
}


void Lifecycle::update_drone_list_cb(const controller_msgs::msg::UpdateDroneList::SharedPtr msg)
{
    LOG_SETUP_INFO(this, "Drone list updated");
    auto size = msg->size;
    auto px4_ids = msg->px4_ids;
    LOG_SETUP_INFO(this, "UPDATE SIZE: %d", size);
    LOG_SETUP_INFO(this, "Drone %s", drone_name_.c_str());


    for (uint i = 0; i < size; i++)
    {
        LOG_SETUP_INFO(this, "PX4 ID: %d", px4_ids[i]);
        LOG_SETUP_INFO(this, "ID: %d", id_);
        if (px4_ids[i] != id_)
        {
            LOG_SETUP_INFO(this, "Looking for drones inside %d", id_);
            if (other_drones_.find(px4_ids[i]) == other_drones_.end())
            {
                LOG_SETUP_INFO(this, "Found one %d", px4_ids[i]);
                other_drones_[px4_ids[i]] = "drone_" + std::to_string(px4_ids[i]);
                LOG_SETUP_INFO(this, "Drone %d added to other drones", px4_ids[i]);
                add_drone_to_coops(other_drones_[px4_ids[i]]);
                LOG_SETUP_INFO(this, "Drone %s added to coops", other_drones_[px4_ids[i]].c_str());
            }
            else if (other_drones_.find(px4_ids[i]) != other_drones_.end())
            {
                LOG_SETUP_INFO(this, "Drone %d already in other drones", px4_ids[i]);
                for (auto drone : other_drones_)
                {
                    LOG_SETUP_INFO(this, "IN OTHER_Drone %s", drone.second.c_str());
                }
                LOG_SETUP_INFO(this, "Drone -------------------------%d", px4_ids[i]);
            }
        }
    }
}


void Lifecycle::add_drone_to_coops(const std::string &drone_name)
{
    launch_coop_->add_drone_to_coop(drone_name);
    formation_coop_->add_drone_to_coop(drone_name);
    return_coop_->add_drone_to_coop(drone_name);

    LOG_SETUP_INFO(this, "Drone %s added to coops", drone_name.c_str());
    LOG_SETUP_INFO(this, "-----------------------------------");
    for(std::string drone : formation_coop_->get_other_drones())
    {
        LOG_SETUP_INFO(this, "Drone from coops:  %s", drone.c_str());
    }
    LOG_SETUP_INFO(this, "-----------------------------------");
}   



/************ Publishing Functions ************/
void Lifecycle::publish_ctrl_state(const std::string &state)
{
    uint8_t num_state;

    num_state = convert_state_to_int(state);
    auto msg = std_msgs::msg::UInt8();
    msg.data = num_state;

    ctrl_state_pub_->publish(msg);
}



void Lifecycle::init_status()
{
    status = Status(drone_name_, id_, "LANDED");
    status.set_publisher(status_publisher_);
    status_timer_= this->create_wall_timer(std::chrono::milliseconds(5000), std::bind(&Lifecycle::gather_status, this));
    LOG_SETUP_INFO(this, "Status for drone %s initialized", drone_name_.c_str());
}

/************ Callbacks ************/



void Lifecycle::state_cb(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{

    // Checks if manual RC overide has occurred.
    if (msg->nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD && off_board_)
    {
        LOG_MISSION_DEBUG(this, "Manual RC override detected");
    }

}



void Lifecycle::gather_status(void)
{
    LOG_MISSION_DEBUG(this, "------------------------------------------");
    LOG_MISSION_DEBUG(this, "------------------------------------------");
    LOG_MISSION_DEBUG(this, "------------------------------------------");
    LOG_MISSION_DEBUG(this, "Gathering status for drone %s", drone_name_.c_str());
    LOG_MISSION_DEBUG(this, "Gathering status for drone %s", drone_name_.c_str());
    LOG_MISSION_DEBUG(this, "------------------------------------------");
    LOG_MISSION_DEBUG(this, "------------------------------------------");
    LOG_MISSION_DEBUG(this, "------------------------------------------");

    uint8_t nav_state;
    uint8_t armed;
    float_t battery_percentage;
    float_t battery_voltage;
    float_t battery_current;

    // Retrieve status from FCU
    fcu_->get_vehicle_status(nav_state, armed);

    fcu_->get_battery_status(battery_percentage, battery_voltage, battery_current);
    // Update the Status object using setter methods
    status.set_armed(armed);
    // Assuming you have methods to set nav_state if needed
    
    status.set_battery_percentage(battery_percentage);

    status.set_battery_voltage(battery_voltage);

    status.set_battery_current(battery_current);

    // Update the state
    auto state_name = get_current_state();
    status.set_state(state_name);

    // Publish the updated status
    status.publish_status();
    LOG_MISSION_DEBUG(this, "%s", status.get_status().dump().c_str());
}


State *Lifecycle::get_state_by_name(const std::string &name)
{
    auto it = states_.find(name);
    if (it != states_.end())
    {
        return &it->second;
    }
    return nullptr;
}

void Lifecycle::rendezvous_done_cb(const std_msgs::msg::Bool::SharedPtr msg)
{
    LOG_MISSION_DEBUG(this, "Rendezvous done Received: %d", msg->data);
    if (msg->data)
    {
        LOG_MISSION_INFO(this, "Rendezvous for drone %s done", drone_name_.c_str());
        formation_coop_->set_value(true);
        if (formation_coop_->synchronise())
        {
            LOG_MISSION_INFO(this, "Renendezvous for all drones done");
            transition("move");
        }
    }
    else
    {

        formation_coop_->set_value(false);
    }
}

void Lifecycle::mode_complete_cb(const px4_msgs::msg::ModeCompleted::SharedPtr msg)
{

    LOG_MISSION_DEBUG(this, "Mode completed: state:%d, state: %d",msg->nav_state, msg->result ? "false" : "true");
    if (msg->result != 0)
    {
        LOG_MISSION_ERROR(this, "Failed to set mode for drone %s", drone_name_.c_str());
        return;
    }
    if (msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF)
    {
        LOG_MISSION_INFO(this, "Takeoff for drone %s done", drone_name_.c_str());
        transition("rendezvous");
    }
}

void Lifecycle::vehicle_command_ack_cb(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
{
    LOG_MISSION_DEBUG(this, "Vehicle command ack: %d", msg->result);

    if (msg->command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM)
    {
        if (msg->result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED)
        {

            transition("takeoff");
        }
        else
        {
            LOG_MISSION_ERROR(this, "Failed to arm drone %s", drone_name_.c_str());
        }
    }
}

/************ State Transitions ************/
bool Lifecycle::transition(const std::string &nextState, bool force)
{
    LOG_MISSION_INFO(this, "Transitioning from %s to %s", get_current_state().c_str(), nextState.c_str());
    if (current_state_)
    {


        if (force || current_state_->is_valid_transition(nextState))
        {
            if (current_state_)
            {
                current_state_->exit();
            }
            current_state_ = &states_[nextState];
            if (current_state_)
            {

                off_board_ = current_state_->is_off_board();
                if (off_board_)
                {
                    // off_board_set_point_ = 0;
                    // 1 is custom, 6 is offboard
                    // ! in theory the alive message should be sending constantly, so should be able to arm straight away;
                    // ! this simplifies the PID control as it should always be sending its updates
                    // ! read somewhere that only need to send alive singals (vv) not trajecotry updates, but shall find out
                    // fcu_->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, param1 = 1, param2 = 6);
                    fcu_->publish_off_board();
                }
                current_state_->enter();
            }

            publish_ctrl_state(nextState);
            return true;
        }
    }

    std::string error_message = "\033[31mInvalid transition from " + current_state_->get_name() + " to " + nextState + "\033[0m\n";
    std::string info_message = "Valid transitions are:\n";
    for (const auto &transition : current_state_->get_valid_transitions())
    {
        info_message += "\033[32m- " + transition + "\033[0m\n";
    }
    LOG_MISSION_ERROR(this, "%s",error_message.c_str());
    LOG_MISSION_DEBUG(this, "%s",info_message.c_str());



    return false;
}

std::string Lifecycle::get_state_config_file_path() const
{
    return state_config_file_path_;
}

std::string Lifecycle::get_current_state() const
{
    if (current_state_)
    {
        return current_state_->get_name();
    }
    return "Undefined State";
}

double Lifecycle::get_takeoff_height() const
{
    return takeoff_height_;
}

bool Lifecycle::is_any_data_stale()
{
    return dropout_checker_->getDropoutFlag();
}

/************ Enter and Exit Actions ************/
/************ Action Map ************/

/************ Main Run Loop ************/
void Lifecycle::run()
{
    LOG_SETUP_DEBUG(this, "Running lifecycle for drone %s", drone_name_.c_str());
}
