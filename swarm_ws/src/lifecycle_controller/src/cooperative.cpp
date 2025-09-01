#include "lifecycle_controller/cooperative.hpp"
#include "logging/logging.hpp"
void Cooperative::handle_subscribe_message(const std::string &drone, const std_msgs::msg::Bool::SharedPtr &msg)
{

    subscribers_sync_states_.at(drone) = msg->data;

    /*MAYBE INIT A PUBLISHER AND CHECK SYNCHRONISE WHEN THIS CB IS HIT*/
}

void Cooperative::init_subs()
{

    // if (other_drones_.empty())
    // {
    //     LOG_MISSION_ERROR(this->node_, "No other drones to synchronise with currently");
    //     // return;
    // }

    reset_sub = node_->create_subscription<std_msgs::msg::Bool>("/life_cycle/coop/" + topic_name_ + "reset", COOP_QUEUE_SIZE,
                                                                [this](const std_msgs::msg::Bool::SharedPtr msg)
                                                                { this->set_value(msg->data); });
    reset_pub = node_->create_publisher<std_msgs::msg::Bool>("/life_cycle/coop/" + topic_name_ + "reset", COOP_QUEUE_SIZE);
}

void Cooperative::init_pubs()
{
    publisher_ = node_->create_publisher<std_msgs::msg::Bool>(drone_name_ + "/life_cycle/" + topic_name_, COOP_QUEUE_SIZE);
}

void Cooperative::set_log_flag(bool flag)
{
    log_coop_flag_ = flag;
}

Cooperative::Cooperative(rclcpp::Node::SharedPtr node, std::string drone_name, std::string topic_name)
    : node_(node), drone_name_(drone_name), topic_name_(topic_name)
{
    init_subs();
    init_pubs();
    value = false;
    log_coop_flag_ = false;
    input_only_flag_ = false;
}

Cooperative::Cooperative(rclcpp::Node::SharedPtr node)
    : node_(node), drone_name_(""), topic_name_(""), value(false)
{
}


void Cooperative::add_drone_to_coop(std::string drone_name)
{
    if(drone_name == drone_name_)
    {
        return;
    }

    if (std::find(other_drones_.begin(), other_drones_.end(), drone_name) != other_drones_.end())
    {
        LOG_MISSION_DEBUG(node_, "Drone already in coop: %s", drone_name.c_str());
        return;
    }

    other_drones_.push_back(drone_name);
    add_sub_(drone_name);


}


void Cooperative::add_sub_(std::string drone)
{
    std::string topic = drone + "/life_cycle/" + topic_name_;
    auto sub = node_->create_subscription<std_msgs::msg::Bool>(
        topic, COOP_QUEUE_SIZE,
        [this, drone](const std_msgs::msg::Bool::SharedPtr msg)
        {
            this->handle_subscribe_message(drone, msg);
        });
    subscribers_.insert({drone, sub});
    subscribers_sync_states_.insert({drone, false});
    LOG_MISSION_DEBUG(node_, "Added drone to coop: %s", drone.c_str());
    for (const auto &pair : subscribers_sync_states_)
    {
        LOG_MISSION_DEBUG(node_, "Drone: %s, Sync State: %d", pair.first.c_str(), pair.second);
    }


}


bool Cooperative::synchronise()
{
    LOG_MISSION_DEBUG(node_, "Attempting to Synchronise %s", "-----");
    LOG_MISSION_DEBUG(node_, "Drone: %s, Sync State: %d", drone_name_.c_str(), value);
    LOG_MISSION_DEBUG(node_, "Other Drones: %d", other_drones_.size());

    if (!value)
    {
        LOG_MISSION_DEBUG(node_, "this drone %s is not there", drone_name_.c_str());
        for (const auto &pair : subscribers_sync_states_)
        {
            LOG_MISSION_DEBUG(node_, "Drone: %s, Sync State: %d", pair.first.c_str(), pair.second);
        }
        return false;
    }
    else
    {
        for (const auto &pair : subscribers_sync_states_)
        {
            LOG_MISSION_DEBUG(node_, "Drone: %s, Sync State: %d", pair.first.c_str(), pair.second);
            if (!pair.second)
            {
                return false;
            }
        }
    }
    return true;
}


std::vector<std::string> Cooperative::get_other_drones()
{
    return other_drones_;
}

void Cooperative::set_value(bool v)
{

    value = v;

    publisher_->publish(std_msgs::msg::Bool().set__data(value));
}

void Cooperative::reset()
{
    value = false;
    reset_pub->publish(std_msgs::msg::Bool().set__data(false));
}

void Cooperative::set_input_only(bool v)
{
    input_only_flag_ = v;
}

bool Cooperative::get_input_only()
{
    return input_only_flag_;
}