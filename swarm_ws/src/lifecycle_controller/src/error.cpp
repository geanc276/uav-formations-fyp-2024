#include "lifecycle_controller/error.hpp"

Error::Error(rclcpp::Node::SharedPtr node, std::string drone_name, std::string topic_name, std::map<std::string, double> error_targets)
    : node_(node),
      drone_name_(std::move(drone_name)),
      topic_name_(std::move(topic_name)),
      error_targets_(std::move(error_targets)),
      topic_converged_(false)
{

    init_subs();
    convergence_timer_ = node_->create_wall_timer(std::chrono::seconds(1), std::bind(&Error::convergence_cb, this));
    convergence_timer_->cancel();
}

Error::Error(rclcpp::Node::SharedPtr node, std::string drone_name)
    : node_(node),
      drone_name_(std::move(drone_name)),
      topic_converged_(false)
{
}

Error::Error(rclcpp::Node::SharedPtr node)
    : node_(node),
      drone_name_(""),
      topic_name_(""),
      topic_converged_(false)
{
}

Error::~Error() = default;

void Error::init_subs()
{
    const std::vector<std::string> topics{"distance", "altitude", "angular"};
    std::string prefix = "/" + drone_name_ + "/edge_errors/" + topic_name_ + "/";

    for (const auto &topic : topics)
    {
        auto sub = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
            prefix + topic, 10, [this, topic](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
            { this->error_cb(topic, msg); });
        error_subscribers_.insert({topic, sub});
        epsilons_[topic] = 0;
        converged_values_[topic] = false;
    }
}

void Error::start_timer()
{
    convergence_timer_->reset(); // Reset the timer to start from now
}

void Error::error_cb(const std::string &topic, const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    epsilons_[topic] = msg->data[0];
    converged_values_[topic] = epsilons_[topic] <= error_targets_.at(topic);
    if (are_all_true())
    {
        start_timer();
    }
    else
    {
        topic_converged_ = false;
        convergence_timer_->cancel();
    }
}

void Error::convergence_cb()
{
    topic_converged_ = true;
}

bool Error::are_all_true()
{
    for (const auto &pair : converged_values_)
    {
        if (!pair.second)
        {
            return false; // Exit early
        }
    }
    return true; // All values were true
}

void Error::reset_timer()
{
    convergence_timer_->reset(); // Reset the timer to start from now
}

bool Error::converged()
{
    return topic_converged_;
}