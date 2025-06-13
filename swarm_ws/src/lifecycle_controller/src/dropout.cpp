#include "lifecycle_controller/dropout.hpp"
#include "logging/logging.hpp"
// TODO FIGURE OUT OTHER_DRONES
// ! DROP OUT IS THE MAIN USE OF OTHER DRONES RN

Dropout::Dropout(std::shared_ptr<rclcpp::Node> node, std::string drone_name, std::vector<std::string> other_drones, int dropout_time_threshold, int check_interval)
    : node_(node), drone_name_(std::move(drone_name)), other_drones_(std::move(other_drones)), dropout_time_threshold_(dropout_time_threshold * 1000), check_interval_(check_interval),
      dropout_flag_(false), io_context_(), work_(boost::asio::make_work_guard(io_context_))
{
    // ! dont forget that this is here!!!!!
    if (other_drones_.empty())
    {

        return;
    }


    initSubs();
    initServiceClients();
    initService();


    log_times_[drone_name_] = node_->now().seconds() + node_->now().nanoseconds() * 1e-9;
    dropout_check_timer_ = node_->create_wall_timer(
        std::chrono::seconds(check_interval_), std::bind(&Dropout::checkForDropouts, this));
    io_thread_ = std::thread([this]()
                             { io_context_.run(); });

    log_flag_ = true;
}

Dropout::~Dropout() = default;

void Dropout::initSubs()
{
    // ! dont think this function is needed
    for (const auto &drone : other_drones_)
    {
        std::string topic = "/" + drone + "/mavros/local_position/pose";
        subscribers_.push_back(node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            topic, 10, [this, drone](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            { dropoutCb(msg, drone); }));
        log_times_[drone] = node_->now().seconds() + node_->now().nanoseconds() * 1e-9;
    }
}
void Dropout::initServiceClients()
{
    for (const auto &drone : other_drones_)
    {
        std::string service_name = "/" + drone + "/trigger";
        alive_check_clients_[drone] = node_->create_client<std_srvs::srv::Trigger>(service_name);
    }
}

void Dropout::initService()
{
    std::string service_name = "/" + drone_name_ + "/trigger";

    service_ = node_->create_service<std_srvs::srv::Trigger>(
        service_name, std::bind(&Dropout::handleTrigger, this, std::placeholders::_1, std::placeholders::_2));
}

void Dropout::handleTrigger(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    response->success = true;
}

void Dropout::dropoutCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg, const std::string &drone)
{
    // ! proabably not needed
    log_times_[drone] = node_->now().seconds() + node_->now().nanoseconds() * 1e-9;

}

bool Dropout::getDropoutFlag()
{
    return dropout_flag_;
}

void Dropout::checkForDropouts()
{
    for (const auto &drone : other_drones_)
    {
        auto client = alive_check_clients_[drone];

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = client->async_send_request(request);
        auto future_ptr = std::make_shared<decltype(future)>(std::move(future));

        // Schedule a timer for the dropout wait time
        auto timer = std::make_shared<boost::asio::steady_timer>(io_context_, std::chrono::milliseconds(dropout_time_threshold_));
        timer->async_wait([this, drone, future_ptr, timer](const boost::system::error_code &ec) mutable
                          {
            if (ec) {

                return;
            }

            if (future_ptr->wait_for(std::chrono::seconds(0)) == std::future_status::timeout) {

                handleDropout(drone);
            } else if (future_ptr->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                auto response = future_ptr->get();
                if (!response->success) {

                    handleDropout(drone);
                }
            } });
    }
}

void Dropout::handleDropout(const std::string &drone)
{
    LOG_MISSION_WARN(node_, "Drone %s has dropped out", drone.c_str());
    dropout_flags_[drone] = true;
}

void Dropout::setLogDropouts(bool log_flag)
{
    log_flag_ = log_flag;
}