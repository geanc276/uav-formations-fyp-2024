#include "dcp/srpClient.h"
#include "dcp_msgs/msg/register_protocol_request.hpp"
#include "dcp_msgs/msg/transmit_payload_request.hpp"

namespace dcp
{
using namespace std::chrono_literals;
/*
    Node wrapping class for State Reporting Protocol
*/
class SRPClientNode : public rclcpp::Node
{
public:
    SRPClientNode(const rclcpp::NodeOptions & options)
    : Node ("SRPClientNode", options)
    {
        std::string name = this->get_name();
        const char* envDroneName = std::getenv("DRONE_NAME");
        std::stringstream ss;

        ss << "Initializing SRP publishers... ";
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());

        beaconingClientSender_ = this->create_publisher<dcp_msgs::msg::RegisterProtocolRequest>(std::string(envDroneName) + "/beaconing/register/protocol", 1);
        beaconingPayloadTransmitter_ = this->create_publisher<dcp_msgs::msg::TransmitPayloadRequest>(std::string(envDroneName) + "/beaconing/payload/transmit", 10);

        ss.str(""); // Clear the contents of ss
        ss << "Initialized SRP publishers... " << beaconingClientSender_->get_topic_name() << ", "
                                               << beaconingPayloadTransmitter_->get_topic_name();
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());

        controller_.reset(new SRPClient(beaconingClientSender_, beaconingPayloadTransmitter_));

        ss.str(""); // Clear the contents of ss
        ss << "Initializing SRP subscribers... ";
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        stateDataSub_ = this->create_subscription<std_msgs::msg::String>(
                std::string(envDroneName) + "/drone_name/mavros/global_position/global",
                10,
                std::bind(&SRPClient::callback_position, controller_, std::placeholders::_1));

        bpIndicationSub_ = this->create_subscription<dcp_msgs::msg::ReceivePayloadIndication>(
                std::string(envDroneName) + "/receive/payload/indication",
                10,
                std::bind(&SRPClient::callback_indications, controller_, std::placeholders::_1));

        ss.str(""); // Clear the contents of ss
        ss << "Initialized SRP subscribers... " << stateDataSub_->get_topic_name() << ", "
                                                << bpIndicationSub_->get_topic_name();
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());

        dropEntriesTimer_ = this->create_wall_timer(SRP_TRAVERSAL_TIME, std::bind(&SRPClient::checkEntriesToDrop, controller_));

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        controller_->registerClientProtocol();
    }
private:
    // Shared pointer reference to underlying state reporting protocol class
    std::shared_ptr<SRPClient> controller_;
    // ROS Subscriber to be used for collecting drone state data from mavros
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stateDataSub_;
    // ROS Subscriber to listen for payload transmitted indications from the Beaconing Protocol
    rclcpp::Subscription<dcp_msgs::msg::ReceivePayloadIndication>::SharedPtr bpIndicationSub_;
    // ROS Publisher for registering as a client protocol on the Beaconing Protocol
    rclcpp::Publisher<dcp_msgs::msg::RegisterProtocolRequest>::SharedPtr beaconingClientSender_;
    // ROS Publisher for sending payloads for transmission to the Beaconing Protocol
    rclcpp::Publisher<dcp_msgs::msg::TransmitPayloadRequest>::SharedPtr beaconingPayloadTransmitter_;
    // ROS Timer to delay SRP registering with the Beaconing Protocol by 1 second
    rclcpp::TimerBase::SharedPtr startupTimer_;
    // Checks if there are any entries to drop.
    rclcpp::TimerBase::SharedPtr dropEntriesTimer_;

};


}; // namespace

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dcp::SRPClientNode);