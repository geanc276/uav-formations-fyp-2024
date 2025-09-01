#include <iostream>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "dcp/beaconingClient.h"


namespace dcp

{
using namespace std::chrono_literals;

/*
    Nodelet wrapping class for Beaconing Protocol
*/
class BeaconingClientNode: public rclcpp::Node
{
public:
    BeaconingClientNode(const rclcpp::NodeOptions & options)
    : Node ("beaconingClientNode", options)
    {
        // Get ROS node name
        const char *name = this->get_name();
        const char* envDroneName = std::getenv("DRONE_NAME");
        std::stringstream ss;

        ss << "Initializing BP publishers... ";
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());

        srpClientSender_ = this->create_publisher<dcp_msgs::msg::ReceivePayloadIndication>(std::string(envDroneName) + "/receive/payload/indication", 10);
        payloadTransmittedIndication_ = this->create_publisher<dcp_msgs::msg::PayloadTransmittedIndication>(std::string(envDroneName) + "/transmit/payload/indication", 10);
        beaconingClientSender_ = this->create_publisher<std_msgs::msg::String>(std::string(envDroneName) + "/drone_name/mavros/global_position/global", 10);
        queryBufferResponsePublisher_ = this->create_publisher<dcp_msgs::msg::QueryNumberBufferedPayloadsResponse>(std::string(envDroneName) + "/vardis/queryBuffer", 10);

        ss.str(""); // Clear the contents of ss
        ss << "Initialized BP publishers... " << srpClientSender_->get_topic_name() << ", "
                                              << beaconingClientSender_->get_topic_name() << ", "
                                              << queryBufferResponsePublisher_->get_topic_name();
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());

        controller_.reset(new BeaconingClient(queryBufferResponsePublisher_, srpClientSender_, payloadTransmittedIndication_));

        rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(5))   // Specify the QoS depth
                .reliable()                      // Set the reliability (e.g., reliable or best-effort)
                .durability_volatile();

        ss.str(""); // Clear the contents of ss
        ss << "Initializing BP subscribers... ";
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        protocolRegisterSub_ = this->create_subscription<dcp_msgs::msg::RegisterProtocolRequest>(
                std::string(envDroneName) + "/beaconing/register/protocol",
                1,
                std::bind(&BeaconingClient::registerProtocolCallback, controller_, std::placeholders::_1));

        protocolTransmitPayloadSub_ = this->create_subscription<dcp_msgs::msg::TransmitPayloadRequest>(
                std::string(envDroneName) + "/beaconing/payload/transmit",
                10,
                std::bind(&BeaconingClient::transmitPayloadCallback, controller_, std::placeholders::_1));

        protocolQueryBufferedPayloads_ = this->create_subscription<dcp_msgs::msg::QueryNumberBufferedPayloadsRequest>(
                std::string(envDroneName) + "/beaconing/queryBuffer",
                10,
                std::bind(&BeaconingClient::queryNumberBufferedPayloads, controller_, std::placeholders::_1));

        ss.str(""); // Clear the contents of ss
        ss << "Initialized BP subscribers... " << protocolRegisterSub_->get_topic_name() << ", "
                                               << protocolTransmitPayloadSub_->get_topic_name() << ", "
                                               << protocolQueryBufferedPayloads_->get_topic_name();
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());

        testingTimer_ = this->create_wall_timer(2000ms, std::bind(&BeaconingClientNode::testingCb, this));
        beaconingProcessTransmit_ = this->create_wall_timer(100ms, std::bind(&BeaconingClient::runBeaconingTransmit, controller_));
        writeResultsToFile_ = this->create_wall_timer(10000ms, std::bind(&BeaconingClient::writeResultsToFile, controller_));
//        beaconingProcessReceive_ = this->create_wall_timer(900ms, std::bind(&BeaconingClient::runBeaconingReceive, controller_));
        controller_->runBeaconingReceive();
//        controller_->runBeaconingTransmit();
    }
private:
    /*
    ROS callback method to send dummy safety data
    */
    void testingCb(void)
    {
        RCLCPP_INFO(this->get_logger(), "Publishing safety data.");
        std_msgs::msg::String msg;
        msg.data = "Test Data";
        beaconingClientSender_->publish(msg);

    }

    // Shared pointer reference to underlying beaconing protocol class
    std::shared_ptr<BeaconingClient> controller_;
    // Sets beaconing transmit frequency
    rclcpp::TimerBase::SharedPtr beaconingProcessTransmit_;

    rclcpp::TimerBase::SharedPtr writeResultsToFile_;
    // Sets beaconing receive frequency
    rclcpp::TimerBase::SharedPtr beaconingProcessReceive_;
    // Timer for calling test safety data callback
    rclcpp::TimerBase::SharedPtr testingTimer_;
    // ROS Subscriber for client protocol register messages
    rclcpp::Subscription<dcp_msgs::msg::RegisterProtocolRequest>::SharedPtr protocolRegisterSub_;
    // ROS Subscriber for client protocol transmit messages
    rclcpp::Subscription<dcp_msgs::msg::TransmitPayloadRequest>::SharedPtr protocolTransmitPayloadSub_;
    // ROS Subscriber for client protocols to check queue buffer
    rclcpp::Subscription<dcp_msgs::msg::QueryNumberBufferedPayloadsRequest>::SharedPtr protocolQueryBufferedPayloads_;
    // ROS Publisher for sending dummy safety data message
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr beaconingClientSender_;
    // ROS Publisher for sending payload transmitted indications to SRP
    rclcpp::Publisher<dcp_msgs::msg::ReceivePayloadIndication>::SharedPtr srpClientSender_;
    // ROS Publisher for sending output of queryNumberBufferedPayloads function
    rclcpp::Publisher<dcp_msgs::msg::QueryNumberBufferedPayloadsResponse>::SharedPtr queryBufferResponsePublisher_;
    // ROS Publisher to notify other protocols that their payload has been transmitted
    rclcpp::Publisher<dcp_msgs::msg::PayloadTransmittedIndication>::SharedPtr payloadTransmittedIndication_;

};
}; // namespace

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dcp::BeaconingClientNode);