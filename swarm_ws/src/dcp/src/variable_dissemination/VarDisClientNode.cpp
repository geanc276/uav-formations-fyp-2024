#include <iostream>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "dcp/VarDisClient.h"
#include "std_msgs/msg/string.hpp"

namespace dcp

{
    using namespace std::chrono_literals;

/*
    Nodelet wrapping class for Beaconing Protocol
*/
    class VarDisClientNode: public rclcpp::Node
    {
    public:
        VarDisClientNode(const rclcpp::NodeOptions & options): Node ("VarDisNode", options) {

            std::string name = this->get_name();
            const char* envDroneName = std::getenv("DRONE_NAME");
            std::stringstream ss;

            ss << "Initializing VarDis publishers... ";
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());

            registerProtocolPublisher_ = this->create_publisher<dcp_msgs::msg::RegisterProtocolRequest>(std::string(envDroneName) + "/beaconing/register/protocol", 1);
            transmistPayloadPublisher_ = this->create_publisher<dcp_msgs::msg::TransmitPayloadRequest>(std::string(envDroneName) + "/beaconing/payload/transmit", 10);
            checkQueueBufferPublisher_ = this->create_publisher<dcp_msgs::msg::QueryNumberBufferedPayloadsRequest>(std::string(envDroneName) + "/beaconing/queryBuffer", 10);

            ss.str(""); // Clear the contents of ss
            ss << "Initialized publishers... " << registerProtocolPublisher_->get_topic_name() << ", "
                                               << transmistPayloadPublisher_->get_topic_name() << ", "
                                               << checkQueueBufferPublisher_->get_topic_name();
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());

            controller_.reset(new VarDisClient(registerProtocolPublisher_, transmistPayloadPublisher_, checkQueueBufferPublisher_));

            ss.str(""); // Clear the contents of ss
            ss << "Initializing VarDis subscribers... ";
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());
            handleQueryBufferResponse_ = this->create_subscription<dcp_msgs::msg::QueryNumberBufferedPayloadsResponse>(
                    std::string(envDroneName) + "/vardis/queryBuffer",
                    1,
                    std::bind(&VarDisClient::queryBufferResponse, controller_, std::placeholders::_1));

            transmitIndicationSub_ = this->create_subscription<dcp_msgs::msg::PayloadTransmittedIndication>(
                    std::string(envDroneName) + "/transmit/payload/indication",
                    10,
                    std::bind(&VarDisClient::transmitted_payload_callback_indication, controller_, std::placeholders::_1));

            receiveIndicationSub_ = this->create_subscription<dcp_msgs::msg::ReceivePayloadIndication>(
                    std::string(envDroneName) + "/receive/payload/indication",
                    10,
                    std::bind(&VarDisClient::receive_payload_callback_indication, controller_, std::placeholders::_1));

            ss.str(""); // Clear the contents of ss
            ss << "Initialized VarDis subscribers... " << handleQueryBufferResponse_->get_topic_name() << ", "
                                                       << transmitIndicationSub_->get_topic_name() << ", "
                                                       << receiveIndicationSub_->get_topic_name();
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());

//            startupTimer_ = this->create_wall_timer(1200ms, std::bind(&VarDisClient::registerClientProtocol, controller_));
//            producerTimer_ = this->create_wall_timer(2000ms, std::bind(&VarDisClient::generateVariable, controller_));
//            updaterTimer_ = this->create_wall_timer(2700ms, std::bind(&VarDisClient::updateVariable, controller_));
//            deletionTimer_ = this->create_wall_timer(3400ms, std::bind(&VarDisClient::deleteVariable, controller_));
            checkIndicationTimer_ = this->create_wall_timer(VARDISPAR_BUFFER_CHECK_PERIOD, std::bind(&VarDisClient::checkNumberBufferedPayloads, controller_));
            writeResultsToFile_ = this->create_wall_timer(10000ms, std::bind(&VarDisClient::writeResultsToFile, controller_));

            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
            controller_->registerClientProtocol();
//            controller_->checkIndication();
//            controller_->generateVariable();
        }
    private:
        std::shared_ptr<VarDisClient> controller_;

        rclcpp::Publisher<dcp_msgs::msg::RegisterProtocolRequest>::SharedPtr registerProtocolPublisher_;
        rclcpp::Publisher<dcp_msgs::msg::TransmitPayloadRequest>::SharedPtr transmistPayloadPublisher_;
        rclcpp::Publisher<dcp_msgs::msg::QueryNumberBufferedPayloadsRequest>::SharedPtr checkQueueBufferPublisher_;
        rclcpp::Subscription<dcp_msgs::msg::QueryNumberBufferedPayloadsResponse>::SharedPtr handleQueryBufferResponse_;
        rclcpp::Subscription<dcp_msgs::msg::PayloadTransmittedIndication>::SharedPtr transmitIndicationSub_;
        rclcpp::Subscription<dcp_msgs::msg::ReceivePayloadIndication>::SharedPtr receiveIndicationSub_;
//        rclcpp::TimerBase::SharedPtr startupTimer_;
        rclcpp::TimerBase::SharedPtr producerTimer_;
        rclcpp::TimerBase::SharedPtr updaterTimer_;
        rclcpp::TimerBase::SharedPtr deletionTimer_;
        rclcpp::TimerBase::SharedPtr checkIndicationTimer_;
        rclcpp::TimerBase::SharedPtr writeResultsToFile_;

    };
};
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dcp::VarDisClientNode);
