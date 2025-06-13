#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include "headers/beaconingClient.h"
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>

#include "data_dissemination_msgs/ReceivePayloadIndication.h"

namespace data_dissemination
{

/*
    Nodelet wrapping class for Beaconing Protocol
*/
class BeaconingClientNodelet : public nodelet::Nodelet
{
public:
    BeaconingClientNodelet() {}

    ~BeaconingClientNodelet() {}

    /*
        Initializes the Beaconing Protocol nodelet
    */
    virtual void onInit()
    {
        // Get ROS node handler
        ros::NodeHandle nh = this->getMTPrivateNodeHandle();

        // Get ROS node name
        std::string name = nh.getUnresolvedNamespace();
        int pos = name.find_last_of('/');
        name = name.substr(pos+1);

        // Setup ROS publishers
        NODELET_INFO_STREAM("Initializing nodelet publishers... ");
        srpClientSender = nh.advertise<data_dissemination_msgs::ReceivePayloadIndication>("/srp/payload/indication", 10);
        NODELET_INFO_STREAM("Initialized publisher... " << srpClientSender.getTopic());

        // Setup Beaconing Protocol
        NODELET_INFO_STREAM("Initializing nodelet... " << name);
        controller_.reset(new BeaconingClient(srpClientSender));

        // Testing
        beaconingClientSender = nh.advertise<std_msgs::String>("/drone_name/mavros/global_position/global", 10);

        // Setup ROS subscribers
        NODELET_INFO_STREAM("Initializing " << name << " subscribers");
        protocolRegisterSub = nh.subscribe("/beaconing/register/protocol", 1, &BeaconingClient::registerProtocolCallback, controller_);
        protocolTransmitPayloadSub = nh.subscribe("/beaconing/payload/transmit", 10, &BeaconingClient::transmitPayloadCallback, controller_);

        // Setup main loop timer
        NODELET_INFO_STREAM("Initializing " << name << " timers");
        testingTimer = nh.createTimer(ros::Duration(3.0), boost::bind(&BeaconingClientNodelet::testingCb, this, _1));
        // ROS duration is beaconing period
        beaconingProcessTransmit = nh.createTimer(ros::Duration(1.0/BP_BEACON_FREQUENCY), boost::bind(&BeaconingClient::runBeaconingTransmit, controller_, _1));
        beaconingProcessReceive = nh.createTimer(ros::Duration(1.0/(BP_BEACON_FREQUENCY * BP_MAX_DRONES)), boost::bind(&BeaconingClient::runBeaconingReceive, controller_, _1));
    }

    /*
        ROS callback method to send dummy safety data
    */
    void testingCb(const ros::TimerEvent& event)
    {
        NODELET_INFO_STREAM("Publishing safety data.");
        std_msgs::String msg;
        msg.data = "Some data or something";
        beaconingClientSender.publish(msg);

    }
private:
    // Shared pointer reference to underlying beaconing protocol class
    boost::shared_ptr<BeaconingClient> controller_;
    // Sets beaconing transmit frequency
    ros::Timer beaconingProcessTransmit;
    // Sets beaconing receive frequency
    ros::Timer beaconingProcessReceive;
    // Timer for calling test safety data callback
    ros::Timer testingTimer;
    // ROS Subscriber for client protocol register messages
    ros::Subscriber protocolRegisterSub;
    // ROS Subscriber for client protocol transmit messages
    ros::Subscriber protocolTransmitPayloadSub;
    // ROS Publisher for sending dummy safety data message
    ros::Publisher beaconingClientSender;
    // ROS Publisher for sending payload transmitted indications to SRP
    ros::Publisher srpClientSender;
};
}; // namespace

PLUGINLIB_EXPORT_CLASS(data_dissemination::BeaconingClientNodelet, nodelet::Nodelet);