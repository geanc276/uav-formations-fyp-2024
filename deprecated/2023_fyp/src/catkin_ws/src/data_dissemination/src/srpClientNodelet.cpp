#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include "headers/srpClient.h"
#include <pluginlib/class_list_macros.h>
#include "data_dissemination_msgs/RegisterProtocolRequest.h"
#include "data_dissemination_msgs/TransmitPayloadRequest.h"

namespace data_dissemination
{

/*
    Nodelet wrapping class for State Reporting Protocol
*/
class SRPClientNodelet : public nodelet::Nodelet
{
public:
    SRPClientNodelet() {}

    ~SRPClientNodelet() {}

    /*
        Initializes the State Reporting Protocol nodelet
    */
    virtual void onInit()
    {
        // Get ROS node handler
        ros::NodeHandle nh = this->getMTPrivateNodeHandle();

        // Get ROS node name
        std::string name = nh.getUnresolvedNamespace();
        int pos = name.find_last_of('/');
        name = name.substr(pos+1);

        // Setup ROS Publishers
        NODELET_INFO_STREAM("Initializing publishers... " << name);
        beaconingClientSender = nh.advertise<data_dissemination_msgs::RegisterProtocolRequest>("/beaconing/register/protocol", 1);
        beaconingPayloadTransmitter = nh.advertise<data_dissemination_msgs::TransmitPayloadRequest>("/beaconing/payload/transmit", 10);
        NODELET_INFO_STREAM("Initialized publishers... " << beaconingClientSender.getTopic() << ", " << beaconingPayloadTransmitter.getTopic());

        // Setup State Reporting Protocol
        NODELET_INFO_STREAM("Initializing nodelet... " << name);
        controller_.reset(new SRPClient(beaconingClientSender, beaconingPayloadTransmitter));
        NODELET_INFO_STREAM("Initialized nodelet... " << name);


        // Setup ROS Subscribers
        // Needs to listen to this drones mavros output for beaconing broadcasting.
        // Get drone name here somehow or create subscriber in SRP
        stateDataSub = nh.subscribe("/drone_name/mavros/global_position/global", 10, &SRPClient::callback_position, controller_);

        bpIndicationSub = nh.subscribe("/srp/payload/indication", 10, &SRPClient::callback_indications, controller_);
        NODELET_INFO_STREAM("Initialized subscribers... " << name);

        // Init and run nodelet
        NODELET_INFO_STREAM("Registering SRP.");
        startupTimer = nh.createTimer(ros::Duration(1.0), boost::bind(&SRPClient::registerClientProtocol, controller_, _1), true, true);

    }

private:
    // Shared pointer reference to underlying state reporting protocol class
    boost::shared_ptr<SRPClient> controller_;
    // ROS Subscriber to be used for collecting drone state data from mavros
    ros::Subscriber stateDataSub;
    // ROS Subscriber to listen for payload transmitted indications from the Beaconing Protocol
    ros::Subscriber bpIndicationSub;
    // ROS Publisher for registering as a client protocol on the Beaconing Protocol
    ros::Publisher beaconingClientSender;
    // ROS Publisher for sending payloads for transmission to the Beaconing Protocol
    ros::Publisher beaconingPayloadTransmitter;
    // ROS Timer to delay SRP registering with the Beaconing Protocol by 1 second
    ros::Timer startupTimer;
};
}; // namespace

PLUGINLIB_EXPORT_CLASS(data_dissemination::SRPClientNodelet, nodelet::Nodelet);