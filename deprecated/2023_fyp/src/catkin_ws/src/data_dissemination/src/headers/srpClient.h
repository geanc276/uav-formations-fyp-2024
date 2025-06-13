#ifndef SRP_CLIENT_H
#define SRP_CLIENT_H

#include "srpDataTypes.h"
#include "beaconingClient.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "netinet/ether.h"
#include "data_dissemination_msgs/ReceivePayloadIndication.h"

#define SRP_STATUS_OK 200

/*
    State Reporting protocol class definition
*/

class SRPClient {
private:
    // Beaconing protocol ID
    BPProtocolId id;
    // Local node identification
    NodeIdentifier nodeId;
    // Local State Reporting protocol packet number
    SRPSequenceNumber currentSequenceNumber;
    // Reference to neighbour table for local drone
    NeighbourTable *neighbours;
    // Holds value of local drones current state data (Not implemented)
    std::string positionData; // Where the ROS topic data is read too.
    // Publisher for registering this State Reporting protocol 
    ros::Publisher protocolRegisterPub;
    // Publisher for sending data to Beaconing protocol for network transmission
    ros::Publisher payloadTransmitPub;
public:
    SRPClient(ros::Publisher pubRegister, ros::Publisher pubTransmit);

    void registerClientProtocol(const ros::TimerEvent& event);

    int transmitSafetyData(SafetyData *request);

    void callback_position(const std_msgs::String::ConstPtr &msg);

    void callback_indications(const data_dissemination_msgs::ReceivePayloadIndication& indic);
};
 
#endif // SRP_CLIENT_H
