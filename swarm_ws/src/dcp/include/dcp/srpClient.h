#ifndef SRP_CLIENT_H
#define SRP_CLIENT_H

#include <iostream>
#include "srpDataTypes.h"
#include "beaconingClient.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "netinet/ether.h"
#include "dcp_msgs/msg/receive_payload_indication.hpp"
#include "Helpers.h"

#define SRP_STATUS_OK 200

/*
    State Reporting protocol class definition
*/

class SRPClient {
private:
    // Beaconing protocol ID
    BPProtocolId id;
    // Helper functions to serialize and deserialize payloads.
    Helpers helpers = Helpers();
    // Local node identification
    NodeIdentifier nodeId;
    // Local State Reporting protocol packet number
    SRPSequenceNumber currentSequenceNumber;
    // Reference to neighbour table for local drone
    NeighbourTable *neighbours;
    // Holds value of local drones current state data (Not implemented)
    std::string positionData; // Where the rclcpp topic data is read too.
    // Publisher for registering this State Reporting protocol 
    rclcpp::Publisher<dcp_msgs::msg::RegisterProtocolRequest>::SharedPtr protocolRegisterPub;
    // Publisher for sending data to Beaconing protocol for network transmission
    rclcpp::Publisher<dcp_msgs::msg::TransmitPayloadRequest>::SharedPtr payloadTransmitPub;
public:
    SRPClient(rclcpp::Publisher<dcp_msgs::msg::RegisterProtocolRequest>::SharedPtr pubRegister, rclcpp::Publisher<dcp_msgs::msg::TransmitPayloadRequest>::SharedPtr pubTransmit);

    void registerClientProtocol();

    int transmitSafetyData(SafetyData *request);

    void callback_position(const std::shared_ptr<const std_msgs::msg::String>& msg);

    void callback_indications(const dcp_msgs::msg::ReceivePayloadIndication& indic);

    void checkEntriesToDrop();
};
 
#endif // SRP_CLIENT_H
