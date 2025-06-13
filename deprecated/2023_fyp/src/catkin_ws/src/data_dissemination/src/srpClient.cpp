#include <ros/ros.h>
#include "headers/srpClient.h"
#include "data_dissemination_msgs/RegisterProtocolRequest.h"
#include "data_dissemination_msgs/TransmitPayloadRequest.h"
#include "data_dissemination_msgs/ReceivePayloadIndication.h"

/*
    State Reporting Protocol constructor.
    @param pubRegister: ROS Publisher used to register the State Reporting Protocol with the Beaconing Protocol
    @param pubTransmit: ROS Publisher used for transmitting safety data to the Beaconing Protocol
*/
SRPClient::SRPClient(ros::Publisher pubRegister, ros::Publisher pubTransmit)
{
    protocolRegisterPub = pubRegister;
    payloadTransmitPub = pubTransmit;

    id.identifier = BP_PROTID_SRP;
    currentSequenceNumber.sequenceNumber = 0;


    // Create socket
    int beaconingSocket = socket(PF_INET, SOCK_DGRAM, 0);
    if (beaconingSocket == -1)
    {
        ROS_INFO_STREAM("Error in socket creation: " << strerror(errno));
    }

    // Get if address name
    struct ifaddrs *ifap, *ifa;
    char ifname[10];
    getifaddrs(&ifap);
    for (ifa = ifap; ifa; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr && ifa->ifa_flags & IFF_BROADCAST && ifa->ifa_addr->sa_family == AF_INET) {
            strncpy(ifname, ifa->ifa_name, sizeof(ifname));
            break;
        }
    }
    // Set interface name and clean up
    const char *if_name = ifname;
    ROS_INFO_STREAM(if_name);
    freeifaddrs(ifap);

    struct ifreq ifr;
    memset(&ifr,0,sizeof(ifr));
    strncpy(ifr.ifr_name,if_name,sizeof(ifr.ifr_name));
    
    // Get interface MAC address
    int err = ioctl(beaconingSocket,SIOCGIFHWADDR,&ifr);
    if (err < 0)
        ROS_INFO_STREAM("Something broke at HW ioctl");
    
    // Set the collected MAC address
    char addr[18];
    const char *hwAddress = addr;
    memset(addr, 0, 18);
    strncpy(addr, ether_ntoa( (ether_addr *) ifr.ifr_hwaddr.sa_data), 18);

    // Clean up
    close(beaconingSocket);

    NodeIdentifier node;
    strncpy(node.identifier, addr, 18); // Local HW address
    nodeId = node;

    ROS_INFO_STREAM(node.identifier);

    // Create Neighbour Table
    neighbours = new NeighbourTable();
}

/*
    Sends register message to Beaconing Protocol
    @param event: ROS Timer event to allow Beaconing Protocol to initialize
*/
void SRPClient::registerClientProtocol(const ros::TimerEvent& event)
{
    // Create register request message.
    data_dissemination_msgs::RegisterProtocolRequest req;
    req.id = id.identifier;
    req.name = "SRP - State Reporting Protocol V1.0";
    req.length = sizeof(ExtendedSafetyData);
    req.mode = BP_QMODE_REPEAT;

    // Send register request to BP.
    ROS_INFO_STREAM("Sending SRP register request");
    protocolRegisterPub.publish(req);
}

/*
    Sends transmit message with safety data to Beaconing Protocol
    @param request: Safety Data structure to be transmitted to Beaconing Protocol for sending
*/
int SRPClient::transmitSafetyData(SafetyData *request)
{
    // Wrap Safety Data with header structure
    ExtendedSafetyData exsData;
    exsData.sData = *request;

    // Populate header fields
    strcpy(exsData.nodeId.identifier, nodeId.identifier);
    TimeStamp time;
    time.time = std::chrono::system_clock::now().time_since_epoch();
    exsData.tStamp = time;
    exsData.seqNum = currentSequenceNumber;

    // Increment sequence number
    currentSequenceNumber.sequenceNumber++ % (UINT32_MAX + 1);

    // Prepare BP-TransmitPayload request
    uint8_t* ptr = reinterpret_cast<uint8_t*>(&exsData);
    std::vector<uint8_t> payload(ptr, ptr+sizeof(exsData));

    // Populate ROS message for sending
    data_dissemination_msgs::TransmitPayloadRequest txReq;
    txReq.id = id.identifier;
    txReq.length = sizeof(exsData);
    txReq.payload = payload;

    // Submit request to BP
    payloadTransmitPub.publish(txReq);

    return SRP_STATUS_OK;
}

/*
    ROS callback method for collecting safety data from flight controller/control loop
    NOTE: Currently only uses dummy data, needs to be updated to collect local drone flight data
    @param msg: Shared pointer to safety data for transmission
*/
void SRPClient::callback_position(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO_STREAM("State data callback running.");
    // Handle reading data from control loop.

    // Read position data and submit transmit request
    positionData = msg->data;

    // All the following should be read from ROS topics in implementation
    // Collect from positionData.
    SafetyData sData;
    // Add position vector values
    sData.position[0] = 10; // x value
    sData.position[1] = 10; // y value
    sData.position[2] = 10; // z value

    // Add rotation vector values
    sData.rotation[0] = 100;

    // Add direction vector values

    // Add velocity value

    transmitSafetyData(&sData);
}

/*
    ROS callback for received payload indications. Passes received data to the Neighbour Table for storage
    @param indic: ROS received payload indication message containing neighbour data for storage
*/
void SRPClient::callback_indications(const data_dissemination_msgs::ReceivePayloadIndication& indic)
{
    // Collect payload data from indication message
    std::vector<unsigned char> data = indic.payload;
    std::string droneName = indic.id;

    // Pass data to neighbour table
    ExtendedSafetyData sd;
    memcpy(&sd, data.data(), sizeof(ExtendedSafetyData));

    neighbours->ntAddEntry(sd, droneName);

}