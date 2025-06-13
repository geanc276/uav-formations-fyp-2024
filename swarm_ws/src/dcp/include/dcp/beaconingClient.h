#ifndef BEACONINGCLIENT_H
#define BEACONINGCLIENT_H

#include <vector>
#include <chrono>
#include <queue>
#include <string>
#include <iostream>
#include <ctime>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include "rclcpp/rclcpp.hpp"
#include <list>
#include <algorithm>

#include <errno.h>
#include <arpa/inet.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <netinet/if_ether.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <netpacket/packet.h>
#include <sys/ioctl.h>
#include <ifaddrs.h>

#include "std_msgs/msg/string.hpp"

#include "beaconingDataTypes.h"
#include "dcp_msgs/msg/register_protocol_request.hpp"
#include "dcp_msgs/msg/transmit_payload_request.hpp"
#include "dcp_msgs/msg/receive_payload_indication.hpp"
#include "dcp_msgs/msg/payload_transmitted_indication.hpp"
#include "dcp_msgs/msg/query_number_buffered_payloads_request.hpp"
#include "dcp_msgs/msg/query_number_buffered_payloads_response.hpp"

#include <fstream>

#define BP_BEACON_FREQUENCY 10.0
#define BP_MAX_DRONES 10

#define BP_STATUS_OK 200
#define BP_STATUS_PROTOCOL_ALREADY_REGISTERED 202
#define BP_STATUS_ILLEGAL_MAX_PAYLOAD_SIZE 204
#define BP_STATUS_UNKNOWN_PROTOCOL 208
#define BP_STATUS_PAYLOAD_TOO_LARGE 210
#define BP_STATUS_EMPTY_PAYLOAD 212

/*
    Beaconing Protocol class definition
*/

class BeaconingClient {
private:
    // All registered local protocols
    std::vector<BPClientProtocol*> currentClientProtocols;
    // Used to create separate processes for each received packet
    std::thread receivingThread;
    // Used to track information about the performance (results found in 2024 report)
    std::unordered_map<std::string, int> bcResults;
    // Drone Name set with environment variable
    const char* envDroneName;
    // Maximum packet size
    int BPPAR_MAXIMUM_PACKET_SIZE;
    // Socket for packet transmission
    int receiveSocket;
    // ROS Publisher for IPC with SRP
    rclcpp::Publisher<dcp_msgs::msg::ReceivePayloadIndication>::SharedPtr srpClientIndicator;
    // ROS Publisher for sending output of queryNumberBufferedPayloads function
    rclcpp::Publisher<dcp_msgs::msg::QueryNumberBufferedPayloadsResponse>::SharedPtr queryBufferResponsePublisher;
    // ROS Publisher to notify other protocols that their payload has been transmitted
    rclcpp::Publisher<dcp_msgs::msg::PayloadTransmittedIndication>::SharedPtr transmittedPayloadIndicator;
    // Network interface index number
    int ifindex;
    // Network interface MAC address
    char localHWaddress[14];
public:
    BeaconingClient(rclcpp::Publisher<dcp_msgs::msg::QueryNumberBufferedPayloadsResponse>::SharedPtr,
                    rclcpp::Publisher<dcp_msgs::msg::ReceivePayloadIndication>::SharedPtr,
                    rclcpp::Publisher<dcp_msgs::msg::PayloadTransmittedIndication>::SharedPtr);
    bool lProtocolExists(BPProtocolId *id);
    void lRemoveProtocol(BPProtocolId id);
    BPClientProtocol* lLookupProtocol(BPProtocolId id);
    void queryNumberBufferedPayloads(const dcp_msgs::msg::QueryNumberBufferedPayloadsRequest& request);
    void registerProtocolCallback(const dcp_msgs::msg::RegisterProtocolRequest& request);
    int registerProtocol(const dcp_msgs::msg::RegisterProtocolRequest& request);
    void transmitPayloadCallback(const dcp_msgs::msg::TransmitPayloadRequest& msg);
    int transmitPayload(const dcp_msgs::msg::TransmitPayloadRequest& request);
    void transmitBeacon(int socketFd);
    void receiveBeacon();
    void initBeaconingProtocol();
    void runBeaconingTransmit();
    void runBeaconingReceive();
    void broadcastPackets();
    void receivePackets();
    void processPacket(const uint8_t* buffer, size_t recvLen, const sockaddr_in* clientAddr, const TimeStamp& timestamp);
    void writeResultsToFile();
};

#endif // BEACONINGCLIENT_H