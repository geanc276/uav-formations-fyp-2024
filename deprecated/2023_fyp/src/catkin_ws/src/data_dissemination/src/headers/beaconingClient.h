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
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
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

#include "beaconingDataTypes.h"
#include "data_dissemination_msgs/RegisterProtocolRequest.h"
#include "data_dissemination_msgs/TransmitPayloadRequest.h"

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
    // Maximum packet size
    int BPPAR_MAXIMUM_PACKET_SIZE;
    // Current packet number
    int packetNumber;
    // Socket for packet transmission
    int beaconingSocket;
    // ROS Publisher for IPC with SRP 
    ros::Publisher srpClientIndicator;
    // Network interface index number
    int ifindex;
    // Network interface MAC address
    char localHWaddress[14];
public:
    BeaconingClient(ros::Publisher srpClientSender);
    bool lProtocolExists(BPProtocolId *id);
    void lRemoveProtocol(BPProtocolId id);
    BPClientProtocol* lLookupProtocol(BPProtocolId id);
    void registerProtocolCallback(const data_dissemination_msgs::RegisterProtocolRequest& request);
    int registerProtocol(const data_dissemination_msgs::RegisterProtocolRequest& request);
    void transmitPayloadCallback(const data_dissemination_msgs::TransmitPayloadRequest& msg);
    int transmitPayload(const data_dissemination_msgs::TransmitPayloadRequest& request);
    void transmitBeacon(int socketFd);
    void receiveBeacon();
    void initBeaconingProtocol(ros::Publisher srpClientSender);
    //void runBeaconingProtocol(const ros::TimerEvent& event);
    void runBeaconingTransmit(const ros::TimerEvent& event);
    void runBeaconingReceive(const ros::TimerEvent& event);
};

#endif // BEACONINGCLIENT_H