#include "dcp/srpClient.h"
#include "dcp_msgs/msg/register_protocol_request.hpp"
#include "dcp_msgs/msg/transmit_payload_request.hpp"
#include "dcp_msgs/msg/receive_payload_indication.hpp"

/*
    State Reporting Protocol constructor.
    @param pubRegister: ROS Publisher used to register the State Reporting Protocol with the Beaconing Protocol
    @param pubTransmit: ROS Publisher used for transmitting safety data to the Beaconing Protocol
*/
SRPClient::SRPClient(rclcpp::Publisher<dcp_msgs::msg::RegisterProtocolRequest>::SharedPtr pubRegister, rclcpp::Publisher<dcp_msgs::msg::TransmitPayloadRequest>::SharedPtr pubTransmit)
{
    protocolRegisterPub = pubRegister;
    payloadTransmitPub = pubTransmit;
    std::stringstream ss;

    id.identifier = BP_PROTID_SRP;
    currentSequenceNumber.sequenceNumber = 0;

    nodeId = helpers.getNodeIdentifier();

    ss.str(""); // Clear the contents of ss
    ss << nodeId;
    RCLCPP_INFO(rclcpp::get_logger("logger"), ss.str().c_str());

    // Create Neighbour Table
    neighbours = new NeighbourTable();
}

/*
    Sends register message to Beaconing Protocol
    @param event: ROS Timer event to allow Beaconing Protocol to initialize
*/
void SRPClient::registerClientProtocol()
{
    // Create register request message.
    dcp_msgs::msg::RegisterProtocolRequest req;
    req.id = id.identifier;
    req.name = "SRP - State Reporting Protocol V1.0";
    req.length = sizeof(ExtendedSafetyData);
    req.mode = BP_QMODE_REPEAT;

    // Send register request to BP.
    RCLCPP_INFO(rclcpp::get_logger("logger"), "Sending SRP register request");
    protocolRegisterPub->publish(req);
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
    exsData.nodeId = nodeId;
    TimeStamp time;
    exsData.tStamp = time;
    exsData.seqNum = currentSequenceNumber;

    if (strcmp(exsData.nodeId.c_str(), nodeId.c_str())) {
        return BP_STATUS_OK;
    }

    RCLCPP_INFO(rclcpp::get_logger("logger"), "ExtendedSafetyData at SRPClient::transmitSafetyData: %s", exsData.toString().c_str());

    // Increment sequence number
    currentSequenceNumber.sequenceNumber++;

    // Prepare BP-TransmitPayload request
    uint8_t* ptr = reinterpret_cast<uint8_t*>(&exsData);
    std::vector<uint8_t> payload(ptr, ptr+sizeof(exsData));

    // Populate ROS message for sending
    dcp_msgs::msg::TransmitPayloadRequest txReq;
    txReq.id = id.identifier;
    txReq.length = sizeof(exsData);

    std::vector<uint8_t> buffer;
    helpers.serializeExtendedSafetyData(buffer, exsData);
    txReq.payload = buffer;

    // Submit request to BP
    payloadTransmitPub->publish(txReq);

    return SRP_STATUS_OK;
}

/*
    ROS callback method for collecting safety data from flight controller/control loop
    NOTE: Currently only uses dummy data, needs to be updated to collect local drone flight data
    @param msg: Shared pointer to safety data for transmission
*/
void SRPClient::callback_position(const std::shared_ptr<const std_msgs::msg::String> &msg)
{
    RCLCPP_INFO(rclcpp::get_logger("logger"), "Collecting the below safety data at: SRPClient::callback_position");
    std::cout << msg->data << std::endl;
    // Handle reading data from control loop.

    // Read position data and submit transmit request
    positionData = msg->data;

    // All the following should be read from ROS topics in implementation
    // Collect from positionData.
    SafetyData sData;
    // Add position vector values
    sData.position[0] = 10; // x value
    sData.position[1] = 11; // y value
    sData.position[2] = 12; // z value

    // Add direction vector values
    sData.direction[0] = 1000;
    sData.direction[1] = 2000;
    sData.direction[2] = 3000;

    // Add rotation vector values
    sData.rotation[0] = 100;
    sData.rotation[1] = 200;
    sData.rotation[2] = 300;

    // Add velocity value
    sData.speed = 5;

    transmitSafetyData(&sData);
}

/*
    ROS callback for received payload indications. Passes received data to the Neighbour Table for storage
    @param indic: ROS received payload indication message containing neighbour data for storage
*/
void SRPClient::callback_indications(const dcp_msgs::msg::ReceivePayloadIndication& indic)
{
    // Collect payload data from indication message
    RCLCPP_INFO(rclcpp::get_logger("logger"), "ReceivePayloadIndication at SRPClient::callback_indications");
    std::vector<uint8_t> data = indic.payload;
    std::string droneName = indic.id;
    uint8_t protocolId = indic.protid;

    if (protocolId == BP_PROTID_SRP) {
        size_t offset = 0;
        ExtendedSafetyData sd = helpers.deserializeExtendedSafetyData(data,offset);
        if (strcmp(sd.nodeId.c_str(), nodeId.c_str()) == 0) {
            RCLCPP_INFO(rclcpp::get_logger("logger"), "ESD at SRPClient::callback_indications: %s.", sd.toString().c_str());
            // Pass data to neighbour table
            neighbours->ntAddEntry(sd, droneName);
        }
    }
}

void SRPClient::checkEntriesToDrop() {
    TimeStamp currentTime;
    Shared::vector<Shared::NeighbourTableEntry> allNodes = neighbours->ntListAllNodes();
    for (Shared::NeighbourTableEntry node: allNodes) {
        if ((currentTime.time.count() - node.receptionTime.time.count()) > SRPPAR_NEIGHBOUR_TABLE_TIMEOUT) {
            neighbours->ntRemoveEntry(node.id);
        }
    }
}