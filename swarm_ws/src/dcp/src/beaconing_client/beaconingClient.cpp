#include "dcp/beaconingClient.h"
#include "dcp/srpClient.h"

/**
 * @brief Constructs a BeaconingClient with the specified publishers.
 *
 * This constructor initializes the BeaconingClient with publishers to manage query responses,
 * payload reception indications, and payload transmission indications. It also sets up the
 * beaconing protocol and retrieves the drone's name from the environment.
 *
 * @param checkQueueBuffer Publisher for QueryNumberBufferedPayloadsResponse messages.
 * @param srpClientSender Publisher for ReceivePayloadIndication messages.
 * @param payloadTransmittedIndication Publisher for PayloadTransmittedIndication messages.
 */
BeaconingClient::BeaconingClient(rclcpp::Publisher<dcp_msgs::msg::QueryNumberBufferedPayloadsResponse>::SharedPtr checkQueueBuffer,
                                 rclcpp::Publisher<dcp_msgs::msg::ReceivePayloadIndication>::SharedPtr srpClientSender,
                                 rclcpp::Publisher<dcp_msgs::msg::PayloadTransmittedIndication>::SharedPtr payloadTransmittedIndication)
{
    queryBufferResponsePublisher = checkQueueBuffer;
    srpClientIndicator = srpClientSender;
    transmittedPayloadIndicator = payloadTransmittedIndication;
    // Call initializing method
    initBeaconingProtocol();
    envDroneName = std::getenv("DRONE_NAME");
}

/**
 * @brief Checks if a protocol is registered.
 *
 * @param id Pointer to the client protocol identifier (BPProtocolId).
 * @return True if the protocol exists, false otherwise.
 */
bool BeaconingClient::lProtocolExists(BPProtocolId *id)
{
    // For each registered protocol check if identifier matches supplied ID
    for (std::vector<BPClientProtocol*>::size_type i = 0; i < currentClientProtocols.size(); i++)
    {
        if (currentClientProtocols[i]->protocolId.identifier == id->identifier)
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief Removes a protocol from the current client protocols.
 *
 * @param id Client protocol identifier (BPProtocolId).
 */
void BeaconingClient::lRemoveProtocol(BPProtocolId id)
{
    // For each registered protocol check if identifier matches
    std::vector<BPClientProtocol *>::iterator iter = currentClientProtocols.begin();
    for (std::vector<BPClientProtocol*>::size_type i = 0; i < currentClientProtocols.size(); i++)
    {
        // If identifier matches remove registered protocol
        if (currentClientProtocols[i]->protocolId.identifier == id.identifier)
        {
            currentClientProtocols.erase(iter);
        }
        next(iter);
    }
}

/**
 * @brief Looks up a protocol from the current client protocols.
 *
 * @param id Client protocol identifier (BPProtocolId).
 * @return The client protocol if found, otherwise returns a null value.
 */

BPClientProtocol* BeaconingClient::lLookupProtocol(BPProtocolId id)
{
    for (std::vector<BPClientProtocol*>::size_type i = 0; i < currentClientProtocols.size(); i++)
    {
        if (currentClientProtocols[i]->protocolId.identifier == id.identifier)
        {
            return currentClientProtocols[i];
        }
    }
    return NULL;
}

/**
 * @brief Handles a request to query the number of buffered payloads for a given protocol.
 *
 * @param request The request message containing the client protocol identifier (ID) to query.
 * @return The response is published using the `queryBufferResponsePublisher` to inform the client
 * of the current buffered payload count and status.
 */

void BeaconingClient::queryNumberBufferedPayloads(const dcp_msgs::msg::QueryNumberBufferedPayloadsRequest& request) {

    BPProtocolId id = {request.id};
    std::stringstream ss;
    ss << "Querying buffer for ID: " << id.identifier;
    RCLCPP_INFO(rclcpp::get_logger("logger"), ss.str().c_str());

    dcp_msgs::msg::QueryNumberBufferedPayloadsResponse response;
    if (not lLookupProtocol(id)) {
        response.status = BP_STATUS_UNKNOWN_PROTOCOL;
        response.number = -1;
    } else {
        BPClientProtocol *protEntry = lLookupProtocol(id);
        if (protEntry->queueMode.queueMode == BP_QMODE_QUEUE) {
            response.status = BP_STATUS_OK;
            response.number = protEntry->bufferQueue.size();
        } else if (protEntry->bufferOccupied) {
            response.status = BP_STATUS_OK;
            response.number = 1;
        } else {
            response.status = BP_STATUS_OK;
            response.number = 0;
        }
    }

    queryBufferResponsePublisher->publish(response);
}

/**
 * @brief Handles a register protocol request callback.
 *
 * @param request The received register protocol request message containing details of the
 * protocol to be registered.
 */
void BeaconingClient::registerProtocolCallback(const dcp_msgs::msg::RegisterProtocolRequest& request)
{
    int result = registerProtocol(request);
    // Process result value
    std::stringstream ss;
    ss << "SRP Client register returned: " << result << " FOR " << request.name;
    RCLCPP_INFO(rclcpp::get_logger("logger"), ss.str().c_str());
}

/**
 * @brief Registers a new client protocol with the Beaconing protocol.
 *
 * @param request The received register protocol request containing the protocol details.
 * @return An integer representing the registration status code:
 *         - `BP_STATUS_OK` for successful registration.
 *         - Other error codes if the registration fails (e.g., duplicate protocol, invalid ID).
 */
int BeaconingClient::registerProtocol(const dcp_msgs::msg::RegisterProtocolRequest& request)
{
    // Unpack request values
    BPProtocolId id = {request.id};
    BPLength length;
    length.length = request.length;
    BPQueueingMode mode;
    mode.queueMode = request.mode;

    std::stringstream ss;
    ss << "Attempting to register protocol: " << request.id << " FOR " << request.name;
    RCLCPP_INFO(rclcpp::get_logger("logger"), ss.str().c_str());

    // Performs at least these actions (potentially more)
    if (lProtocolExists(&id))
    {
        return BP_STATUS_PROTOCOL_ALREADY_REGISTERED;
    }

    if (length.length > static_cast<decltype(BPPAR_MAXIMUM_PACKET_SIZE)>((BPPAR_MAXIMUM_PACKET_SIZE - (sizeof(BPPayloadBlockHeader) + 4))))
    {
        return BP_STATUS_ILLEGAL_MAX_PAYLOAD_SIZE;
    }
    // Create new client protocol
    BPClientProtocol *newent = new BPClientProtocol;

    // Create protocol fields
    TimeStamp *time = new TimeStamp;
    auto t = std::chrono::system_clock::now().time_since_epoch();
    BPBufferEntry *buffer = new BPBufferEntry;
    buffer->length.length = 0;
    buffer->payload = (uint8_t *)malloc(length.length);

    std::queue<BPBufferEntry *> buffQueue;

    // Populate client protocol structure fields
    newent->protocolId = id;
    newent->protocolName = request.name;
    newent->maxPayloadSize = length;
    newent->queueMode = mode;
    newent->timestamp = time;
    newent->bufferQueue = buffQueue;
    newent->bufferOccupied = false;
    newent->bufferEntry = buffer;

    // Add registered protocol 
    currentClientProtocols.push_back(newent);
    return BP_STATUS_OK;
}

/**
 * @brief Handles the transmit payload request callback.
 *
 * Pretty much only used to print the status code of transmitPayload
 *
 * @param msg Received transmit payload request containing the payload data and parameters.
 *
 * @return void
 */
void BeaconingClient::transmitPayloadCallback(const dcp_msgs::msg::TransmitPayloadRequest& msg)
{
    std::stringstream ss;
    RCLCPP_INFO(rclcpp::get_logger("logger"), "BeaconingClient::transmitPayloadCallback entered");
    ss << "transmitPayload returned: " << transmitPayload(msg) << " at: BeaconingClient::transmitPayloadCallback";
    RCLCPP_INFO(rclcpp::get_logger("logger"), ss.str().c_str());
}

/**
 * @brief Prepares the requested payload for transmission and adds it to the queue or buffer.
 *
 * @param request Received transmit payload request containing necessary data.
 * @return Transmit payload status code indicating the result of the operation.
 */
int BeaconingClient::transmitPayload(const dcp_msgs::msg::TransmitPayloadRequest& request)
{
    // Create from msg
    BPProtocolId id = {request.id};
    BPLength length = {request.length};
    std::stringstream ss;
    ss << "ID.id: " << request.id << ", Length: " << length.length << " at: BeaconingClient::transmitPayload";
    RCLCPP_INFO(rclcpp::get_logger("logger"), ss.str().c_str());

//    std::ostringstream payloadStream;
//    payloadStream << "Payload at BeaconingClient::transmitPayload: 1 \n";
//    for (size_t i = 0; i < request.payload.size(); ++i) {
//        payloadStream << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(request.payload[i]) << " ";
//    }
//    RCLCPP_INFO(rclcpp::get_logger("logger"), "%s", payloadStream.str().c_str());

    std::vector<unsigned char> data = request.payload;
    uint8_t payload[length.length];
    memcpy(&payload, data.data(), data.size());

    // Check requesting client protocol
    if (not lProtocolExists(&id))
    {
        RCLCPP_INFO(rclcpp::get_logger("logger"), "Unknown protocol: %d", id.identifier);
        return BP_STATUS_UNKNOWN_PROTOCOL;
    }
    // Check payload size restrictions
    BPClientProtocol *prot = lLookupProtocol(id);
    if (length.length > prot->maxPayloadSize.length)
    {
        return BP_STATUS_PAYLOAD_TOO_LARGE;
    }
    // If payload can be queued
    if (prot->queueMode.queueMode == BP_QMODE_QUEUE)
    {
        if (length.length > 0)
        {
            // Create buffer entry
            BPBufferEntry *newent = new BPBufferEntry;
            newent->length = length;
            newent->payload = (uint8_t *)malloc(length.length);
            memcpy(newent->payload, &payload[0], length.length);

            // Queue buffer
            prot->bufferQueue.push(newent);
//            RCLCPP_INFO(rclcpp::get_logger("logger"), "Broadcasting packet at: BeaconingClient::transmitPayload");
//            broadcastPackets();
            return BP_STATUS_OK;
        }
        else
        {
            return BP_STATUS_EMPTY_PAYLOAD;
        }
    }
    // If payload cannot be queued
    if ((prot->queueMode.queueMode == BP_QMODE_ONCE) || (prot->queueMode.queueMode == BP_QMODE_REPEAT))
    {
        if (length.length > 0)
        {
            // Create buffer entry
            prot->bufferEntry->length = length;
            prot->bufferEntry->payload = (uint8_t *)malloc(length.length);
            memcpy(prot->bufferEntry->payload, &payload[0], length.length);
            prot->bufferOccupied = true;
//            RCLCPP_INFO(rclcpp::get_logger("logger"), "Broadcasting packet at: BeaconingClient::transmitPayload");
//            broadcastPackets();
        }
        else
        {
            // Clear buffer
            prot->bufferOccupied = false;
            prot->bufferEntry->length.length = 0;
            prot->bufferEntry->payload = NULL;
        }
        return BP_STATUS_OK;
    }
}

// Generic checksum (https://stackoverflow.com/questions/70600663/how-do-i-send-to-an-ip-address-with-pf-packet)
unsigned short csum(unsigned short *ptr,int nbytes) 
{
    long sum;
    unsigned short oddbyte;
    short answer;

    sum=0;
    while(nbytes>1) {
        sum+=*ptr++;
        nbytes-=2;
    }
    if(nbytes==1) {
        oddbyte=0;
        *((u_char*)&oddbyte)=*(u_char*)ptr;
        sum+=oddbyte;
    }

    sum = (sum>>16)+(sum & 0xffff);
    sum = sum + (sum>>16);
    answer=(short)~sum;
    
    return(answer);
}

void printHex(const uint8_t* data, size_t length)
{
    std::cout << "Data: ";
    for (size_t i = 0; i < length; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)data[i] << " ";
        if ((i + 1) % 16 == 0) { // Print a newline every 16 bytes for readability
            std::cout << std::endl;
        }
    }
    std::cout << std::dec << std::endl; // Reset to decimal format
}

/**
 * @brief Initializes the beaconing protocol.
 */
void BeaconingClient::initBeaconingProtocol()
{
    // Configure required values
    currentClientProtocols = {};

    // Retreive UWB max packet size
    BPPAR_MAXIMUM_PACKET_SIZE = 1024; // Read from config

    // Create socket
    receiveSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (receiveSocket == -1)
    {
        RCLCPP_INFO(rclcpp::get_logger("logger"), "Error in socket creation.");
        RCLCPP_INFO(rclcpp::get_logger("logger"), strerror(errno));
    }

    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY); // Listen on all network interfaces
    serverAddr.sin_port = htons(5950); // Port number

    int reuse = 1;
    if (setsockopt(receiveSocket, SOL_SOCKET, SO_REUSEADDR, (const void*)&reuse, sizeof(reuse)) < 0) {
        perror("setsockopt(SO_REUSEADDR) failed");
    }

    // Step 1: Increase the socket buffer size
    int bufferSize = BPPAR_MAXIMUM_PACKET_SIZE;
    if (setsockopt(receiveSocket, SOL_SOCKET, SO_RCVBUF, &bufferSize, sizeof(bufferSize)) < 0) {
        perror("setsockopt(SO_RCVBUF) failed");
    }

    int flags = fcntl(receiveSocket, F_GETFL, 0);
    fcntl(receiveSocket, F_SETFL, flags | O_NONBLOCK);

    // Bind socket descriptor to address
    if (bind(receiveSocket, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) != 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("logger"), "Error in socket bind.");
    }

    int broadcast = 1;
    if (setsockopt(receiveSocket, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0) {
        perror("setsockopt failed");
    }

    runBeaconingReceive();

}

/**
 * @brief Executes the main operations of the beaconing protocol.
 *
 * This method is called periodically by a ROS timer to perform beaconing actions,
 * specifically sending beacon packets to clients.
 *
 * @return void
 */
void BeaconingClient::runBeaconingTransmit()
{
    // Main periodic actions of beaconing protocol
    // Send Beacon
    broadcastPackets();
}

/**
 * @brief Starts a thread to receive packets continuously.
 *
 * This method launches a separate thread that continuously listens for incoming packets
 * until the ROS node is shut down. It utilizes the `receivePackets` method to handle
 * the reception of packets.
 *
 * @return void
 */
void BeaconingClient::runBeaconingReceive() {
    receivingThread = std::thread([this]() {
        while (rclcpp::ok()) {  // Continue receiving packets until the node is shut down
            receivePackets();
        }
    });
    receivingThread.detach(); // Detach the thread so it runs independently
}

/**
 * @brief Broadcasts packets to clients.
 *
 * This function constructs and sends broadcast packets containing payload data from registered
 * client protocols. It prepares payload blocks from the protocol buffers and sends them to the
 * broadcast address.
 *
 * @details
 * - It collects beacon payload data from the client protocol buffers and queues, creating
 *   `BPPayloadBlock` structures for each valid payload.
 * - The constructed payload blocks are combined into a single byte array for broadcasting.
 * - The function sends the assembled data to the broadcast address (255.255.255.255) on the
 *   specified port (5950).
 * - After sending, it logs the number of packets sent and publishes indications for each protocol
 *   that had a transmitted payload.
 *
 * @return void
 */
void BeaconingClient::broadcastPackets() {

    // Configure required values
    int index = 0;
    int bearerLen = BPPAR_MAXIMUM_PACKET_SIZE;
    BPDroneName droneName = {};

    // Ensure the number of characters copied does not exceed the size of droneName.name - 1 (for the null terminator)
    size_t len = std::min(strlen(envDroneName), sizeof(droneName.name) - 1);
    for (size_t i = 0; i < len; i++) {
        droneName.name[i] = envDroneName[i];
    }
    // Null-terminate the string
    droneName.name[len] = '\0';

    std::vector<BPPayloadBlock> blockList;
    std::set <uint8_t> protocolIndicationIDs;
    // Collect beacon payload data from client protocol buffers and queues
    for (int i = 0; i < currentClientProtocols.size(); i++) {
        if (currentClientProtocols[i]->queueMode.queueMode == BP_QMODE_QUEUE) {
            if (!currentClientProtocols[i]->bufferQueue.empty()) {
                while (!currentClientProtocols[i]->bufferQueue.empty() &&
                       currentClientProtocols[i]->bufferQueue.front()->length.length + index <= bearerLen) {
                    // Create payload block
                    BPPayloadBlock block;
                    BPPayloadBlockHeader header;
                    header.protocolId = currentClientProtocols[i]->protocolId;
                    header.length = currentClientProtocols[i]->bufferQueue.front()->length;
                    header.droneName = droneName;
                    block.header = header;

                    block.header.length.length = htonl(block.header.length.length);

                    // Copy payload data and clean up
                    memcpy(block.payload, currentClientProtocols[i]->bufferQueue.front()->payload, header.length.length);

                    free(currentClientProtocols[i]->bufferQueue.front()->payload);
                    currentClientProtocols[i]->bufferQueue.pop();

                    // Add block to outgoing list
                    blockList.push_back(block);

                    // TODO: Indicate client payload will be transmitted (Still testing)
                    protocolIndicationIDs.insert(header.protocolId.identifier);

                    // Update index
                    index += currentClientProtocols[i]->bufferQueue.front()->length.length;
                }
            }
        } else {
            if (currentClientProtocols[i]->bufferOccupied &&
                currentClientProtocols[i]->bufferEntry->length.length + index <= bearerLen) {
                // Create payload block
                BPPayloadBlock block;
                BPPayloadBlockHeader header;
                header.protocolId = currentClientProtocols[i]->protocolId;
                header.length = currentClientProtocols[i]->bufferEntry->length;
                header.droneName = droneName;
                block.header = header;

                block.header.length.length = htonl(block.header.length.length);

                // Copy payload data
                memcpy(block.payload, currentClientProtocols[i]->bufferEntry->payload, header.length.length);

                // Add block to outgoing list
                blockList.push_back(block);

                // TODO: Indicate client payload will be transmitted (Still testing)
                protocolIndicationIDs.insert(header.protocolId.identifier);

                // Clear buffer if required
                if (currentClientProtocols[i]->queueMode.queueMode == BP_QMODE_ONCE) {
                    currentClientProtocols[i]->bufferOccupied = false;
                    free(currentClientProtocols[i]->bufferEntry->payload);
                }
            }
        }
    }

    std::vector<uint8_t> blocks;
    for (const auto& block : blockList) {
        blocks.insert(blocks.end(), reinterpret_cast<const uint8_t*>(&block), reinterpret_cast<const uint8_t*>(&block) + sizeof(BPPayloadBlock));
    }

    struct sockaddr_in broadcastAddr;
    memset(&broadcastAddr, 0, sizeof(broadcastAddr));
    broadcastAddr.sin_family = AF_INET;
    broadcastAddr.sin_addr.s_addr = inet_addr("255.255.255.255");
    broadcastAddr.sin_port = htons(5950);

    if (blocks.size() == 0) {
        return;
    }

    if (sendto(receiveSocket, blocks.data(), blocks.size(), 0, (struct sockaddr*)&broadcastAddr, sizeof(broadcastAddr)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("logger"), "sendto failed");
    } else {
        bcResults["packetsSent"] += 1;
        RCLCPP_INFO(rclcpp::get_logger("logger"), "Broadcast message sent! From: BeaconingClient::broadcastPackets");
    }

    //TODO: Send indication messages (still testing)
    for (uint8_t protocolId: protocolIndicationIDs) {
        dcp_msgs::msg::PayloadTransmittedIndication payloadTransmittedIndication;
        payloadTransmittedIndication.id = protocolId;
        transmittedPayloadIndicator->publish(payloadTransmittedIndication);
    }

}

/**
 * @brief Receives and processes packets from clients.
 *
 * This function continuously listens for incoming packets from clients in a loop, processing
 * each packet as it is received. It handles network reception in a blocking manner and checks
 * for node shutdown conditions.
 */
void BeaconingClient::receivePackets() {
    while (rclcpp::ok()) {
        uint8_t buffer[BPPAR_MAXIMUM_PACKET_SIZE] = {0}; // Initialize buffer to zero
        struct sockaddr_in clientAddr;
        socklen_t clientAddrLen = sizeof(clientAddr);

        // Blocking call to receive from socket
        ssize_t recvLen = recvfrom(receiveSocket, buffer, sizeof(buffer), 0, (struct sockaddr*)&clientAddr, &clientAddrLen);
        TimeStamp timeStamp;
        if (recvLen <= 0) {
            // Check if the node is shutting down
            if (!rclcpp::ok()) {
                RCLCPP_INFO(rclcpp::get_logger("logger"), "Node is shutting down, stopping receive.");
                break;
            }
            continue; // Continue to next iteration if no valid packet is received
        }
        processPacket(buffer, recvLen, &clientAddr, timeStamp);

    }
}

/**
 * @brief Processes a received packet.
 *
 * @param buffer Pointer to the buffer containing the packet data.
 * @param recvLen Length of the received packet data.
 * @param clientAddr Pointer to the sockaddr_in structure containing the client's address information.
 * @param timeStamp Timestamp indicating when the packet was received.
 */
void BeaconingClient::processPacket(const uint8_t* buffer, size_t recvLen, const sockaddr_in* clientAddr, const TimeStamp& timeStamp) {
    // Convert received bytes back into structure
    BPPayloadBlock block;
    if (recvLen < sizeof(BPPayloadBlock)) {
        RCLCPP_WARN(rclcpp::get_logger("logger"), "Received data length %zd is less than expected structure size", recvLen);
        return;
    }
    memcpy(&block, buffer, sizeof(BPPayloadBlock));

    if (strcmp(block.header.droneName.name, envDroneName) == 0) {
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("logger"), "Received %zd bytes at: BeaconingClient::processPacket", recvLen);

    char clientIP[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(clientAddr->sin_addr), clientIP, INET_ADDRSTRLEN); // Convert to readable format
    RCLCPP_INFO(rclcpp::get_logger("logger"), "From %s:%d", clientIP, ntohs(clientAddr->sin_port));

    uint16_t clientPort = ntohs(clientAddr->sin_port);
    if (clientPort != 5950) {
        return;
    }

    // Print out the structure fields for verification
    RCLCPP_INFO(rclcpp::get_logger("logger"), "Protocol ID: %d, Drone Name: %s, Length: %d. At: BeaconingClient::processPacket. Recvlen: %d",
                block.header.protocolId.identifier,
                block.header.droneName.name,
                ntohl(block.header.length.length),
                recvLen);

    if (lProtocolExists(&(block.header.protocolId))) {
        // Prepare Receive Payload indication
        dcp_msgs::msg::ReceivePayloadIndication indic;
        indic.id = block.header.droneName.name;
        indic.protid = block.header.protocolId.identifier;
        indic.length = ntohl(block.header.length.length);
        indic.timestamp = timeStamp.time.count();

        uint8_t* ptr = block.payload;
        std::vector<uint8_t> dataVector(ptr, ptr + static_cast<int>(indic.length));
        indic.payload = dataVector;

        if (indic.protid == 2) {
            std::ostringstream payloadStream;
            payloadStream << "Payload at BeaconingClient::receivePackets \n";
            for (size_t i = 0; i < indic.payload.size(); ++i) {
                payloadStream << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(indic.payload[i]) << " ";
            }
            RCLCPP_INFO(rclcpp::get_logger("logger"), "%s", payloadStream.str().c_str());
        }
        bcResults["packetsReceived"] += 1;
        RCLCPP_INFO(rclcpp::get_logger("logger"), "Sending indication message at: BeaconingClient::processPacket");
        srpClientIndicator->publish(indic);
    }
}

/**
 * @brief Writes BeaconingClient results to a file.
 *
 * @details
 * - The results include the number of packets sent and received, stored in the `bcResults` map.
 * - If the file cannot be opened for writing, an error is logged and the function returns.
 */
void BeaconingClient::writeResultsToFile() {
    // Create an ofstream object
    std::ofstream outputFile("BC-Results.txt");

    // Check if the file was opened successfully
    if (!outputFile) {
        std::cerr << "Error opening file for writing!" << std::endl;
        return;
    }

    // Write to the file instead of std::cout
    outputFile << "Packets Sent: " << bcResults["packetsSent"] << std::endl;
    outputFile << "Packets Received: " << bcResults["packetsReceived"] << std::endl;

    // Close the file
    outputFile.close();

}