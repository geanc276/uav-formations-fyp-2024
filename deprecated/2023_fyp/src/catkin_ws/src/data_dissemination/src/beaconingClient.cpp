#include "headers/beaconingClient.h"
#include "headers/srpClient.h"


/*
    Beaconing protocol constructor
*/
BeaconingClient::BeaconingClient(ros::Publisher srpClientSender)
{
    // Call initializing method
    initBeaconingProtocol(srpClientSender);
}

/*
    Check if protocol exists in registered protocols
    @param id: Client protocol identifier
    @return True if it exists else false
*/
bool BeaconingClient::lProtocolExists(BPProtocolId *id)
{
    // For each registered protocol check if identifier matches supplied ID
    for (int i = 0; i < currentClientProtocols.size(); i++)
    {
        if (currentClientProtocols[i]->protocolId.identifier == id->identifier)
        {
            return true;
        }
    }
    return false;
}

/*
    Remove protocol from current client protocols
    @param id: Client protocol identifier
*/
void BeaconingClient::lRemoveProtocol(BPProtocolId id)
{
    // For each registered protocol check if identifier matches
    std::vector<BPClientProtocol *>::iterator iter = currentClientProtocols.begin();
    for (int i = 0; i < currentClientProtocols.size(); i++)
    {
        // If identifier matches remove registered protocol
        if (currentClientProtocols[i]->protocolId.identifier == id.identifier)
        {
            currentClientProtocols.erase(iter);
        }
        next(iter);
    }
}

/*
    Lookup protocol from current client protocols
    @param id: Client protocol identifier
    @return Client protocol if it exists
*/
BPClientProtocol* BeaconingClient::lLookupProtocol(BPProtocolId id)
{
    for (int i = 0; i < currentClientProtocols.size(); i++)
    {
        if (currentClientProtocols[i]->protocolId.identifier == id.identifier)
        {
            return currentClientProtocols[i];
        }
    }
    return NULL;
}

/*
    Register protocol request callback function
    @param request: Received register protocol request
*/
void BeaconingClient::registerProtocolCallback(const data_dissemination_msgs::RegisterProtocolRequest& request)
{
    int result = registerProtocol(request);
    // Process result value
    ROS_INFO_STREAM("SRP Client register returned: " << result);
}

/*
    Register a new client protocol with the Beaconing protocol
    @param request: Received register protocol request
    @return Register status code
*/
int BeaconingClient::registerProtocol(const data_dissemination_msgs::RegisterProtocolRequest& request)
{
    // Unpack request values
    BPProtocolId id = {request.id};
    BPLength length;
    length.length = request.length;
    BPQueueingMode mode;
    mode.queueMode = request.mode;

    // Performs at least these actions (potentially more)
    if (lProtocolExists(&id) == true)
    {
        return BP_STATUS_PROTOCOL_ALREADY_REGISTERED;
    }
    if (length.length > (BPPAR_MAXIMUM_PACKET_SIZE - (sizeof(BPPayloadBlockHeader) + 4)))
    {
        return BP_STATUS_ILLEGAL_MAX_PAYLOAD_SIZE;
    }
    // Create new client protocol
    BPClientProtocol *newent = new BPClientProtocol;

    // Create protocol fields
    TimeStamp *time = new TimeStamp;
    auto t = std::chrono::system_clock::now().time_since_epoch();
    time->time = t;
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

/*
    Transmits payload request callback method
    @param msg: Received transmit payload request for handling
*/
void BeaconingClient::transmitPayloadCallback(const data_dissemination_msgs::TransmitPayloadRequest& msg)
{
    ROS_INFO_STREAM("transmitPayload returned: " << transmitPayload(msg));
}

/*
    Prepare the requested payload for transmission and add to queue/buffer
    @param request: Received transmit payload request
    @return: Transmit payload status code
*/
int BeaconingClient::transmitPayload(const data_dissemination_msgs::TransmitPayloadRequest& request)
{
    // Create from msg
    BPProtocolId id = {request.id};
    BPLength length = {request.length};
    
    std::vector<unsigned char> data = request.payload;
    uint8_t payload[length.length];
    memcpy(&payload, data.data(), data.size());

    // Check requesting client protocol
    if (lProtocolExists(&id) == false)
    {
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
    register long sum;
    unsigned short oddbyte;
    register short answer;

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

/*
    Transmit a broadcast Ethernet beacon
    @destination
    @param socketFd: Socket descriptor to be used to transmit the beacon
*/
void BeaconingClient::transmitBeacon(int socketFd)
{
    // Configure required values
    int index = 0;
    int bearerLen = BPPAR_MAXIMUM_PACKET_SIZE;
    BPDroneName droneName = {};

    char *namePtr = std::getenv("DRONE_NAME");
    for (int i = 0; i < 5; i++) {
        droneName.name[i] = namePtr[i];
    }

    if (packetNumber == 0) {
        ROS_INFO_STREAM("Starting beaconing.");
    }

    std::vector<BPPayloadBlock> blockList = {};
    // Collect beacon payload data from client protocol buffers and queues
    for (int i = 0; i < currentClientProtocols.size(); i++)
    {
        if (currentClientProtocols[i]->queueMode.queueMode == BP_QMODE_QUEUE)
        {
            if (currentClientProtocols[i]->bufferQueue.empty() == false)
            {
                while (currentClientProtocols[i]->bufferQueue.empty() == false &&
                       currentClientProtocols[i]->bufferQueue.front()->length.length + index <= bearerLen)
                {
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

                    // Indicate client payload will be transmitted (Not implemented)

                    // Update index
                    index = index + currentClientProtocols[i]->bufferQueue.front()->length.length;
                }
            }
        }
        else
        {
            if (currentClientProtocols[i]->bufferOccupied == true &&
                currentClientProtocols[i]->bufferEntry->length.length + index <= bearerLen)
            {
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

                // Indicate client payload will be transmitted (Not implemented)

                // Clear buffer if required
                if (currentClientProtocols[i]->queueMode.queueMode == BP_QMODE_ONCE)
                {
                    currentClientProtocols[i]->bufferOccupied = false;
                    free(currentClientProtocols[i]->bufferEntry->payload);

                }
            }
        }
    }
    // Concatenate all payload data blocks
    uint8_t blocks[sizeof(BPPayloadBlock)*blockList.size()];
    memset(blocks, 0, sizeof(BPPayloadBlock)*blockList.size());
    uint8_t *ptr = blocks;

    // Create packet
    for (int i = 0; i < blockList.size(); i++)
    {
        // Assemble serialized data packet
        memcpy(ptr, blockList.data(), sizeof(BPPayloadBlock));
        ptr = ptr + sizeof(BPPayloadBlock);
    }

    // Create headers
    uint8_t header[sizeof(ether_header) + sizeof(iphdr) + sizeof(udphdr)];
    memset(header, 0, sizeof(ether_header) + sizeof(iphdr) + sizeof(udphdr));

    struct ether_header *eh = (struct ether_header *) header;

    // Fill eth header
    eh->ether_dhost[0] = 0xff;
    eh->ether_dhost[1] = 0xff;
    eh->ether_dhost[2] = 0xff;
    eh->ether_dhost[3] = 0xff;
    eh->ether_dhost[4] = 0xff;
    eh->ether_dhost[5] = 0xff;

    ether_addr * ad;
    ad = ether_aton(localHWaddress);

    for (int i = 0; i < 6; i++) {
        eh->ether_shost[i] = ad->ether_addr_octet[i];
    }
    eh->ether_type = 0x0008;

    struct iphdr *iph = (struct iphdr *) (header + sizeof(ether_header));
    
    // Fill ip header
    iph->ihl = 5;
    iph->version = 4;
    iph->tos = 0;
    iph->tot_len = htons(sizeof(struct iphdr) + sizeof(struct udphdr) + sizeof(blocks));
    iph->id = htons(packetNumber); // Id of this packet
    iph->frag_off = 0;
    iph->ttl = 255;
    iph->protocol = IPPROTO_UDP;
    iph->saddr = inet_addr("0.0.0.0"); // Spoof the source ip address
    iph->daddr = inet_addr("1.1.1.1"); // Spoof destination

    iph->check = csum ((unsigned short *) header, sizeof(struct iphdr) + sizeof(struct udphdr) + sizeof(blocks));

    // UDP header
    struct udphdr *udp = (struct udphdr *) (header + sizeof(ether_header) + sizeof(iphdr));
    udp->source = htons(5950);
    udp->dest = htons(5950);
    udp->len = htons(sizeof(udphdr) + sizeof(blocks)); // UDP header size is always 8 bytes
    udp->check = 0;

    // Ethernet frame destination
    const unsigned char ether_broadcast_addr[]=
    {0xff,0xff,0xff,0xff,0xff,0xff};

    // Construct address structure
    struct sockaddr_ll addr={0};
    addr.sll_family=PF_PACKET;
    addr.sll_ifindex=ifindex;
    addr.sll_halen=ETHER_ADDR_LEN;
    addr.sll_protocol=htons(ETH_P_IP);
    memcpy(addr.sll_addr,ether_broadcast_addr,ETHER_ADDR_LEN);

    socklen_t len = sizeof(addr);

    // Collect headers and block data
    uint8_t data[BPPAR_MAXIMUM_PACKET_SIZE];
    memcpy(data, header, sizeof(ether_header) + sizeof(iphdr) + sizeof(udphdr));
    if (blockList.size() > 0) {
        memcpy(data + sizeof(ether_header) + sizeof(iphdr) + sizeof(udphdr), blocks, blockList.size()*sizeof(BPPayloadBlock));
    }
    
    size_t dataSize = blockList.size()*sizeof(BPPayloadBlock) + sizeof(ether_header) + sizeof(iphdr) + sizeof(udphdr);

    // Send packet and print byte amount
    int bytes = sendto(socketFd, (char*) data, dataSize, 0, (sockaddr *) &addr, len);
    packetNumber++;

    // Clean up
    while (blockList.size() > 0)
    {
        BPPayloadBlock block = blockList.front();
        //delete block.header;
        blockList.erase(blockList.begin());
    }
}

/*
    Receive a beacon from another Data Dissemination protocol instance
*/
void BeaconingClient::receiveBeacon()
{
    // Configure required values
    int index = 0;
    int bearerLen = BPPAR_MAXIMUM_PACKET_SIZE;
    char transMAC[18]; // Node ID of node sending the beacon

    // Read socket data
    uint8_t data[BPPAR_MAXIMUM_PACKET_SIZE];
    uint8_t *ptr = data;

    int res = recvfrom(beaconingSocket, data, BPPAR_MAXIMUM_PACKET_SIZE, 0, 0, 0);
    
    // If nothing is received return
    if (res == -1) {
        return;
    }

    // Unpack packet headers
    int headerSize = sizeof(ether_header) + sizeof(iphdr) + sizeof(udphdr);
    uint8_t headers[headerSize];
    memset(headers, 0, headerSize);
    struct ether_header *eh = (struct ether_header *) headers;
    struct iphdr *iph = (struct iphdr *) (headers + sizeof(ether_header));
    struct udphdr *udp = (struct udphdr *) (headers + sizeof(ether_header) + sizeof(iphdr));
    memcpy(headers, ptr, headerSize);

    // Update ptr
    ptr = ptr + headerSize;
    
    // Validate and use header fields
    snprintf(transMAC, sizeof(transMAC), "%02x:%02x:%02x:%02x:%02x:%02x",
         eh->ether_shost[0], eh->ether_shost[1], eh->ether_shost[2], eh->ether_shost[3], eh->ether_shost[4], eh->ether_shost[5]);

    // If headers are not valid
    if (strcmp(transMAC, "ff:ff:ff:ff:ff:ff") == 0 || ntohs(udp->source) != 5950) // Local node address
    {
        // Drop beacon
        ROS_INFO_STREAM("Dropping packet....");
        return;
    }

    ROS_INFO_STREAM("Received " << res << " bytes from " << transMAC << ".");

    // Otherwise process received beacon
    while (index < res)
    {
        // Unpack data
        uint8_t payload[sizeof(BPPayloadBlock)];
        memset(payload, 0, sizeof(BPPayloadBlock));
        BPPayloadBlock *block = (BPPayloadBlock *) payload;
        memcpy(payload, ptr, sizeof(BPPayloadBlock));

        int blockLength = (int) block->header.length.length + sizeof(BPPayloadBlockHeader);
        index = index + sizeof(BPPayloadBlock);
        
        // If client destination client protocol exists
        if (lProtocolExists(&(block->header.protocolId)) == true)
        {
            // Prepare Receive Payload indication
            data_dissemination_msgs::ReceivePayloadIndication indic;
            indic.id = block->header.droneName.name;
            indic.length = ntohl(block->header.length.length);

            uint8_t* ptr = block->payload;

            std::vector<uint8_t> dataVector(ptr, ptr + (int)indic.length);

            indic.payload = dataVector;

            // Send indication
            srpClientIndicator.publish(indic);
        }
    }
}

/*
    Initialize the beaconing protocol
    @param srpClientSender: ROS Publisher for sending indications to State Reporting protocol
*/
void BeaconingClient::initBeaconingProtocol(ros::Publisher srpClientSender)
{
    // Configure required values
    currentClientProtocols = {};
    packetNumber = 0;
    srpClientIndicator = srpClientSender;

    // Retreive UWB max packet size
    BPPAR_MAXIMUM_PACKET_SIZE = 1024; // Read from config

    // Create socket
    beaconingSocket = socket(PF_PACKET, SOCK_RAW, IPPROTO_UDP);
    if (beaconingSocket == -1)
    {
        ROS_INFO_STREAM("Error in socket creation.");
        ROS_INFO_STREAM(strerror(errno));
    }

    // Set socket options
    fcntl(beaconingSocket, F_SETFL, O_NONBLOCK);
    int broadcast=1;
    setsockopt(beaconingSocket, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

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
    const char *if_name = ifname;
    ROS_INFO_STREAM(if_name);
    freeifaddrs(ifap);

    // Set ifr structure
    struct ifreq ifr;
    memset(&ifr,0,sizeof(ifr));
    strncpy(ifr.ifr_name,if_name,sizeof(ifr.ifr_name));

    // Get the index number of the interface
    int err = ioctl(beaconingSocket,SIOCGIFINDEX,&ifr);
    if (err < 0)
        ROS_INFO_STREAM("Something broke at IF ioctl");

    // Store network interface number
    memcpy(&ifindex, &ifr.ifr_ifindex, sizeof(int));
    
    // Reset ifr structure
    memset(&ifr,0,sizeof(ifr));   
    strncpy(ifr.ifr_name,if_name,sizeof(ifr.ifr_name));

    // Get the network interface hardware address
    err = ioctl(beaconingSocket,SIOCGIFHWADDR,&ifr);
    if (err < 0)
        ROS_INFO_STREAM("Something broke at HW address ioctl");
    
    // Store the network interface hardware address
    memset(localHWaddress, 0, 18);
    strncpy(localHWaddress, ether_ntoa( (ether_addr *) ifr.ifr_hwaddr.sa_data), 18);

    ROS_INFO_STREAM(localHWaddress);

    // Prepare address for binding socket descriptor
    const unsigned char ether_broadcast_addr[]=
    {0xff,0xff,0xff,0xff,0xff,0xff};
    
    // Set address values
    struct sockaddr_ll addr={0};
    addr.sll_family=PF_PACKET;
    addr.sll_ifindex=ifindex;
    addr.sll_protocol=htons(ETH_P_IP);
    addr.sll_halen=ETHER_ADDR_LEN;
    memcpy(addr.sll_addr,ether_broadcast_addr,ETHER_ADDR_LEN);

    // Bind socket descriptor to address
    if (bind(beaconingSocket, (struct sockaddr *)&addr, sizeof(addr)) != 0)
    {
        ROS_INFO_STREAM("Error in socket bind.");
    }
}

/*
    Beaconing protocol main operation method called periodically by ROS Timer
    @param event: ROS Timer event to indicate beaconing period passed
*/

void BeaconingClient::runBeaconingTransmit(const ros::TimerEvent& event)
{
    // Main periodic actions of beaconing protocol
    // Send Beacon
    transmitBeacon(beaconingSocket);
}

void BeaconingClient::runBeaconingReceive(const ros::TimerEvent& event)
{
    // Receive Beacon
    receiveBeacon();
}