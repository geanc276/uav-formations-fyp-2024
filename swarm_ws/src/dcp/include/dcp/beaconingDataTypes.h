#ifndef BEACONING_DATA_TYPES_H
#define BEACONING_DATA_TYPES_H

#include <queue>
#include <string>
#include <string>
#include <chrono>
#include <ctime>
#include "Types.h"

//Protocol Ids
#define BP_PROTID_SRP       0x0001
#define BP_PROTID_VARDIS    0x0002

//Queue Types
#define BP_QMODE_QUEUE      0
#define BP_QMODE_ONCE       1
#define BP_QMODE_REPEAT     2

// Beaconing protocol identifier
struct BPProtocolId {
    uint8_t identifier;
};

// Length of contained data
struct BPLength {
    int32_t length = 0;
};

// Name of local drone for Beaconing protocol
struct BPDroneName {
    char name[6];
};

// Headers of tranmissable BP Payload Block
struct BPPayloadBlockHeader {
    BPProtocolId protocolId;
    BPDroneName droneName;
    BPLength length;
};

// Beaconing protocol transmission block
struct BPPayloadBlock {
    BPPayloadBlockHeader header;
    uint8_t payload[VARDISPAR_MAX_PAYLOAD_SIZE]; // Max size of SRP payload
};

// Queing mode of Beaconing protocol
struct BPQueueingMode {
    uint8_t queueMode;
};

// Local sending buffer entry
struct BPBufferEntry {
    BPLength length;
    uint8_t* payload;
};

// Registered client protocol description
struct BPClientProtocol {
    BPProtocolId protocolId;
    std::string protocolName;
    BPLength maxPayloadSize;
    BPQueueingMode queueMode;
    TimeStamp *timestamp;
    std::queue<BPBufferEntry*> bufferQueue;
    bool bufferOccupied;
    BPBufferEntry *bufferEntry;
};

#endif