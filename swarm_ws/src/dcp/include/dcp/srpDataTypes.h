#ifndef SRP_DATA_TYPES_H
#define SRP_DATA_TYPES_H

#include <inttypes.h>
#include <vector>
#include <utility>
#include <string>
#include <chrono>
#include <ctime>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/managed_mapped_file.hpp>
#include <boost/interprocess/containers/vector.hpp>   
#include <boost/interprocess/containers/string.hpp> 
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include "rclcpp/rclcpp.hpp"

#include "Types.h"

#define SRPPAR_NEIGHBOUR_TABLE_TIMEOUT 3000 // If changed update the below SRP_TRAVERSAL_TIME
#define SRPPAR_NEIGHBOUR_TABLE_TIMEOUT_TIMER 3000ms // If changed update the below SRP_TRAVERSAL_TIME and above SRPPAR_NEIGHBOUR_TABLE_TIMEOUT (same value, no "ms")
#define SRP_TRAVERSAL_TIME 600ms // Needs to be minimum SRPPAR_NEIGHBOUR_TABLE_TIMEOUT/5


// Local drone name
struct DroneName {
    char name[5];
};

// State Reporting protocol sequence number
struct SRPSequenceNumber {
    uint32_t sequenceNumber;
};

// Local drone safety data
struct SafetyData {
    int16_t position[3];
    uint8_t speed;
    int16_t direction[3];
    int16_t rotation[3];

    std::string toString() const {
        std::ostringstream oss;
        oss << "Position: [" << position[0] << ", " << position[1] << ", " << position[2] << "], ";
        oss << "Speed: " << static_cast<int>(speed) << ", ";
        oss << "Direction: [" << direction[0] << ", " << direction[1] << ", " << direction[2] << "], ";
        oss << "Rotation: [" << rotation[0] << ", " << rotation[1] << ", " << rotation[2] << "]";
        return oss.str();
    }
};

// Transmissable SafetyData with header fields
struct ExtendedSafetyData {
    SafetyData sData;
    NodeIdentifier nodeId;
    TimeStamp tStamp;
    SRPSequenceNumber seqNum;

    std::string toString() const {
        std::ostringstream oss;
        oss << "{ SafetyData: {" << sData.toString() << "}, ";
        oss << "NodeIdentifier: " << nodeId << ", ";
        oss << "TimeStamp: " << tStamp << ", ";
        oss << "SequenceNumber: " << seqNum.sequenceNumber << "}";
        return oss.str();
    }
};

namespace bip = boost::interprocess;

/*
    Neighbour Table type definitions
*/
// Neighbour Table entry for a single neighbour drone
struct SharedNeighbourTableEntry {
    /*
        Neighbour Table Entry constructors
    */
    SharedNeighbourTableEntry() {}

//    SharedNeighbourTableEntry(const SharedNeighbourTableEntry) {}

    SharedNeighbourTableEntry(NodeIdentifier ident, DroneName name, SRPSequenceNumber seqno, SafetyData sada, TimeStamp recep) :
        id(ident), name(name), seq(seqno), sd(sada), receptionTime(recep)
    { }

    // Identifier of neighbour drone
    NodeIdentifier id;
    // Name of neighbour drone
    DroneName name;
    // Sequence number of received SafetyData
    SRPSequenceNumber seq;
    // Received SafetyData of neighbour drone
    SafetyData sd;
    // Reception time of the SafetyData contained in this entry
    TimeStamp receptionTime;
    // Mutex controlling access to this table entry
    boost::interprocess::interprocess_mutex *mtx;
    uint8_t padding[33] = {0};

    // String debug output of entry
    std::string toString() {
        std::string ident(name.name);
        std::string position = "Position = " + std::to_string(sd.position[0]) + ", " + std::to_string(sd.position[1]) + ", " + std::to_string(sd.position[2]) + ", ";
        std::string direction = "Direction = " + std::to_string(sd.direction[0]) + ", " + std::to_string(sd.direction[1]) + ", " + std::to_string(sd.direction[2]) + ", ";
        std::string rotation = "Rotation = " + std::to_string(sd.rotation[0]) + ", " + std::to_string(sd.rotation[1]) + ", " + std::to_string(sd.rotation[2]) + "}";
        std::string data = position + "Speed = " + std::to_string(sd.speed) + ", " + direction + rotation;
//        int size = sizeof(*mtx) + sizeof(id) + sizeof(seq) + sizeof(sd) + sizeof(receptionTime) + sizeof(padding);

        return "ID: " + ident + ", Seqno: " + std::to_string(seq.sequenceNumber) + ", Data: {" + data;
    }
};

using NeighbourTableEntry = SharedNeighbourTableEntry;

/*
    Shared namespace definitions
*/

namespace Shared {
    // Alias segment as boost::interprocess::managed_mapped_file
    using segment = bip::managed_mapped_file;
    // Alies segment_manager as boost::interprocess::managed_mapped_file::segment_manager
    using segment_manager = segment::segment_manager;

    // Allocator template
    template <typename T> using alloc = bip::allocator<T, segment_manager>;
    // Shared memory vector template
    template <typename T> using vector = bip::vector<T, alloc<T> >;

    using NeighbourTableEntry = SharedNeighbourTableEntry;

    // Find or constructs Neighbour Table in shared memory
    inline vector<NeighbourTableEntry>& locate(segment& smt) {
        vector<NeighbourTableEntry>* v = smt.find_or_construct<vector<NeighbourTableEntry> >("NeighbourTableVector")(smt.get_segment_manager());
        return *v;
    }
}

/*
    Neighbour Table class definition
*/

class NeighbourTable {
private:
    // Vector of references to each neighbour entry in table
    std::vector<Shared::NeighbourTableEntry*> neighbourPtrs;
    // Mutex for the entire Neighbour Table
    boost::interprocess::interprocess_mutex *mtx_;
public:
    NeighbourTable();

    Shared::NeighbourTableEntry ntLookup(NodeIdentifier nodeId);

    Shared::vector<Shared::NeighbourTableEntry> ntListAllNodes();

    void ntAddEntry(ExtendedSafetyData entry, std::string droneName);

    void ntRemoveEntry(NodeIdentifier nodeId);
};



#endif