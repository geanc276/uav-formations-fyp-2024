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


#include "dataTypes.h"

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
};

// Transmissable SafetyData with header fields
struct ExtendedSafetyData {
    SafetyData sData;
    NodeIdentifier nodeId;
    TimeStamp tStamp;
    SRPSequenceNumber seqNum;
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

    SharedNeighbourTableEntry(const SharedNeighbourTableEntry &ent) {}

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
        std::string position = "Position = " + std::to_string(sd.position[0]) + ", " + std::to_string(sd.position[1]) + ", " + std::to_string(sd.position[2]) + ";\n";
        std::string direction = "Direction = " + std::to_string(sd.direction[0]) + ", " + std::to_string(sd.direction[1]) + ", " + std::to_string(sd.direction[2]) + ";\n";
        std::string rotation = "Rotation = " + std::to_string(sd.rotation[0]) + ", " + std::to_string(sd.rotation[1]) + ", " + std::to_string(sd.rotation[2]) + ";\n";
        std::string data = position + "Speed = " + std::to_string(sd.speed) + ";\n" + direction + rotation;
        int size = sizeof(*mtx) + sizeof(id) + sizeof(seq) + sizeof(sd) + sizeof(receptionTime) + sizeof(padding);

        return "ID: " + ident + ", Seqno: " + std::to_string(seq.sequenceNumber) + ",\nData:\n" + data;
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
    vector<NeighbourTableEntry>& locate(segment& smt) {
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
};


#endif