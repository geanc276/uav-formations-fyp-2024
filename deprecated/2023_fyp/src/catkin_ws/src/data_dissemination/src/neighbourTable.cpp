#include <iostream>
#include <cstdlib> //std::system
#include <cstddef>
#include <cassert>
#include <utility>

#include "headers/srpDataTypes.h"

/*
    Neighbour Table class constructor
*/
NeighbourTable::NeighbourTable()
{
    // Attempt shared memory creation
    try {
        if (!bip::file_mapping::remove("/neighbourtable")) {
            // Remove failed
            bip::remove_file_on_destroy destroy("/neighbourtable");
        }
        // Create memory mapped file
        Shared::segment smt(bip::create_only, "/neighbourtable", 10240);
        // Contruct neighbour table mutex
        mtx_ = smt.find_or_construct<bip::interprocess_mutex>("neighbourtablemutex")();
    } catch (const std::exception &ex) {
        std::cout << "shared_memory creation ex: "  << ex.what() << "\n";
    }
}

/*
    Lookup an entry in the neighbour table
    @param nodeId: Node Identifier of the neighbour table entry to lookup
    @return Neighbour Table Entry for nodeId or an empty entry
*/
Shared::NeighbourTableEntry NeighbourTable::ntLookup(NodeIdentifier nodeId) 
{
    // Try lock neighbour table mutex and locate entry for nodeId
    try {
        mtx_->lock();
        Shared::segment smt(bip::open_only, "/neighbourtable");
        auto& data = Shared::locate(smt);

        // Check if located data is what we need
        for (auto& d : data) {
            if (strcmp(d.id.identifier, nodeId.identifier) == 0) 
            {
                // If ID's match then lock individual entry and return data
                mtx_->unlock();
                // Manage unlocking this mutex when done with d
                d.mtx->lock();
                return d;
            }        
        }
        // Unlock mutex and return
        mtx_->unlock();
        return Shared::NeighbourTableEntry();
    } catch (const std::exception &ex) {
        std::cout << "shared_memory lookup ex: " << ex.what() << "\n";
    }
}

/*
    List all entries in Neighbour Table
    @return Shared vector containing all neighbour table entries
*/
Shared::vector<Shared::NeighbourTableEntry> NeighbourTable::ntListAllNodes() 
{
    try
    {  
        // Open Neighbour Table 
        Shared::segment smt(bip::open_only, "/neighbourtable");
        // Find or create mutex and lock it
        bip::interprocess_mutex *mtx = smt.find_or_construct<bip::interprocess_mutex>("neighbourtablemutex")();
        mtx->lock();
        auto& data = Shared::locate(smt);
        // Copy data to ensure no simultaneous accesses
        // Otherwise keep mutex lock until done with data
        mtx->unlock();
        return data;
    } 
    catch (const std::exception &ex) {
        std::cout << "shared_memory list ex: "  << ex.what() << "\n";
    }
}

/*
    Add an entry to the Neighbour Table
    @param entry: The ExtendedSafetyData structure to store in the neighbour table
    @param droneName: The string name of the drone to store it under
*/
void NeighbourTable::ntAddEntry(ExtendedSafetyData entry, std::string droneName)
{
    try
    {   
        // Open neighbour table
        Shared::segment smt(bip::open_only, "/neighbourtable");
        // Find or construct mutex and lock access
        bip::interprocess_mutex *mtx = smt.find_or_construct<bip::interprocess_mutex>("neighbourtablemutex")();
        mtx->lock();     

        // Debug output
        std::cout << "Adding/Updating drone neighbour: " << droneName << "\n";
        
        // Prepare sender drone name for storage
        DroneName name;
        strcpy(name.name, droneName.c_str());

        // Find entry or construct and store a new entry
        Shared::NeighbourTableEntry *newEntry = smt.find_or_construct<Shared::NeighbourTableEntry>(name.name, std::nothrow)(entry.nodeId, name, entry.seqNum, entry.sData, entry.tStamp);
        
        // Create or find mutex for specific entry
        mtx->unlock();
        std::string s(name.name);
        s = s + "mutex";
        newEntry->mtx = smt.find_or_construct<bip::interprocess_mutex>(s.c_str())();
        newEntry->mtx->lock();

        std::cout << "Drone " << droneName << " table entry: " << newEntry->toString();

        // Received entry is newer than current neighbour table entry
        if (newEntry->seq.sequenceNumber < entry.seqNum.sequenceNumber) {
            // Update values in memory
            newEntry->receptionTime = entry.tStamp;
            newEntry->sd = entry.sData;
            newEntry->seq = entry.seqNum;
        }

        newEntry->mtx->unlock();
    } 
    catch (const std::exception &ex) {
        std::cout << "managed_shared_memory add ex: "  << ex.what() << "\n";
    }
}
