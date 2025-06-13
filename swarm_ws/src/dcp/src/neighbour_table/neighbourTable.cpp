#include <iostream>
#include <utility>
#include "dcp/srpDataTypes.h"

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
        std::cout << "Created memory table" << std::endl;
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
            if (d.id == nodeId)
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
        return Shared::NeighbourTableEntry();
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
        Shared::segment smt(bip::open_only, "/neighbourtable");
        bip::interprocess_mutex *mtx = smt.find_or_construct<bip::interprocess_mutex>("neighbourtablemutex")();
        mtx->lock();

        RCLCPP_INFO(rclcpp::get_logger("logger"), "At NeighbourTable::ntAddEntry, adding/Updating drone neighbour: %s", droneName.c_str());

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

        RCLCPP_INFO(rclcpp::get_logger("logger"), "Drone %s table entry: %s", droneName.c_str(), newEntry->toString().c_str());

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
        RCLCPP_ERROR(rclcpp::get_logger("logger"), "Managed shared memory add exception: %s", ex.what());
    }
}

/*
    Remove an entry from the Neighbour Table
    @param nodeId: Node Identifier of the neighbour table entry to remove
*/
void NeighbourTable::ntRemoveEntry(NodeIdentifier nodeId)
{
    try
    {
        // Open the shared memory segment
        Shared::segment smt(bip::open_only, "/neighbourtable");
        bip::interprocess_mutex *mtx = smt.find_or_construct<bip::interprocess_mutex>("neighbourtablemutex")();
        mtx->lock();

        // Locate the data in shared memory
        auto& data = Shared::locate(smt);

        // Iterate over the neighbour table entries
        for (auto it = data.begin(); it != data.end(); ++it)
        {
            if (it->id == nodeId)
            {
                // Lock the specific entry's mutex before modifying
                it->mtx->lock();
                data.erase(it);
                // Unlock the entry's mutex and break the loop
                it->mtx->unlock();
                break;
            }
        }

        // Unlock the global mutex after operation
        mtx->unlock();

        RCLCPP_INFO(rclcpp::get_logger("logger"), "Removed entry with Node Identifier: %d", nodeId);
    }
    catch (const std::exception &ex)
    {
        RCLCPP_ERROR(rclcpp::get_logger("logger"), "Managed shared memory remove exception: %s", ex.what());
    }
}