#include "dcp/DatabaseManager.h"
#include "dcp/Helpers.h"
#include "dcp/beaconingDataTypes.h"
#include "dcp/VarDisClient.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>

#include "stdlib.h"

/**
 * @brief Used to test implementation during development.
 * On the terminal navigate to swarm_ws/build/dcp and run the below.
 * ./binary
 */
int normalChecking() {
    VarDisClient varDisClient = VarDisClient(); // Create custom queues and real-time database
    DatabaseManager& databaseManager = varDisClient.getDatabaseManager(); // Get database instance

    Helpers helpers = Helpers(); // Helper functions

    // Create a "list" of uint8_t
    std::vector<uint8_t> value;
    // Add some bytes to the "list"
    value.push_back(static_cast<uint8_t>(0x41)); // Example: adding byte 'A'
    value.push_back(static_cast<uint8_t>(0x42)); // Example: adding byte 'B'
    value.push_back(static_cast<uint8_t>(0x43)); // Example: adding byte 'C'

    VarId startingId = 1;

    VarSpec spec(startingId,
                 helpers.getNodeIdentifier(),
                 2,
                 "Description");

    // Create a Database entry
    databaseManager.create(spec, value.size(), value);

    std::cout << "--------- Checking 1 variable was added to database ---------" << std::endl;
    // Get all the entries, should just be 1
    std::vector<DBEntry> entries = databaseManager.getAllDBEntries();
    databaseManager.printResults(entries);
    std::cout << std::endl;

    std::cout << "--------- Checking variable lookup ---------" << std::endl;
    // Get the only database entry
    DBEntry entry = databaseManager.lookup(startingId);
    std::cout << entry << std::endl;
    std::cout << std::endl;

    std::cout << "--------- Checking variable was updated in database ---------" << std::endl;
    value.pop_back(); // Remove last item from the "list"
    databaseManager.updateDBEntryById(startingId, value.size(), value); // Update "list" in database
    DBEntry updated_entry = databaseManager.lookup(startingId);
    std::cout << updated_entry << std::endl;
    std::cout << std::endl;

    std::cout << "--------- Checking variable was prepared for deletion ---------" << std::endl;
    // Prepare to delete from database, only changes toBeDeleted flag to 1/true
    databaseManager.deleteDBEntryById(startingId);
    // Check the update by getting all entries (again just 1).
    entries = databaseManager.getAllDBEntries();
    databaseManager.printResults(entries);
    std::cout << std::endl;

    varDisClient.createQ.qAppend(1);
    varDisClient.createQ.qAppend(2);
    varDisClient.createQ.qAppend(3);
    varDisClient.createQ.qAppend(startingId);

    std::cout << "--------- Checking custom queue is populated ---------" << std::endl;
    varDisClient.createQ.printQueue();
    std::cout << std::endl;

    std::cout << "--------- Checking queue items are removed ---------" << std::endl;
    varDisClient.createQ.qDropNonexisting();
    varDisClient.createQ.printQueue();
    std::cout << std::endl;

    std::cout << "--------- Checking queue items are removed pt. 2 ---------" << std::endl;
    varDisClient.createQ.qDropNonexistingDeleted();
    varDisClient.createQ.printQueue();

    char input;

    while (true) {
        std::cout << "Enter 'Y' if you are ready to delete the variable from the RTDB: ";
        std::cin >> input;

        // Check if the input character is 'Y' or 'y'
        if (input == 'Y' || input == 'y') {

            // Fully delete/remove the entry from the DB
            std::cout << "--------- Checking variable was deleted from database ---------" << std::endl;
            databaseManager.remove(1);
            // Check the entry got removed
            entries = databaseManager.getAllDBEntries();
            databaseManager.printResults(entries);
            std::cout << std::endl;

            break;
        }
    }

    return 1;
}

/**
 * @brief Checks that the serialization is working
 */
void serializeChecking() {
    VarDisClient varDisClient = VarDisClient(); // Create custom queues and real-time database
    DatabaseManager& databaseManager = varDisClient.getDatabaseManager(); // Get database instance
    Helpers helpers = Helpers(); // Helper functions

    // Create a "list" of uint8_t
    std::vector<uint8_t> value;
    // Add some bytes to the "list"
    value.push_back(static_cast<uint8_t>(0x41)); // Example: adding byte 'A'
    value.push_back(static_cast<uint8_t>(0x42)); // Example: adding byte 'B'
    value.push_back(static_cast<uint8_t>(0x43)); // Example: adding byte 'C'

    VarSpec spec(databaseManager.getNextAvailableVarId(),
                 helpers.getNodeIdentifier(),
                 10,
                 "Description");

    VarUpd upd(1, 1, static_cast<VarLen>(value.size()), value);

    VarCreate varCreate = {spec, upd};
    std::vector<VarCreate> varCreateList = {varCreate, varCreate};

    std::cout << "-------------Original VarCreateList-------------" << std::endl;
    databaseManager.printResults(varCreateList);
    std::cout << std::endl;

    std::vector<uint8_t> serializedData = helpers.serializeVarCreateList(varCreateList);
    for (const auto& byte : serializedData) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    std::cout << std::endl;

    std::vector<VarCreate> deserializedVarCreate = helpers.deserializeVarCreateList(serializedData);

    std::cout << "-------------Deserialized VarCreate-------------" << std::endl;
    databaseManager.printResults(deserializedVarCreate);


}

/**
 * @brief Checks that adding multiple IEElements is working
 */
void addingIEElements() {
    VarDisClient varDisClient = VarDisClient(); // Create custom queues and real-time database
    DatabaseManager& databaseManager = varDisClient.getDatabaseManager(); // Get database instance
    Helpers helpers = Helpers(); // Helper functions

    // Create a "list" of uint8_t
    std::vector<uint8_t> value;
    // Add some bytes to the "list"
    value.push_back(static_cast<uint8_t>(0x41)); // Example: adding byte 'A'
    value.push_back(static_cast<uint8_t>(0x42)); // Example: adding byte 'B'
    value.push_back(static_cast<uint8_t>(0x43)); // Example: adding byte 'C'

    VarSpec spec1(databaseManager.getNextAvailableVarId(),
                  helpers.getNodeIdentifier(),
                  2,
                  "Variable1");

    databaseManager.create(spec1, value.size(), value);

    VarSpec spec2(databaseManager.getNextAvailableVarId(),
                  helpers.getNodeIdentifier(),
                  2,
                  "Variable2");

    // Create a Database entry
    databaseManager.create(spec2, value.size(), value);

    varDisClient.updateQ.qAppend(1);
    varDisClient.updateQ.qAppend(2);
    varDisClient.deleteQ.qAppend(1);
    varDisClient.deleteQ.qAppend(2);
    varDisClient.reqUpdQ.qAppend(1);
    varDisClient.reqUpdQ.qAppend(2);
    varDisClient.reqCreateQ.qAppend(1);
    varDisClient.reqCreateQ.qAppend(2);

    std::cout << "--------- Checking 2 variables were added to database ---------" << std::endl;
    std::vector<DBEntry> entries = databaseManager.getAllDBEntries();
    databaseManager.printResults(entries);
    std::cout << std::endl;

    size_t currentPayloadSize = 0;
    InformationElement informationElementSummaries = varDisClient.composeSummaryIEElements(&currentPayloadSize);
    InformationElement informationElementCreates = varDisClient.composeCreateVariablesIEElements(&currentPayloadSize);
    InformationElement informationElementUpdates = varDisClient.composeUpdateIEElements(&currentPayloadSize);
    InformationElement informationElementDeletes = varDisClient.composeDeleteVariablesIEElements(&currentPayloadSize);
    InformationElement informationElementRequestUpdates = varDisClient.composeRequestVarUpdatesIEElements(&currentPayloadSize);
    InformationElement informationElementRequestCreates = varDisClient.composeRequestVarCreatesIEElements(&currentPayloadSize);

    std::vector<VarSumm> varSumms = helpers.deserializeVarSummList(informationElementSummaries.ieList);
    std::vector<VarCreate> varCreates = helpers.deserializeVarCreateList(informationElementCreates.ieList);
    std::vector<VarUpd> varUpdates = helpers.deserializeVarUpdList(informationElementUpdates.ieList);
    std::vector<VarId> varDeletes = helpers.deserializeVarIdList(informationElementDeletes.ieList);
    std::vector<VarSumm> varRequestUpdates = helpers.deserializeVarSummList(informationElementRequestUpdates.ieList);
    std::vector<VarId> varRequestCreates = helpers.deserializeVarIdList(informationElementRequestCreates.ieList);

    std::cout << "-------------Deserialized VarSumms-------------" << std::endl;
    for (VarSumm varSumm: varSumms) {
        std::cout << varSumm << std::endl;
    }
    std::cout << "-------------Deserialized varCreates-------------" << std::endl;
    for (VarCreate varCreate: varCreates) {
        std::cout << varCreate << std::endl;
    }
    std::cout << "-------------Deserialized varUpdates-------------" << std::endl;
    for (VarUpd varUpd: varUpdates) {
        std::cout << varUpd << std::endl;
    }
    std::cout << "-------------Deserialized varDeletes-------------" << std::endl;
    for (VarId varId: varDeletes) {
        std::cout << varId << std::endl;
    }
    std::cout << "-------------Deserialized varRequestUpdates-------------" << std::endl;
    for (VarSumm varSumm: varRequestUpdates) {
        std::cout << varSumm << std::endl;
    }
    std::cout << "-------------Deserialized varRequestCreates-------------" << std::endl;
    for (VarId varId: varRequestCreates) {
        std::cout << varId << std::endl;
    }

    std::cout << "-------------Serialize informationElementSummaries-------------" << std::endl;
    std::cout << informationElementSummaries <<std::endl;
    std::vector<uint8_t> serializedInformationElementSummaries;
    helpers.serializeInformationElement(serializedInformationElementSummaries, informationElementSummaries);

    for (const auto& byte : serializedInformationElementSummaries) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    std::cout << std::endl;
    std::cout << std::dec; // Reset to decimal format

    size_t offset = 0;
    InformationElement deserializedInformationElementSummaries = helpers.deserializeInformationElement(serializedInformationElementSummaries, offset);
    std::cout << deserializedInformationElementSummaries <<std::endl;

    std::cout << "-------------Serialize informationElementCreates-------------" << std::endl;
    std::cout << informationElementCreates <<std::endl;
    std::vector<uint8_t> serializedInformationElementCreates;
    helpers.serializeInformationElement(serializedInformationElementCreates, informationElementCreates);

    for (const auto& byte : serializedInformationElementCreates) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    std::cout << std::endl;
    std::cout << std::dec; // Reset to decimal format

    offset = 0;
    InformationElement deserializedInformationElementCreates = helpers.deserializeInformationElement(serializedInformationElementCreates, offset);
    std::cout << deserializedInformationElementCreates <<std::endl;

    std::cout << "-------------Serialize informationElementUpdates-------------" << std::endl;
    std::cout << informationElementUpdates <<std::endl;
    std::vector<uint8_t> serializedInformationElementUpdates;
    helpers.serializeInformationElement(serializedInformationElementUpdates, informationElementUpdates);

    for (const auto& byte : serializedInformationElementUpdates) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    std::cout << std::endl;
    std::cout << std::dec; // Reset to decimal format

    offset = 0;
    InformationElement deserializedInformationElementUpdates = helpers.deserializeInformationElement(serializedInformationElementUpdates, offset);
    std::cout << deserializedInformationElementUpdates <<std::endl;

    std::cout << "-------------Serialize informationElementDeletes-------------" << std::endl;
    std::cout << informationElementDeletes <<std::endl;
    std::vector<uint8_t> serializedInformationElementDeletes;
    helpers.serializeInformationElement(serializedInformationElementDeletes, informationElementDeletes);

    for (const auto& byte : serializedInformationElementDeletes) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    std::cout << std::endl;
    std::cout << std::dec; // Reset to decimal format

    offset = 0;
    InformationElement deserializedInformationElementDeletes = helpers.deserializeInformationElement(serializedInformationElementDeletes, offset);
    std::cout << deserializedInformationElementDeletes <<std::endl;

    std::cout << "-------------Serialize informationElementRequestUpdates-------------" << std::endl;
    std::cout << informationElementRequestUpdates <<std::endl;
    std::vector<uint8_t> serializedInformationElementRequestUpdates;
    helpers.serializeInformationElement(serializedInformationElementRequestUpdates, informationElementRequestUpdates);

    for (const auto& byte : serializedInformationElementRequestUpdates) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    std::cout << std::endl;
    std::cout << std::dec; // Reset to decimal format

    offset = 0;
    InformationElement deserializedInformationElementRequestUpdates = helpers.deserializeInformationElement(serializedInformationElementRequestUpdates, offset);
    std::cout << deserializedInformationElementRequestUpdates <<std::endl;

    std::cout << "-------------Serialize informationElementRequestCreates-------------" << std::endl;
    std::cout << informationElementRequestCreates <<std::endl;
    std::vector<uint8_t> serializedInformationElementRequestCreates;
    helpers.serializeInformationElement(serializedInformationElementRequestCreates, informationElementRequestCreates);

    for (const auto& byte : serializedInformationElementRequestCreates) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    std::cout << std::endl;
    std::cout << std::dec; // Reset to decimal format

    offset = 0;
    InformationElement deserializedInformationElementRequestCreates = helpers.deserializeInformationElement(serializedInformationElementRequestCreates, offset);
    std::cout << deserializedInformationElementRequestCreates <<std::endl;
}

/**
 * @brief Checks that serializing the VarDis payload is working
 */
void serializingVarDisPayload() {
    VarDisClient varDisClient = VarDisClient(); // Create custom queues and real-time database
    DatabaseManager& databaseManager = varDisClient.getDatabaseManager(); // Get database instance
    Helpers helpers = Helpers(); // Helper functions

    // Create a "list" of uint8_t
    std::vector<uint8_t> value;
    // Add some bytes to the "list"
    value.push_back(static_cast<uint8_t>(0x41)); // Example: adding byte 'A'
    value.push_back(static_cast<uint8_t>(0x42)); // Example: adding byte 'B'
    value.push_back(static_cast<uint8_t>(0x43)); // Example: adding byte 'C'

    VarSpec spec1(databaseManager.getNextAvailableVarId(),
                  helpers.getNodeIdentifier(),
                  2,
                  "Variable1");

    databaseManager.create(spec1, value.size(), value);

    VarSpec spec2(databaseManager.getNextAvailableVarId(),
                  helpers.getNodeIdentifier(),
                  2,
                  "Variable2");

    // Create a Database entry
    databaseManager.create(spec2, value.size(), value);

    varDisClient.updateQ.qAppend(1);
    varDisClient.updateQ.qAppend(2);
    varDisClient.deleteQ.qAppend(1);
    varDisClient.deleteQ.qAppend(2);
    varDisClient.reqUpdQ.qAppend(1);
    varDisClient.reqUpdQ.qAppend(2);
    varDisClient.reqCreateQ.qAppend(1);
    varDisClient.reqCreateQ.qAppend(2);

    std::cout << "--------- Checking 2 variables were added to database ---------" << std::endl;
    std::vector<DBEntry> entries = databaseManager.getAllDBEntries();
    databaseManager.printResults(entries);
    std::cout << std::endl;

    varDisClient.summaryQ.printQueue();
    varDisClient.createQ.printQueue();
    varDisClient.updateQ.printQueue();
    varDisClient.deleteQ.printQueue();
    varDisClient.reqUpdQ.printQueue();
    varDisClient.reqCreateQ.printQueue();

    size_t currentPayloadSize = 0;
    InformationElement informationElementSummaries = varDisClient.composeSummaryIEElements(&currentPayloadSize);
    InformationElement informationElementCreates = varDisClient.composeCreateVariablesIEElements(&currentPayloadSize);
    InformationElement informationElementUpdates = varDisClient.composeUpdateIEElements(&currentPayloadSize);
    InformationElement informationElementDeletes = varDisClient.composeDeleteVariablesIEElements(&currentPayloadSize);
    InformationElement informationElementRequestUpdates = varDisClient.composeRequestVarUpdatesIEElements(&currentPayloadSize);
    InformationElement informationElementRequestCreates = varDisClient.composeRequestVarCreatesIEElements(&currentPayloadSize);

    std::cout << "Summaries: " << informationElementSummaries.ieHeader.ieNumRecords << std::endl;
    std::cout << "Creates: " << informationElementCreates.ieHeader.ieNumRecords << std::endl;
    std::cout << "Updates: " << informationElementUpdates.ieHeader.ieNumRecords << std::endl;
    std::cout << "Deletes: " << informationElementDeletes.ieHeader.ieNumRecords << std::endl;
    std::cout << "RequestUpdates: " << informationElementRequestUpdates.ieHeader.ieNumRecords << std::endl;
    std::cout << "RequestCreates: " << informationElementRequestCreates.ieHeader.ieNumRecords << std::endl;

    std::cout << "-------------Serialize informationElementSummaries-------------" << std::endl;
    std::vector<uint8_t> serializedInformationElements;

    helpers.serializeInformationElement(serializedInformationElements, informationElementSummaries);
    helpers.serializeInformationElement(serializedInformationElements, informationElementCreates);
    helpers.serializeInformationElement(serializedInformationElements, informationElementUpdates);
    helpers.serializeInformationElement(serializedInformationElements, informationElementDeletes);
    helpers.serializeInformationElement(serializedInformationElements, informationElementRequestUpdates);
    helpers.serializeInformationElement(serializedInformationElements, informationElementRequestCreates);

    std::cout << "Elements size: " << serializedInformationElements.size() << std::endl;

    for (const auto& byte : serializedInformationElements) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    std::cout << std::endl;
    std::cout << std::dec; // Reset to decimal format

    size_t offset = 0;
    InformationElement deserializedInformationElementSummaries = helpers.deserializeInformationElement(serializedInformationElements, offset);
    std::cout << "One: " << offset << std::endl;
    InformationElement deserializedInformationElementCreates = helpers.deserializeInformationElement(serializedInformationElements, offset);
    std::cout << "Two: " << offset << std::endl;
    InformationElement deserializedInformationElementUpdates = helpers.deserializeInformationElement(serializedInformationElements, offset);
    std::cout << "Three: " << offset << std::endl;
    InformationElement deserializedInformationElementDeletes = helpers.deserializeInformationElement(serializedInformationElements, offset);
    std::cout << "Four: " << offset << std::endl;
    InformationElement deserializedInformationElementRequestUpdates = helpers.deserializeInformationElement(serializedInformationElements, offset);
    std::cout << "Five: " << offset << std::endl;
    InformationElement deserializedInformationElementRequestCreates = helpers.deserializeInformationElement(serializedInformationElements, offset);
    std::cout << "Size: " << offset << std::endl;

    std::cout << deserializedInformationElementSummaries <<std::endl;
    std::cout << deserializedInformationElementCreates <<std::endl;
    std::cout << deserializedInformationElementUpdates <<std::endl;
    std::cout << deserializedInformationElementDeletes <<std::endl;
    std::cout << deserializedInformationElementRequestUpdates <<std::endl;
    std::cout << deserializedInformationElementRequestCreates <<std::endl;

    varDisClient.summaryQ.printQueue();
    varDisClient.createQ.printQueue();
    varDisClient.updateQ.printQueue();
    varDisClient.deleteQ.printQueue();
    varDisClient.reqUpdQ.printQueue();
    varDisClient.reqCreateQ.printQueue();

    varDisClient.processCreateIEElements(deserializedInformationElementCreates.ieList);
    varDisClient.processDeleteIEElements(deserializedInformationElementDeletes.ieList);
    varDisClient.processUpdateIEElements(deserializedInformationElementUpdates.ieList);
    varDisClient.processSummaryIEElements(deserializedInformationElementSummaries.ieList);
    varDisClient.processRequestVarUpdsIEElements(informationElementRequestUpdates.ieList);
    varDisClient.processRequestVarCreatesIEElements(informationElementRequestCreates.ieList);

    varDisClient.summaryQ.printQueue();
    varDisClient.createQ.printQueue();
    varDisClient.updateQ.printQueue();
    varDisClient.deleteQ.printQueue();
    varDisClient.reqUpdQ.printQueue();
    varDisClient.reqCreateQ.printQueue();

    currentPayloadSize = 0;
    informationElementSummaries = varDisClient.composeSummaryIEElements(&currentPayloadSize);
    informationElementCreates = varDisClient.composeCreateVariablesIEElements(&currentPayloadSize);
    informationElementUpdates = varDisClient.composeUpdateIEElements(&currentPayloadSize);
    informationElementDeletes = varDisClient.composeDeleteVariablesIEElements(&currentPayloadSize);
    informationElementRequestUpdates = varDisClient.composeRequestVarUpdatesIEElements(&currentPayloadSize);
    informationElementRequestCreates = varDisClient.composeRequestVarCreatesIEElements(&currentPayloadSize);

    std::cout << "Summaries: " << informationElementSummaries.ieHeader.ieNumRecords << std::endl;
    std::cout << "Creates: " << informationElementCreates.ieHeader.ieNumRecords << std::endl;
    std::cout << "Updates: " << informationElementUpdates.ieHeader.ieNumRecords << std::endl;
    std::cout << "Deletes: " << informationElementDeletes.ieHeader.ieNumRecords << std::endl;
    std::cout << "RequestUpdates: " << informationElementRequestUpdates.ieHeader.ieNumRecords << std::endl;
    std::cout << "RequestCreates: " << informationElementRequestCreates.ieHeader.ieNumRecords << std::endl;

    std::cout << "-------------Serialize informationElementSummaries-------------" << std::endl;
    serializedInformationElements = {};

    helpers.serializeInformationElement(serializedInformationElements, informationElementSummaries);
    helpers.serializeInformationElement(serializedInformationElements, informationElementCreates);
    helpers.serializeInformationElement(serializedInformationElements, informationElementUpdates);
    helpers.serializeInformationElement(serializedInformationElements, informationElementDeletes);
    helpers.serializeInformationElement(serializedInformationElements, informationElementRequestUpdates);
    helpers.serializeInformationElement(serializedInformationElements, informationElementRequestCreates);

    std::cout << "Elements size: " << serializedInformationElements.size() << std::endl;

    for (const auto& byte : serializedInformationElements) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    std::cout << std::endl;
    std::cout << std::dec; // Reset to decimal format

    offset = 0;
    deserializedInformationElementSummaries = helpers.deserializeInformationElement(serializedInformationElements, offset);
    std::cout << "One: " << offset << std::endl;
    deserializedInformationElementCreates = helpers.deserializeInformationElement(serializedInformationElements, offset);
    std::cout << "Two: " << offset << std::endl;
    deserializedInformationElementUpdates = helpers.deserializeInformationElement(serializedInformationElements, offset);
    std::cout << "Three: " << offset << std::endl;
    deserializedInformationElementDeletes = helpers.deserializeInformationElement(serializedInformationElements, offset);
    std::cout << "Four: " << offset << std::endl;
    deserializedInformationElementRequestUpdates = helpers.deserializeInformationElement(serializedInformationElements, offset);
    std::cout << "Five: " << offset << std::endl;
    deserializedInformationElementRequestCreates = helpers.deserializeInformationElement(serializedInformationElements, offset);
    std::cout << "Size: " << offset << std::endl;

    std::cout << deserializedInformationElementSummaries <<std::endl;
    std::cout << deserializedInformationElementCreates <<std::endl;
    std::cout << deserializedInformationElementUpdates <<std::endl;
    std::cout << deserializedInformationElementDeletes <<std::endl;
    std::cout << deserializedInformationElementRequestUpdates <<std::endl;
    std::cout << deserializedInformationElementRequestCreates <<std::endl;

    varDisClient.summaryQ.printQueue();
    varDisClient.createQ.printQueue();
    varDisClient.updateQ.printQueue();
    varDisClient.deleteQ.printQueue();
    varDisClient.reqUpdQ.printQueue();
    varDisClient.reqCreateQ.printQueue();

    varDisClient.processCreateIEElements(deserializedInformationElementCreates.ieList);
    varDisClient.processDeleteIEElements(deserializedInformationElementDeletes.ieList);
    varDisClient.processUpdateIEElements(deserializedInformationElementUpdates.ieList);
    varDisClient.processSummaryIEElements(deserializedInformationElementSummaries.ieList);
    varDisClient.processRequestVarUpdsIEElements(informationElementRequestUpdates.ieList);
    varDisClient.processRequestVarCreatesIEElements(informationElementRequestCreates.ieList);

    varDisClient.summaryQ.printQueue();
    varDisClient.createQ.printQueue();
    varDisClient.updateQ.printQueue();
    varDisClient.deleteQ.printQueue();
    varDisClient.reqUpdQ.printQueue();
    varDisClient.reqCreateQ.printQueue();
}

int main() {
    normalChecking();
//    serializingVarDisPayload();
//    serializeChecking();
//    addingIEElements();
}