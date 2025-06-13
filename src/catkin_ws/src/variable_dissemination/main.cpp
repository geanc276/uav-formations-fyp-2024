#include "headers/DatabaseManager.h"
#include "headers/Helpers.h"
#include "headers/VarDisProtocol.h"

/**
 * @brief Used to test implementation during development. On the terminal run the below commands.
 * The 1st command deletes the existing database (if there is one) and compiles the binary.
 * The 2nd command runs the binary
 * ./compile.sh
 * ./binary
 */
int normalChecking() {
    VarDisProtocol varDisProtocol = VarDisProtocol(); // Create custom queues and real-time database
    DatabaseManager& databaseManager = varDisProtocol.getDatabaseManager(); // Get database instance

    Helpers helpers = Helpers(); // Helper functions

    // Create a "list" of std::byte
    std::vector<std::byte> value;
    // Add some bytes to the "list"
    value.push_back(static_cast<std::byte>(0x41)); // Example: adding byte 'A'
    value.push_back(static_cast<std::byte>(0x42)); // Example: adding byte 'B'
    value.push_back(static_cast<std::byte>(0x43)); // Example: adding byte 'C'

    VarSpec spec(databaseManager.getNextAvailableVarId(),
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
    DBEntry entry = databaseManager.lookup(1);
    std::cout << entry << std::endl;
    std::cout << std::endl;

    std::cout << "--------- Checking variable was updated in database ---------" << std::endl;
    value.pop_back(); // Remove last item from the "list"
    databaseManager.updateDBEntryById(1, value.size(), value); // Update "list" in database
    DBEntry updated_entry = databaseManager.lookup(1);
    std::cout << updated_entry << std::endl;
    std::cout << std::endl;

    std::cout << "--------- Checking variable was prepared for deletion ---------" << std::endl;
    // Prepare to delete from database, only changes toBeDeleted flag to 1/true
    databaseManager.deleteDBEntryById(1);
    // Check the update by getting all entries (again just 1).
    entries = databaseManager.getAllDBEntries();
    databaseManager.printResults(entries);
    std::cout << std::endl;

    varDisProtocol.createQ.qAppend(1);
    varDisProtocol.createQ.qAppend(2);
    varDisProtocol.createQ.qAppend(3);
    varDisProtocol.createQ.qAppend(4);

    std::cout << "--------- Checking custom queue is populated ---------" << std::endl;
    varDisProtocol.createQ.printQueue();
    std::cout << std::endl;

    std::cout << "--------- Checking queue items are removed ---------" << std::endl;
    varDisProtocol.createQ.qDropNonexisting();
    varDisProtocol.createQ.printQueue();
    std::cout << std::endl;

    std::cout << "--------- Checking queue items are removed pt. 2 ---------" << std::endl;
    varDisProtocol.createQ.qDropNonexistingDeleted();
    varDisProtocol.createQ.printQueue();

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

void serializeChecking() {
    VarDisProtocol varDisProtocol = VarDisProtocol(); // Create custom queues and real-time database
    DatabaseManager& databaseManager = varDisProtocol.getDatabaseManager(); // Get database instance
    Helpers helpers = Helpers(); // Helper functions

    // Create a "list" of std::byte
    std::vector<std::byte> value;
    // Add some bytes to the "list"
    value.push_back(static_cast<std::byte>(0x41)); // Example: adding byte 'A'
    value.push_back(static_cast<std::byte>(0x42)); // Example: adding byte 'B'
    value.push_back(static_cast<std::byte>(0x43)); // Example: adding byte 'C'

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

    std::vector<std::byte> serializedData = helpers.serializeVarCreateList(varCreateList);
    for (const auto& byte : serializedData) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    std::cout << std::endl;

    std::vector<VarCreate> deserializedVarCreate = helpers.deserializeVarCreateList(serializedData);

    std::cout << "-------------Deserialized VarCreate-------------" << std::endl;
    databaseManager.printResults(varCreateList);


}

void addingIEElements() {
    VarDisProtocol varDisProtocol = VarDisProtocol(); // Create custom queues and real-time database
    DatabaseManager& databaseManager = varDisProtocol.getDatabaseManager(); // Get database instance
    Helpers helpers = Helpers(); // Helper functions

    // Create a "list" of std::byte
    std::vector<std::byte> value;
    // Add some bytes to the "list"
    value.push_back(static_cast<std::byte>(0x41)); // Example: adding byte 'A'
    value.push_back(static_cast<std::byte>(0x42)); // Example: adding byte 'B'
    value.push_back(static_cast<std::byte>(0x43)); // Example: adding byte 'C'

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

    varDisProtocol.updateQ.qAppend(1);
    varDisProtocol.updateQ.qAppend(2);
    varDisProtocol.deleteQ.qAppend(1);
    varDisProtocol.deleteQ.qAppend(2);
    varDisProtocol.reqUpdQ.qAppend(1);
    varDisProtocol.reqUpdQ.qAppend(2);
    varDisProtocol.reqCreateQ.qAppend(1);
    varDisProtocol.reqCreateQ.qAppend(2);

    std::cout << "--------- Checking 2 variables were added to database ---------" << std::endl;
    std::vector<DBEntry> entries = databaseManager.getAllDBEntries();
    databaseManager.printResults(entries);
    std::cout << std::endl;

    size_t currentPayloadSize = 0;
    InformationElement informationElementSummaries = varDisProtocol.composeSummaryIEElements(&currentPayloadSize);
    InformationElement informationElementCreates = varDisProtocol.composeCreateVariablesIEElements(&currentPayloadSize);
    InformationElement informationElementUpdates = varDisProtocol.composeUpdateIEElements(&currentPayloadSize);
    InformationElement informationElementDeletes = varDisProtocol.composeDeleteVariablesIEElements(&currentPayloadSize);
    InformationElement informationElementRequestUpdates = varDisProtocol.composeRequestVarUpdatesIEElements(&currentPayloadSize);
    InformationElement informationElementRequestCreates = varDisProtocol.composeRequestVarCreatesIEElements(&currentPayloadSize);

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
    std::vector<std::byte> serializedInformationElementSummaries;
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
    std::vector<std::byte> serializedInformationElementCreates;
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
    std::vector<std::byte> serializedInformationElementUpdates;
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
    std::vector<std::byte> serializedInformationElementDeletes;
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
    std::vector<std::byte> serializedInformationElementRequestUpdates;
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
    std::vector<std::byte> serializedInformationElementRequestCreates;
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

void serializingVarDisPayload() {
    VarDisProtocol varDisProtocol = VarDisProtocol(); // Create custom queues and real-time database
    DatabaseManager& databaseManager = varDisProtocol.getDatabaseManager(); // Get database instance
    Helpers helpers = Helpers(); // Helper functions

    // Create a "list" of std::byte
    std::vector<std::byte> value;
    // Add some bytes to the "list"
    value.push_back(static_cast<std::byte>(0x41)); // Example: adding byte 'A'
    value.push_back(static_cast<std::byte>(0x42)); // Example: adding byte 'B'
    value.push_back(static_cast<std::byte>(0x43)); // Example: adding byte 'C'

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

    varDisProtocol.updateQ.qAppend(1);
    varDisProtocol.updateQ.qAppend(2);
    varDisProtocol.deleteQ.qAppend(1);
    varDisProtocol.deleteQ.qAppend(2);
    varDisProtocol.reqUpdQ.qAppend(1);
    varDisProtocol.reqUpdQ.qAppend(2);
    varDisProtocol.reqCreateQ.qAppend(1);
    varDisProtocol.reqCreateQ.qAppend(2);

    std::cout << "--------- Checking 2 variables were added to database ---------" << std::endl;
    std::vector<DBEntry> entries = databaseManager.getAllDBEntries();
    databaseManager.printResults(entries);
    std::cout << std::endl;

    size_t currentPayloadSize = 0;
    InformationElement informationElementSummaries = varDisProtocol.composeSummaryIEElements(&currentPayloadSize);
    InformationElement informationElementCreates = varDisProtocol.composeCreateVariablesIEElements(&currentPayloadSize);
    InformationElement informationElementUpdates = varDisProtocol.composeUpdateIEElements(&currentPayloadSize);
    InformationElement informationElementDeletes = varDisProtocol.composeDeleteVariablesIEElements(&currentPayloadSize);
    InformationElement informationElementRequestUpdates = varDisProtocol.composeRequestVarUpdatesIEElements(&currentPayloadSize);
    InformationElement informationElementRequestCreates = varDisProtocol.composeRequestVarCreatesIEElements(&currentPayloadSize);

    std::cout << "-------------Serialize informationElementSummaries-------------" << std::endl;
    std::vector<std::byte> serializedInformationElements;

    helpers.serializeInformationElement(serializedInformationElements, informationElementSummaries);
    helpers.serializeInformationElement(serializedInformationElements, informationElementCreates);
    helpers.serializeInformationElement(serializedInformationElements, informationElementUpdates);
    helpers.serializeInformationElement(serializedInformationElements, informationElementDeletes);
    helpers.serializeInformationElement(serializedInformationElements, informationElementRequestUpdates);
    helpers.serializeInformationElement(serializedInformationElements, informationElementRequestCreates);

    for (const auto& byte : serializedInformationElements) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    std::cout << std::endl;
    std::cout << std::dec; // Reset to decimal format

    size_t offset = 0;
    InformationElement deserializedInformationElementSummaries = helpers.deserializeInformationElement(serializedInformationElements, offset);
    InformationElement deserializedInformationElementCreates = helpers.deserializeInformationElement(serializedInformationElements, offset);
    InformationElement deserializedInformationElementUpdates = helpers.deserializeInformationElement(serializedInformationElements, offset);
    InformationElement deserializedInformationElementDeletes = helpers.deserializeInformationElement(serializedInformationElements, offset);
    InformationElement deserializedInformationElementRequestUpdates = helpers.deserializeInformationElement(serializedInformationElements, offset);
    InformationElement deserializedInformationElementRequestCreates = helpers.deserializeInformationElement(serializedInformationElements, offset);

    std::cout << deserializedInformationElementSummaries <<std::endl;
    std::cout << deserializedInformationElementCreates <<std::endl;
    std::cout << deserializedInformationElementUpdates <<std::endl;
    std::cout << deserializedInformationElementDeletes <<std::endl;
    std::cout << deserializedInformationElementRequestUpdates <<std::endl;
    std::cout << deserializedInformationElementRequestCreates <<std::endl;
}

int main() {
//    mapChecking();
    serializingVarDisPayload();
//    serializeChecking();
//    addingIEElements();
//    normalChecking();

}