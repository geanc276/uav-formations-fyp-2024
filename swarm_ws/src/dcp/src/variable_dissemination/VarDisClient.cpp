#include "dcp/VarDisClient.h"


VarDisClient::VarDisClient() {

    // Set the database manager for each VarDisQueue
    createQ.setDatabaseManager(&databaseManager);
    deleteQ.setDatabaseManager(&databaseManager);
    updateQ.setDatabaseManager(&databaseManager);
    summaryQ.setDatabaseManager(&databaseManager);
    reqUpdQ.setDatabaseManager(&databaseManager);
    reqCreateQ.setDatabaseManager(&databaseManager);

    // Set the queues in the DatabaseManager
    databaseManager.setQueues(&createQ, &deleteQ, &updateQ, &summaryQ, &reqUpdQ, &reqCreateQ);
}

 DatabaseManager& VarDisClient::getDatabaseManager() {
    return databaseManager;
}

VarDisClient::VarDisClient(rclcpp::Publisher<dcp_msgs::msg::RegisterProtocolRequest>::SharedPtr pubRegister,
                           rclcpp::Publisher<dcp_msgs::msg::TransmitPayloadRequest>::SharedPtr transmitRequest,
                           rclcpp::Publisher<dcp_msgs::msg::QueryNumberBufferedPayloadsRequest>::SharedPtr checkQueueBuffer) {

    protocolRegisterPub = pubRegister;
    protocolTransmitPayload = transmitRequest;
    protocolCheckQueueBuffer = checkQueueBuffer;

    // Set the database manager for each VarDisQueue
    createQ.setDatabaseManager(&databaseManager);
    deleteQ.setDatabaseManager(&databaseManager);
    updateQ.setDatabaseManager(&databaseManager);
    summaryQ.setDatabaseManager(&databaseManager);
    reqUpdQ.setDatabaseManager(&databaseManager);
    reqCreateQ.setDatabaseManager(&databaseManager);

    // Set the queues in the DatabaseManager
    databaseManager.setQueues(&createQ, &deleteQ, &updateQ, &summaryQ, &reqUpdQ, &reqCreateQ);

}

void VarDisClient::processCreateIEElements(std::vector<uint8_t> rawIEByteVector) {

    std::vector<VarCreate> deserializedVarCreateList = helpers.deserializeVarCreateList(rawIEByteVector);

    for(VarCreate rcvdVC: deserializedVarCreateList) {
        VarSpec spec = rcvdVC.spec;
        VarUpd upd = rcvdVC.upd;
        VarId varId = spec.varId;

        if (databaseManager.lookup(varId).existsInDB) {
            continue;
        }

        if (spec.prodId == helpers.getNodeIdentifier()) {
            continue;
        }

        TimeStamp timeStamp;

        std::cout << "Added via process: " << spec << std::endl;
        DBEntry dbEntry = {
                .spec = spec,
                .length = upd.varLen,
                .value = upd.Value,
                .seqno = upd.varSeqNo,
                .tStamp = timeStamp.time.count(),
                .countUpdate = 0,
                .countCreate = spec.repCnt,
                .countDelete = 0,
                .toBeDeleted = false,
                .existsInDB = true
        };

        databaseManager.update(dbEntry);
        createQ.qAppend(varId);
        summaryQ.qAppend(varId);
        reqCreateQ.qRemove(varId);
    }
}

void VarDisClient::processDeleteIEElements(std::vector<uint8_t> rawIEByteVector) {

    std::vector<VarId> varIds = helpers.deserializeVarIdList(rawIEByteVector);

    for(VarId varId: varIds) {
        DBEntry dbEntry = databaseManager.lookup(varId);

        if (not dbEntry.existsInDB) {
            continue;
        }

        if (dbEntry.toBeDeleted) {
            continue;
        }

        if (dbEntry.spec.prodId == helpers.getNodeIdentifier()) {
            continue;
        }

        dbEntry.toBeDeleted = true;
        dbEntry.countUpdate = 0;
        dbEntry.countCreate = 0;
        dbEntry.countDelete = dbEntry.spec.repCnt;
        databaseManager.update(dbEntry);

        updateQ.qRemove(varId);
        createQ.qRemove(varId);
        reqUpdQ.qRemove(varId);
        reqCreateQ.qRemove(varId);
        summaryQ.qRemove(varId);
        deleteQ.qRemove(varId);
        deleteQ.qAppend(varId);
    }
}

void VarDisClient::processUpdateIEElements(std::vector<uint8_t> rawIEByteVector) {

    std::vector<VarUpd> deserializeVarUpdList = helpers.deserializeVarUpdList(rawIEByteVector);

    for(VarUpd upd: deserializeVarUpdList) {
        DBEntry dbEntry = databaseManager.lookup(upd.varId);
        if (not dbEntry.existsInDB) {
            if (!reqCreateQ.qExists(upd.varId)) {
                reqCreateQ.qAppend(upd.varId);
                continue;
            }
        }
        if (dbEntry.toBeDeleted) {
            continue;
        }
        if (dbEntry.spec.prodId == helpers.getNodeIdentifier()) {
            continue;
        }
        if (upd.varSeqNo == dbEntry.seqno) {
            continue;
        }

        if (upd.varSeqNo < dbEntry.seqno) {
            if (!updateQ.qExists(upd.varId)) {
                updateQ.qAppend(upd.varId);
                dbEntry.countUpdate = dbEntry.spec.repCnt;
                databaseManager.update(dbEntry);
                continue;
            }
        }

        TimeStamp timeStamp;
        dbEntry.length = upd.varLen;
        dbEntry.value = upd.Value;
        dbEntry.seqno = upd.varSeqNo;
        dbEntry.tStamp = timeStamp.time.count();
        dbEntry.countUpdate = dbEntry.spec.repCnt;
        databaseManager.update(dbEntry);

        if (!updateQ.qExists(upd.varId)) {
            updateQ.qAppend(upd.varId);
        }
        reqUpdQ.qRemove(upd.varId);
    }
}

void VarDisClient::processSummaryIEElements(std::vector<uint8_t> rawIEByteVector) {

    std::vector<VarSumm> deserializeVarSummList = helpers.deserializeVarSummList(rawIEByteVector);

    for (VarSumm summ: deserializeVarSummList) {
        DBEntry dbEntry = databaseManager.lookup(summ.varId);
        if (not dbEntry.existsInDB) {
            if (!reqCreateQ.qExists(summ.varId)) {
                reqCreateQ.qAppend(summ.varId);
            }
            continue;
        }
        if (dbEntry.toBeDeleted) {
            continue;
        }
        if (dbEntry.spec.prodId == helpers.getNodeIdentifier()) {
            continue;
        }
        if (summ.varSeqNo == dbEntry.seqno) {
            continue;
        }

        if (summ.varSeqNo < dbEntry.seqno) {
            if (!updateQ.qExists(summ.varId)) {
                updateQ.qAppend(summ.varId);
                dbEntry.countUpdate = dbEntry.spec.repCnt;
                databaseManager.update(dbEntry);
                continue;
            }
        }
        if (!reqUpdQ.qExists(summ.varId)) {
            reqUpdQ.qAppend(summ.varId);
        }
    }
}

void VarDisClient::processRequestVarUpdsIEElements(std::vector<uint8_t> rawIEByteVector) {

    std::vector<VarSumm> deserializeVarSummList = helpers.deserializeVarSummList(rawIEByteVector);

    for (VarSumm summ: deserializeVarSummList) {
        DBEntry dbEntry = databaseManager.lookup(summ.varId);
        if (not dbEntry.existsInDB) {
            continue;
        }
        if (dbEntry.toBeDeleted) {
            continue;
        }
        if (dbEntry.seqno < summ.varSeqNo) {
        }
        dbEntry.countUpdate = dbEntry.spec.repCnt;
        databaseManager.update(dbEntry);
        if (!updateQ.qExists(summ.varId)) {
            updateQ.qAppend(summ.varId);
        }
    }
 }

void VarDisClient::processRequestVarCreatesIEElements(std::vector<uint8_t> rawIEByteVector) {

    std::vector<VarId> varIds = helpers.deserializeVarIdList(rawIEByteVector);

    for (VarId varId: varIds) {
        DBEntry dbEntry = databaseManager.lookup(varId);
        if (not dbEntry.existsInDB) {
            continue;
        }
        if (dbEntry.toBeDeleted) {
            continue;
        }
        dbEntry.countCreate = dbEntry.spec.repCnt;
        databaseManager.update(dbEntry);
        if (!createQ.qExists(varId)) {
            createQ.qAppend(varId);
        }
    }
}

InformationElement VarDisClient::composeSummaryIEElements(size_t* currentPayloadSize) {
    TimeStamp time;
    IEHeader ieHeader = {
            .ieType = IETYPE_SUMMARIES,
            .ieNumRecords = 0,
            .ieTimestamp = time,
            .ieSeqNum = varSeqNum,
    };
    InformationElement informationElement = {
            .ieHeader = ieHeader,
    };

    summaryQ.qDropNonexistingDeleted();
    if (    summaryQ.qIsEmpty()
         || VARDISPAR_MAX_PAYLOAD_SIZE - *currentPayloadSize < (sizeof(VarSumm) + sizeof(IEHeader))
         || VARDISPAR_MAX_SUMMARIES == 0) {
        return informationElement;
    }

    uint8_t numRecordsToAdd = std::min(numberFittingRecords(summaryQ, sizeof(VarSumm), currentPayloadSize), static_cast<uint8_t>(VARDISPAR_MAX_SUMMARIES));
    informationElement.ieHeader.ieNumRecords = numRecordsToAdd;
    for (uint8_t i = 0; i < numRecordsToAdd; i++) {
        VarId nextVarId = summaryQ.qTake();
        summaryQ.qAppend(nextVarId);
        DBEntry newDbEntry = databaseManager.lookup(nextVarId);
        helpers.addVarSummToIEElement(&informationElement, &newDbEntry, currentPayloadSize);
    }
    varSeqNum++;
    return informationElement;
}

InformationElement VarDisClient::composeCreateVariablesIEElements(size_t* currentPayloadSize) {
    TimeStamp time;
    IEHeader ieHeader = {
            .ieType = IETYPE_CREATE_VARIABLES,
            .ieNumRecords = 0,
            .ieTimestamp = time,
            .ieSeqNum = varSeqNum,
    };
    InformationElement informationElement = {
            .ieHeader = ieHeader,
    };
    createQ.qDropNonexistingDeleted();

    if (createQ.qIsEmpty() || VARDISPAR_MAX_PAYLOAD_SIZE - *currentPayloadSize < (sizeof(VarCreate) + sizeof(IEHeader))) {
        return informationElement;
    }

    uint8_t numRecordsToAdd = numberFittingRecords(createQ, sizeof(VarCreate), currentPayloadSize);
    informationElement.ieHeader.ieNumRecords = numRecordsToAdd;

    for (uint8_t i = 0; i < numRecordsToAdd; i++) {
        VarId nextVarId = createQ.qPeek();
        DBEntry nextVar = databaseManager.lookup(nextVarId);
        if (nextVar.countCreate == 0) {
            continue;
        }
        createQ.qTake();
        nextVar.countCreate -= 1;
        databaseManager.update(nextVar);
        helpers.addVarCreateToIEElement(&informationElement, &nextVar, currentPayloadSize);
        if (nextVar.countCreate > 0) {
            createQ.qAppend(nextVarId);
        }
    }
    varSeqNum++;
    return informationElement;
}

InformationElement VarDisClient::composeUpdateIEElements(size_t* currentPayloadSize) {
    TimeStamp time;
    IEHeader ieHeader = {
            .ieType = IETYPE_UPDATES,
            .ieNumRecords = 0,
            .ieTimestamp = time,
            .ieSeqNum = varSeqNum,
    };
    InformationElement informationElement = {
            .ieHeader = ieHeader,
    };
    updateQ.qDropNonexistingDeleted();

    if (updateQ.qIsEmpty() || VARDISPAR_MAX_PAYLOAD_SIZE - *currentPayloadSize < (sizeof(VarUpd) + sizeof(IEHeader))) {
        return informationElement;
    }

    uint8_t numRecordsToAdd = numberFittingRecords(updateQ, sizeof(VarUpd), currentPayloadSize);
    informationElement.ieHeader.ieNumRecords = numRecordsToAdd;

    for (uint8_t i = 0; i < numRecordsToAdd; i++) {
        VarId nextVarId = updateQ.qPeek();
        DBEntry nextVar = databaseManager.lookup(nextVarId);
        if (nextVar.countUpdate == 0) {
            continue;
        }
        updateQ.qTake();
        nextVar.countUpdate -= 1;
        databaseManager.update(nextVar);
        helpers.addVarUpdateToIEElement(&informationElement, &nextVar, currentPayloadSize);
        if (nextVar.countUpdate > 0) {
            updateQ.qAppend(nextVarId);
        }
    }
    varSeqNum++;
    return informationElement;
}

InformationElement VarDisClient::composeDeleteVariablesIEElements(size_t* currentPayloadSize) {
    TimeStamp time;
    IEHeader ieHeader = {
            .ieType = IETYPE_DELETE_VARIABLES,
            .ieNumRecords = 0,
            .ieTimestamp = time,
            .ieSeqNum = varSeqNum,
    };
    InformationElement informationElement = {
            .ieHeader = ieHeader,
    };
    deleteQ.qDropNonexisting();

    if (deleteQ.qIsEmpty() || VARDISPAR_MAX_PAYLOAD_SIZE - *currentPayloadSize < (sizeof(VarId) + sizeof(IEHeader))) {
        RCLCPP_INFO(rclcpp::get_logger("logger"), "Stopped composeDeleteVariablesIEElements at first check");
        return informationElement;
    }
    uint8_t numRecordsToAdd = numberFittingRecords(deleteQ, sizeof(VarId), currentPayloadSize);
    informationElement.ieHeader.ieNumRecords = numRecordsToAdd;
    for (uint8_t i = 0; i < numRecordsToAdd; i++) {
        VarId nextVarId = deleteQ.qPeek();
        DBEntry nextVar = databaseManager.lookup(nextVarId);
        if (nextVar.countDelete == 0) {
            continue;
        }
        deleteQ.qTake();
        nextVar.countDelete -= 1;
        databaseManager.update(nextVar);
        helpers.addVarIdToIEElement(&informationElement, &nextVar, currentPayloadSize);
        if (nextVar.countDelete > 0) {
            deleteQ.qAppend(nextVarId);
        } else {
            databaseManager.remove(nextVarId);
        }
    }
    varSeqNum++;
    return informationElement;
}

InformationElement VarDisClient::composeRequestVarUpdatesIEElements(size_t* currentPayloadSize) {
    TimeStamp time;
    IEHeader ieHeader = {
            .ieType = IETYPE_REQUEST_VARUPDATES,
            .ieNumRecords = 0,
            .ieTimestamp = time,
            .ieSeqNum = varSeqNum,
    };
    InformationElement informationElement = {
            .ieHeader = ieHeader,
    };
    reqUpdQ.qDropNonexistingDeleted();
    if (reqUpdQ.qIsEmpty() || VARDISPAR_MAX_PAYLOAD_SIZE - *currentPayloadSize < (sizeof(VarSumm) + sizeof(IEHeader))) {
        return informationElement;
    }
    uint8_t numRecordsToAdd = numberFittingRecords(reqUpdQ, sizeof(VarSumm), currentPayloadSize);
    informationElement.ieHeader.ieNumRecords = numRecordsToAdd;
    for (uint8_t i = 0; i < numRecordsToAdd; i++) {
        VarId nextVarId = reqUpdQ.qTake();
        DBEntry nextVar = databaseManager.lookup(nextVarId);
        helpers.addVarSummToIEElement(&informationElement, &nextVar, currentPayloadSize);
    }
    varSeqNum++;
    return informationElement;
}

InformationElement VarDisClient::composeRequestVarCreatesIEElements(size_t* currentPayloadSize) {
    TimeStamp time;
    IEHeader ieHeader = {
            .ieType = IETYPE_REQUEST_VARCREATES,
            .ieNumRecords = 0,
            .ieTimestamp = time,
            .ieSeqNum = varSeqNum,
    };
    InformationElement informationElement = {
            .ieHeader = ieHeader,
    };
    reqCreateQ.qDropDeleted();
    if (reqCreateQ.qIsEmpty() || VARDISPAR_MAX_PAYLOAD_SIZE - *currentPayloadSize < (sizeof(VarId) + sizeof(IEHeader))) {
        return informationElement;
    }
    uint8_t numRecordsToAdd = numberFittingRecords(reqCreateQ, sizeof(VarId), currentPayloadSize);
    informationElement.ieHeader.ieNumRecords = numRecordsToAdd;
    for (uint8_t i = 0; i < numRecordsToAdd; i++) {
        VarId nextVarId = reqCreateQ.qTake();
        DBEntry nextVar = databaseManager.lookup(nextVarId);
        helpers.addVarSummToIEElement(&informationElement, &nextVar, currentPayloadSize);
    }
    varSeqNum++;
    return informationElement;
}

void VarDisClient::registerClientProtocol()
{
    // Create register request message.
    dcp_msgs::msg::RegisterProtocolRequest request;
    request.id = BP_PROTID_VARDIS;
    request.name = "VarDis – Variable Dissemination Protocol V1.0";
    request.length = VARDISPAR_MAX_PAYLOAD_SIZE;
    request.mode = BP_QMODE_ONCE;

    // Send register request to BP.
    RCLCPP_INFO(rclcpp::get_logger("logger"), "Sending VarDis register request");
    protocolRegisterPub->publish(request);
}

void VarDisClient::checkNumberBufferedPayloads() {
    dcp_msgs::msg::QueryNumberBufferedPayloadsRequest request;
    request.id = BP_PROTID_VARDIS;
    RCLCPP_INFO(rclcpp::get_logger("logger"), "Querying buffer queue from VarDis");
    protocolCheckQueueBuffer->publish(request);
}

void VarDisClient::queryBufferResponse(const dcp_msgs::msg::QueryNumberBufferedPayloadsResponse& response) {
    RCLCPP_INFO(rclcpp::get_logger("logger"), "Reading query buffer response at: queryBufferResponse");
    RCLCPP_INFO(rclcpp::get_logger("logger"), "Number: %d, Status: %d", response.number, response.status);
    if (response.number == 0) {
        RCLCPP_INFO(rclcpp::get_logger("logger"), "Generating VarDisPayload at VarDisClient::queryBufferResponse");

        dcp_msgs::msg::TransmitPayloadRequest transmitPayloadRequest;
        transmitPayloadRequest.id = BP_PROTID_VARDIS;
        transmitPayloadRequest.payload = generatePayload();
        transmitPayloadRequest.length = transmitPayloadRequest.payload.size();

        RCLCPP_INFO(rclcpp::get_logger("logger"), "Successfully generated payload of size: %u", transmitPayloadRequest.length);
        if (transmitPayloadRequest.length > 0) {
            varDisResults["packetsSent"] += 1;
            protocolTransmitPayload->publish(transmitPayloadRequest);
            RCLCPP_INFO(rclcpp::get_logger("logger"), "VarDisPayload sent at VarDisClient::queryBufferResponse");
        } else {
            RCLCPP_INFO(rclcpp::get_logger("logger"), "Empty VarDisPayload not sent at VarDisClient::queryBufferResponse");
        }
    }
}

void VarDisClient::receive_payload_callback_indication(const dcp_msgs::msg::ReceivePayloadIndication& indic)
{
    // Collect payload data from indication message
    std::vector<uint8_t> data = indic.payload;
    std::string droneName = indic.id;
    uint8_t protocolId = indic.protid;

    if (protocolId == BP_PROTID_VARDIS) {
        varDisResults["packetsReceived"] += 1;
        RCLCPP_INFO(rclcpp::get_logger("logger"), "ReceivePayloadIndication at VarDisClient::receive_payload_callback_indication");

        RCLCPP_INFO(rclcpp::get_logger("logger"), "Payload size at VarDisClient::receive_payload_callback_indication: %ld", data.size());
        RCLCPP_INFO(rclcpp::get_logger("logger"), "Processing elements at VarDisClient::receive_payload_callback_indication");
        int64_t timestamp = indic.timestamp;

        size_t offset = 0;
        std::vector<InformationElement> allInformationElements = {};

        while (offset < data.size()) {
            InformationElement informationElement = helpers.deserializeInformationElement(data, offset);
            allInformationElements.push_back(informationElement);

            double timeToReceive = timestamp - informationElement.ieHeader.ieTimestamp.time.count();

            varDisResults["averageTime"] += timeToReceive / 1000000000;
            std::cout << informationElement << " after: " << timestamp - informationElement.ieHeader.ieTimestamp.time.count() << std::endl;
        }

        for (size_t i = 0; i < allInformationElements.size(); ++i) {
            if (i < processFunctions.size()) {
                InformationElement element = allInformationElements[i];
                processFunctions[element.ieHeader.ieType](element.ieList);
            }
        }

        std::cout << "Printing all variables in the Database" << std::endl;
        databaseManager.printResults(databaseManager.getAllDBEntries());
        createQ.printQueue();
        deleteQ.printQueue();
        updateQ.printQueue();
        summaryQ.printQueue();
        reqCreateQ.printQueue();
        reqUpdQ.printQueue();

    }
}

void VarDisClient::transmitted_payload_callback_indication(const dcp_msgs::msg::PayloadTransmittedIndication& indic)
{
    // Collect payload data from indication message
    uint8_t protocolId = indic.id;

    if (protocolId == BP_PROTID_VARDIS) {
        RCLCPP_INFO(rclcpp::get_logger("logger"), "PayloadTransmittedIndication at VarDisClient::transmitted_payload_callback_indication");
        dcp_msgs::msg::TransmitPayloadRequest transmitPayloadRequest;
        transmitPayloadRequest.id = BP_PROTID_VARDIS;
        transmitPayloadRequest.payload = generatePayload();
        transmitPayloadRequest.length = transmitPayloadRequest.payload.size();
        TimeStamp timeStamp;
        transmitPayloadRequest.timestamp = timeStamp.time.count();
        RCLCPP_INFO(rclcpp::get_logger("logger"), "Payload size at VarDisClient::transmitted_payload_callback_indication: %d", transmitPayloadRequest.length);

        if (transmitPayloadRequest.length > 0) {
            varDisResults["packetsSent"] += 1;
            protocolTransmitPayload->publish(transmitPayloadRequest);
            RCLCPP_INFO(rclcpp::get_logger("logger"), "VarDisPayload sent at VarDisClient::transmitted_payload_callback_indication");
        } else {
            RCLCPP_INFO(rclcpp::get_logger("logger"), "Empty VarDisPayload not sent at VarDisClient::queryBufferResponse");
        }
    }
}

std::vector<uint8_t> VarDisClient::generatePayload() {
    // Create as many elements as possible
    size_t currentPayloadSize = 0;
    std::vector<InformationElement> allInformationElements = {};

    if (!createQ.qIsEmpty()) {
        InformationElement informationElementCreates = composeCreateVariablesIEElements(&currentPayloadSize);
        allInformationElements.push_back(informationElementCreates);
    }
    if (!deleteQ.qIsEmpty()) {
        InformationElement informationElementDeletes = composeDeleteVariablesIEElements(&currentPayloadSize);
        allInformationElements.push_back(informationElementDeletes);
    }
    if (!updateQ.qIsEmpty()) {
        InformationElement informationElementUpdates = composeUpdateIEElements(&currentPayloadSize);
        allInformationElements.push_back(informationElementUpdates);
    }
    if (!summaryQ.qIsEmpty()) {
        InformationElement informationElementSummaries = composeSummaryIEElements(&currentPayloadSize);
        allInformationElements.push_back(informationElementSummaries);
    }
    if (!reqCreateQ.qIsEmpty()) {
        InformationElement informationElementRequestCreates = composeRequestVarCreatesIEElements(&currentPayloadSize);
        allInformationElements.push_back(informationElementRequestCreates);
    }
    if (!reqUpdQ.qIsEmpty()) {
        InformationElement informationElementRequestUpdates = composeRequestVarUpdatesIEElements(&currentPayloadSize);
        allInformationElements.push_back(informationElementRequestUpdates);
    }

    std::vector<uint8_t> serializedInformationElements = {};
    for (size_t i = 0; i < allInformationElements.size(); ++i) {
        std::cout << allInformationElements[i] << std::endl;
        helpers.serializeInformationElement(serializedInformationElements, allInformationElements[i]);
    }

    return serializedInformationElements;
}

void VarDisClient::generateVariable() {
    std::vector<uint8_t> value;
    value.push_back(static_cast<uint8_t>(0x41)); // Example: adding byte 'A'
    value.push_back(static_cast<uint8_t>(0x42)); // Example: adding byte 'B'
    value.push_back(static_cast<uint8_t>(0x43)); // Example: adding byte 'C'

    VarId id = databaseManager.getNextAvailableVarId();

    VarSpec spec(id,
                  helpers.getNodeIdentifier(),
                  2,
                  "Variable" + std::to_string(id));

    std::cout << "Attempting to generate for: " << id << std::endl;
    databaseManager.create(spec, value.size(), value);
}

void VarDisClient::deleteVariable() {

    std::random_device randomDevice;
    std::mt19937 gen(randomDevice());
    std::uniform_int_distribution<VarId> dis(1, databaseManager.getNextAvailableVarId() - 1);
    VarId randomNumber = dis(gen);

    std::cout << "Attempting to delete: " << randomNumber << std::endl;
    databaseManager.deleteDBEntryById(randomNumber);
}

void VarDisClient::updateVariable() {

    std::random_device randomDevice;
    std::mt19937 gen(randomDevice());
    std::uniform_int_distribution<VarId> dis(1, databaseManager.getNextAvailableVarId() - 1);
    VarId randomNumber = dis(gen);

    DBEntry entry = databaseManager.lookup(randomNumber);
    if (entry.length == 0 || not entry.existsInDB) {
        return;
    }
    entry.value.pop_back();
    std::cout << "Attempting to update for: " << randomNumber << " or  " << entry.spec.varId << std::endl;
    databaseManager.updateDBEntryById(randomNumber, entry.value.size(), entry.value);
}

uint8_t VarDisClient::numberFittingRecords(VarDisQueue &queue, const size_t recordSize, size_t* currentPayloadSize) {
    size_t availablePayloadSize = VARDISPAR_MAX_PAYLOAD_SIZE - *currentPayloadSize;
    uint8_t numRecords = 0;

    while (!queue.qIsEmpty() && availablePayloadSize >= recordSize + sizeof(IEHeader) && numRecords < queue.size()) {
        availablePayloadSize -= recordSize;
        numRecords++;
    }

    return numRecords;
}

void VarDisClient::writeResultsToFile() {
    // Create an ofstream object
    std::ofstream outputFile("vardis-Results.txt");

    // Check if the file was opened successfully
    if (!outputFile) {
        std::cerr << "Error opening file for writing!" << std::endl;
        return;
    }

    // Write to the file instead of std::cout
    outputFile << "Packets Sent: " << varDisResults["packetsSent"] << std::endl;
    outputFile << "Packets Received: " << varDisResults["packetsReceived"] << std::endl;
    outputFile << "Average Time: " << (varDisResults["averageTime"] / varDisResults["packetsReceived"]) << std::endl;

    // Close the file
    outputFile.close();

}