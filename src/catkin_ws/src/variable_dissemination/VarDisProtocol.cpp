#include "headers/VarDisProtocol.h"

VarDisProtocol::VarDisProtocol() {

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

 DatabaseManager& VarDisProtocol::getDatabaseManager() {
    return databaseManager;
}


void VarDisProtocol::processCreateIEElements(std::vector<std::byte> rawIEByteVector) {

    std::vector<VarCreate> deserializedVarCreateList = helpers.deserializeVarCreateList(rawIEByteVector);

    for(VarCreate rcvdVC: deserializedVarCreateList) {
        VarSpec spec = rcvdVC.spec;
        VarUpd upd = rcvdVC.upd;
        VarId varId = spec.varId;

        if (databaseManager.lookup(varId).spec.varId == 0) {
            std::cerr << "Stopping function processCreateIEElements for VarId: " << varId << std::endl;
            continue;
        }

        if (spec.prodId == helpers.getNodeIdentifier()) {
            std::cerr << "Stopping function processCreateIEElements for prodId: " << spec.prodId << std::endl;
            continue;
        }

        TimeStamp timeStamp;

        DBEntry dbEntry = {
                .spec = spec,
                .length = upd.varLen,
                .value = upd.Value,
                .seqno = upd.varSeqNo,
                .tStamp = timeStamp.time.count(),
                .countUpdate = 0,
                .countCreate = spec.repCnt,
                .countDelete = 0,
                .toBeDeleted = false
        };

        databaseManager.update(dbEntry);

    }
}

void VarDisProtocol::processDeleteIEElements(std::vector<std::byte> rawIEByteVector) {

    std::vector<VarId> varIds = helpers.deserializeVarIdList(rawIEByteVector);

    for(VarId varId: varIds) {
        DBEntry dbEntry = databaseManager.lookup(varId);

        if (dbEntry.spec.varId == 0) {
            std::cerr << "Stopping function processDeleteIEElements for VarId: " << varId << std::endl;
            continue;
        }

        if (dbEntry.toBeDeleted) {
            std::cerr << "Stopping function processDeleteIEElements as toBeDeleted is true for VarId: " << varId << std::endl;
            continue;
        }

        if (dbEntry.spec.prodId == helpers.getNodeIdentifier()) {
            std::cerr << "Stopping function processDeleteIEElements for prodId: " << dbEntry.spec.prodId << std::endl;
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

void VarDisProtocol::processUpdateIEElements(std::vector<std::byte> rawIEByteVector) {

    std::vector<VarUpd> deserializeVarUpdList = helpers.deserializeVarUpdList(rawIEByteVector);
    for(VarUpd upd: deserializeVarUpdList) {
        DBEntry dbEntry = databaseManager.lookup(upd.varId);
        if (dbEntry.spec.varId == 0) {
            if (!reqCreateQ.qExists(upd.varId)) {
                reqCreateQ.qAppend(upd.varId);
                std::cerr << "Stopping function processUpdateIEElements as reqCreateQ doesn't contain VarId: " << upd.varId << std::endl;
                continue;
            }
        }
        if (dbEntry.toBeDeleted) {
            std::cerr << "Stopping function processUpdateIEElements as toBeDeleted is true for VarId: " << upd.varId << std::endl;
            continue;
        }
        if (dbEntry.spec.prodId == helpers.getNodeIdentifier()) {
            std::cerr << "Stopping function processUpdateIEElements for prodId: " << dbEntry.spec.prodId << std::endl;
            continue;
        }
        if (upd.varSeqNo == dbEntry.seqno) {
            std::cerr << "Stopping function processUpdateIEElements as same seqno for VarId: " << upd.varId << std::endl;
            continue;
        }
        // TODO: if (upd.seqno is strictly older than ent.seqno), what does this mean?
        if (upd.varSeqNo) {
            if (!updateQ.qExists(upd.varId)) {
                updateQ.qAppend(upd.varId);
                dbEntry.countUpdate = dbEntry.spec.repCnt;
                databaseManager.update(dbEntry);
                std::cerr << "Stopping function processUpdateIEElements as upd.seqno is strictly older for VarId: " << upd.varId << std::endl;
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

void VarDisProtocol::processSummaryIEElements(std::vector<std::byte> rawIEByteVector) {

    std::vector<VarSumm> deserializeVarSummList = helpers.deserializeVarSummList(rawIEByteVector);
    for (VarSumm summ: deserializeVarSummList) {
        DBEntry dbEntry = databaseManager.lookup(summ.varId);
        if (dbEntry.spec.varId == 0) {
            if (!reqCreateQ.qExists(summ.varId)) {
                reqCreateQ.qAppend(summ.varId);
            }
        }
        if (dbEntry.toBeDeleted) {
            std::cerr << "Stopping function processSummaryIEElements as toBeDeleted is true for VarId: " << summ.varId << std::endl;
            continue;
        }
        if (dbEntry.spec.prodId == helpers.getNodeIdentifier()) {
            std::cerr << "Stopping function processSummaryIEElements as same prodId for VarId: " << summ.varId << std::endl;
            continue;
        }
        if (summ.varSeqNo == dbEntry.seqno) {
            std::cerr << "Stopping function processSummaryIEElements as same seqno for VarId: " << summ.varId << std::endl;
            continue;
        }
        // TODO: if (summ.seqno is strictly older than ent.seqno), what does this mean?
        if (summ.varSeqNo) {
            if (!updateQ.qExists(summ.varId)) {
                updateQ.qAppend(summ.varId);
                dbEntry.countUpdate = dbEntry.spec.repCnt;
                databaseManager.update(dbEntry);
                std::cerr << "Stopping function processSummaryIEElements as summ.seqno is strictly older for VarId: " << summ.varId << std::endl;
                continue;
            }
        }
        if (!reqUpdQ.qExists(summ.varId)) {
            reqUpdQ.qAppend(summ.varId);
        }
    }
}

void VarDisProtocol::processRequestVarUpdsIEElements(std::vector<std::byte> rawIEByteVector) {

    std::vector<VarSumm> deserializeVarSummList = helpers.deserializeVarSummList(rawIEByteVector);

    for (VarSumm summ: deserializeVarSummList) {
        DBEntry dbEntry = databaseManager.lookup(summ.varId);
        if (dbEntry.spec.varId == 0) {
            std::cerr << "Stopping function processRequestVarUpdsIEElements for VarId: " << summ.varId << std::endl;
            continue;
        }
        if (dbEntry.toBeDeleted) {
            std::cerr << "Stopping function processRequestVarUpdsIEElements as toBeDeleted is true for VarId: " << summ.varId << std::endl;
            continue;
        }
        if (dbEntry.seqno < summ.varSeqNo) {
            std::cerr << "Stopping function processRequestVarUpdsIEElements as dbEntry.seqno is not strictly larger for VarId: " << summ.varId << std::endl;
        }
        dbEntry.countUpdate = dbEntry.spec.repCnt;
        databaseManager.update(dbEntry);
        if (!updateQ.qExists(summ.varId)) {
            updateQ.qAppend(summ.varId);
        }
    }
 }

void VarDisProtocol::processRequestVarCreatesIEElements(std::vector<std::byte> rawIEByteVector) {

    std::vector<VarId> varIds = helpers.deserializeVarIdList(rawIEByteVector);

    for (VarId varId: varIds) {
        DBEntry dbEntry = databaseManager.lookup(varId);
        if (dbEntry.spec.varId == 0) {
            std::cerr << "Stopping function processRequestVarCreatesIEElements for VarId: " << varId << std::endl;
            continue;
        }
        if (dbEntry.toBeDeleted) {
            std::cerr << "Stopping function processRequestVarCreatesIEElements as toBeDeleted is true for VarId: " << varId << std::endl;
            continue;
        }
        dbEntry.countCreate = dbEntry.spec.repCnt;
        databaseManager.update(dbEntry);
        if (!createQ.qExists(varId)) {
            createQ.qAppend(varId);
        }
    }
}

InformationElement VarDisProtocol::composeSummaryIEElements(size_t* currentPayloadSize) {
    InformationElement informationElement = {
            .ieType = IETYPE_SUMMARIES,
            .ieLen = 0,
    };
    summaryQ.qDropNonexistingDeleted();
    if (    summaryQ.qIsEmpty()
         || VARDISPAR_MAX_PAYLOAD_SIZE - *currentPayloadSize < sizeof(VarSumm)
         || VARDISPAR_MAX_SUMMARIES == 0) {
        std::cout << "Stopped composeSummaryIEElements at first check" << std::endl;
        return informationElement;
    }
    int numSummaries = 0;
    VarId firstVarId = summaryQ.qTake();
    DBEntry dbEntry = databaseManager.lookup(firstVarId);
    helpers.addVarSummToIEElement(&informationElement, &dbEntry, currentPayloadSize);
    summaryQ.qAppend(firstVarId);

    while (     summaryQ.qPeek() != firstVarId
             && summaryQ.qPeek() != 0
             && numSummaries < VARDISPAR_MAX_SUMMARIES
             && VARDISPAR_MAX_PAYLOAD_SIZE - *currentPayloadSize >= sizeof(VarSumm)) {
        VarId nextVarId = summaryQ.qTake();
        summaryQ.qAppend(nextVarId);
        numSummaries += 1;
        DBEntry newDbEntry = databaseManager.lookup(nextVarId);
        helpers.addVarSummToIEElement(&informationElement, &newDbEntry, currentPayloadSize);
    }
    informationElement.ieLen = informationElement.ieList.size();

    return informationElement;
}

InformationElement VarDisProtocol::composeCreateVariablesIEElements(size_t* currentPayloadSize) {
    InformationElement informationElement = {
            .ieType = IETYPE_CREATE_VARIABLES,
            .ieLen = 0,
    };
    createQ.qDropNonexistingDeleted();

    if (createQ.qIsEmpty() || VARDISPAR_MAX_PAYLOAD_SIZE - *currentPayloadSize < sizeof(VarCreate)) {
        std::cout << "Stopped composeCreateVariablesIEElements at first check" << std::endl;
        return informationElement;
    }

    while (     !createQ.qIsEmpty()
             && databaseManager.lookup(createQ.qPeek()).countCreate == 1
             && VARDISPAR_MAX_PAYLOAD_SIZE - *currentPayloadSize >= sizeof(VarCreate)) {
        VarId firstVarId = createQ.qTake();
        DBEntry firstVar = databaseManager.lookup(firstVarId);
        helpers.addVarCreateToIEElement(&informationElement, &firstVar, currentPayloadSize);
    }

    if (createQ.qIsEmpty() || VARDISPAR_MAX_PAYLOAD_SIZE - *currentPayloadSize < sizeof(VarCreate)) {
        std::cout << "Stopped composeCreateVariablesIEElements at second check" << std::endl;
        return informationElement;
    }

    VarId firstVarId = createQ.qTake();
    DBEntry firstVar = databaseManager.lookup(firstVarId);
    firstVar.countCreate -= 1;
    helpers.addVarCreateToIEElement(&informationElement, &firstVar, currentPayloadSize);
    createQ.qAppend(firstVarId);

    while (     !createQ.qIsEmpty()
             && createQ.qPeek() != firstVarId
             && VARDISPAR_MAX_PAYLOAD_SIZE - *currentPayloadSize >= sizeof(VarCreate)) {
        VarId nextVarId = createQ.qTake();
        DBEntry nextVar = databaseManager.lookup(nextVarId);
        helpers.addVarCreateToIEElement(&informationElement, &nextVar, currentPayloadSize);
        nextVar.countCreate -= 1;
        if (nextVar.countCreate > 0) {
            createQ.qAppend(nextVarId);
        }
    }
    informationElement.ieLen = informationElement.ieList.size();
    return informationElement;
}

InformationElement VarDisProtocol::composeUpdateIEElements(size_t* currentPayloadSize) {
    InformationElement informationElement = {
            .ieType = IETYPE_UPDATES,
            .ieLen = 0,
    };
    updateQ.qDropNonexistingDeleted();

    if (updateQ.qIsEmpty() || VARDISPAR_MAX_PAYLOAD_SIZE - *currentPayloadSize < sizeof(VarUpd)) {
        std::cout << "Stopped composeUpdateIEElements at first check" << std::endl;
        return informationElement;
    }

    while (     !updateQ.qIsEmpty()
                && databaseManager.lookup(updateQ.qPeek()).countUpdate == 1
                && VARDISPAR_MAX_PAYLOAD_SIZE - *currentPayloadSize >= sizeof(VarUpd)) {
        VarId firstVarId = updateQ.qTake();
        DBEntry firstVar = databaseManager.lookup(firstVarId);
        helpers.addVarUpdateToIEElement(&informationElement, &firstVar, currentPayloadSize);
    }

    if (updateQ.qIsEmpty() || VARDISPAR_MAX_PAYLOAD_SIZE - *currentPayloadSize < sizeof(VarUpd)) {
        std::cout << "Stopped composeUpdateIEElements at second check" << std::endl;
        return informationElement;
    }

    VarId firstVarId = updateQ.qTake();
    DBEntry firstVar = databaseManager.lookup(firstVarId);
    firstVar.countUpdate -= 1;
    helpers.addVarUpdateToIEElement(&informationElement, &firstVar, currentPayloadSize);
    updateQ.qAppend(firstVarId);

    while (     !updateQ.qIsEmpty()
                && updateQ.qPeek() != firstVarId
                && VARDISPAR_MAX_PAYLOAD_SIZE - *currentPayloadSize >= sizeof(VarUpd)) {
        VarId nextVarId = updateQ.qTake();
        DBEntry nextVar = databaseManager.lookup(nextVarId);
        helpers.addVarUpdateToIEElement(&informationElement, &nextVar, currentPayloadSize);
        nextVar.countUpdate -= 1;
        if (nextVar.countUpdate > 0) {
            updateQ.qAppend(nextVarId);
        }
    }
    informationElement.ieLen = informationElement.ieList.size();
    return informationElement;
}

InformationElement VarDisProtocol::composeDeleteVariablesIEElements(size_t* currentPayloadSize) {
    InformationElement informationElement = {
            .ieType = IETYPE_DELETE_VARIABLES,
            .ieLen = 0,
    };
    deleteQ.qDropNonexistingDeleted();

    if (deleteQ.qIsEmpty() || VARDISPAR_MAX_PAYLOAD_SIZE - *currentPayloadSize < sizeof(VarId)) {
        std::cout << "Stopped composeDeleteVariablesIEElements at first check" << std::endl;
        return informationElement;
    }

    while (     !deleteQ.qIsEmpty()
                && databaseManager.lookup(deleteQ.qPeek()).countDelete == 1
                && VARDISPAR_MAX_PAYLOAD_SIZE - *currentPayloadSize >= sizeof(VarId)) {
        VarId firstVarId = deleteQ.qTake();
        DBEntry firstVar = databaseManager.lookup(firstVarId);
        helpers.addVarIdToIEElement(&informationElement, &firstVar, currentPayloadSize);
    }

    if (deleteQ.qIsEmpty() || VARDISPAR_MAX_PAYLOAD_SIZE - *currentPayloadSize < sizeof(VarId)) {
        std::cout << "Stopped composeDeleteVariablesIEElements at second check" << std::endl;
        return informationElement;
    }

    VarId firstVarId = deleteQ.qTake();
    DBEntry firstVar = databaseManager.lookup(firstVarId);
    firstVar.countDelete -= 1;
    helpers.addVarIdToIEElement(&informationElement, &firstVar, currentPayloadSize);
    deleteQ.qAppend(firstVarId);

    while (     !deleteQ.qIsEmpty()
                && deleteQ.qPeek() != firstVarId
                && VARDISPAR_MAX_PAYLOAD_SIZE - *currentPayloadSize >= sizeof(VarId)) {
        VarId nextVarId = deleteQ.qTake();
        DBEntry nextVar = databaseManager.lookup(nextVarId);
        helpers.addVarIdToIEElement(&informationElement, &nextVar, currentPayloadSize);
        nextVar.countDelete -= 1;
        if (nextVar.countDelete > 0) {
            deleteQ.qAppend(nextVarId);
        }
    }
    informationElement.ieLen = informationElement.ieList.size();
    return informationElement;
}

InformationElement VarDisProtocol::composeRequestVarUpdatesIEElements(size_t* currentPayloadSize) {
    InformationElement informationElement = {
            .ieType = IETYPE_REQUEST_VARUPDATES,
            .ieLen = 0,
    };
    reqUpdQ.qDropNonexistingDeleted();
    if (reqUpdQ.qIsEmpty() || VARDISPAR_MAX_PAYLOAD_SIZE - *currentPayloadSize < sizeof(VarSumm)) {
        std::cout << "Stopped composeRequestVarUpdatesIEElements at first check" << std::endl;
        return informationElement;
    }
    while (     !reqUpdQ.qIsEmpty()
                && VARDISPAR_MAX_PAYLOAD_SIZE - *currentPayloadSize >= sizeof(VarSumm)) {
        VarId nextVarId = reqUpdQ.qTake();
        DBEntry nextVar = databaseManager.lookup(nextVarId);
        helpers.addVarSummToIEElement(&informationElement, &nextVar, currentPayloadSize);
    }
    informationElement.ieLen = informationElement.ieList.size();
    return informationElement;
}

InformationElement VarDisProtocol::composeRequestVarCreatesIEElements(size_t* currentPayloadSize) {
    InformationElement informationElement = {
            .ieType = IETYPE_REQUEST_VARCREATES,
            .ieLen = 0,
    };
    reqCreateQ.qDropNonexistingDeleted();
    if (reqCreateQ.qIsEmpty() || VARDISPAR_MAX_PAYLOAD_SIZE - *currentPayloadSize < sizeof(VarId)) {
        std::cout << "Stopped composeRequestVarCreatesIEElements at first check" << std::endl;
        return informationElement;
    }
    while (     !reqCreateQ.qIsEmpty()
                && VARDISPAR_MAX_PAYLOAD_SIZE - *currentPayloadSize >= sizeof(VarId)) {
        VarId nextVarId = reqCreateQ.qTake();
        DBEntry nextVar = databaseManager.lookup(nextVarId);
        helpers.addVarIdToIEElement(&informationElement, &nextVar, currentPayloadSize);
    }
    informationElement.ieLen = informationElement.ieList.size();
    return informationElement;
}