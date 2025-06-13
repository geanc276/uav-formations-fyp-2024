#include "dcp/DatabaseManager.h"

Helpers helpers = Helpers();

DatabaseManager::DatabaseManager() : createQ(nullptr), deleteQ(nullptr), updateQ(nullptr),
                                                        summaryQ(nullptr), reqUpdQ(nullptr), reqCreateQ(nullptr) {
}

void DatabaseManager::setQueues(VarDisQueue* createQueue, VarDisQueue* deleteQueue, VarDisQueue* updateQueue,
                                VarDisQueue* summaryQueue, VarDisQueue* reqUpdQueue, VarDisQueue* reqCreateQueue) {
    this->createQ = createQueue;
    this->deleteQ = deleteQueue;
    this->updateQ = updateQueue;
    this->summaryQ = summaryQueue;
    this->reqUpdQ = reqUpdQueue;
    this->reqCreateQ = reqCreateQueue;
}

DatabaseManager::~DatabaseManager() {
}


tbb::concurrent_unordered_map<VarId, DBEntry>  DatabaseManager::getDatabaseInstance() {
    return dataBase;
}

int DatabaseManager::create(VarSpec spec, VarLen length, std::vector<uint8_t> value) {

    if (lookup(spec.varId).existsInDB) {
        std::cout << "Can't create variable: VARDIS_STATUS_VARIABLE_EXISTS" << std::endl;
        return VARDIS_STATUS_VARIABLE_EXISTS;
    }

    if (spec.descr.length() > (VARDISPAR_MAX_DESCRIPTION_LENGTH - 1)) {
        std::cout << "Can't create variable: VARDIS_STATUS_VARIABLE_DESCRIPTION_TOO_LONG" << std::endl;
        return VARDIS_STATUS_VARIABLE_DESCRIPTION_TOO_LONG;
    }

    if (length > VARDISPAR_MAX_VALUE_LENGTH) {
        std::cout << "Can't create variable: VARDIS_STATUS_VALUE_TOO_LONG" << std::endl;
        return VARDIS_STATUS_VALUE_TOO_LONG;
    }

    if (length == 0) {
        std::cout << "Can't create variable: VARDIS_STATUS_INVALID_VALUE" << std::endl;
        return VARDIS_STATUS_INVALID_VALUE;
    }

    if ((spec.repCnt <= 0) || (spec.repCnt > VARDISPAR_MAX_REPITITIONS)) {
        std::cout << "Can't create variable: VARDIS_STATUS_ILLEGAL_REPCOUNT" << std::endl;
        return VARDIS_STATUS_ILLEGAL_REPCOUNT;
    }

    TimeStamp timeStamp;

    DBEntry entry = {
        .spec = spec,
        .length = length,
        .value = value,
        .seqno = 0,
        .tStamp = timeStamp.time.count(),
        .countUpdate = 0,
        .countCreate = spec.repCnt,
        .countDelete = 0,
        .toBeDeleted = 0,
        .existsInDB = 1,
    };

    update(entry);

    createQ->qAppend(spec.varId);
    summaryQ->qAppend(spec.varId);

    return VARDIS_STATUS_OK;
}

bool DatabaseManager::updateDBEntryById(VarId varId, VarLen length, const std::vector<uint8_t>& value) {

    if (not lookup(varId).existsInDB) {
        std::cout << "Can't update variable: VARDIS_STATUS_VARIABLE_DOES_NOT_EXIST" << std::endl;
        return VARDIS_STATUS_VARIABLE_DOES_NOT_EXIST;
    }

    DBEntry entry = lookup(varId);

    if (entry.spec.prodId != helpers.getNodeIdentifier()) {
        std::cout << "Can't update variable: VARDIS_STATUS_VARIABLE_DOES_NOT_EXIST" << std::endl;
        return VARDIS_STATUS_NOT_PRODUCER;
    }

    if (entry.toBeDeleted) {
        std::cout << "Can't update variable: VARDIS_STATUS_VARIABLE_BEING_DELETED" << std::endl;
        return VARDIS_STATUS_VARIABLE_BEING_DELETED;
    }

    if (length > VARDISPAR_MAX_VALUE_LENGTH) {
        std::cout << "Can't update variable: VARDIS_STATUS_VALUE_TOO_LONG" << std::endl;
        return VARDIS_STATUS_VALUE_TOO_LONG;
    }

    if (length == 0) {
        std::cout << "Can't update variable: VARDIS_STATUS_INVALID_VALUE" << std::endl;
        return VARDIS_STATUS_INVALID_VALUE;
    }

    TimeStamp timeStamp;

    entry.seqno = entry.seqno + 1; // TODO: % Maximum seqno, not sure what this is
    entry.length = length;
    entry.value = value;
    entry.countUpdate = entry.spec.repCnt;
    entry.tStamp = timeStamp.time.count();
    update(entry);

    if (not updateQ->qExists(varId)) {
        updateQ->qAppend(varId);
    }

    return VARDIS_STATUS_OK;
}

bool DatabaseManager::update(DBEntry dbEntry) {
    TimeStamp timeStamp;
    dbEntry.tStamp = timeStamp.time.count();

    if (not lookup(dbEntry.spec.varId).existsInDB) {
        nextVarId += 1;
    }

    dataBase[dbEntry.spec.varId] = dbEntry;
    std::cout << "All DBEntry Data update successfully\n";
    return VARDIS_STATUS_OK;
}

bool DatabaseManager::deleteDBEntryById(VarId varId) {

    if (not lookup(varId).existsInDB) {
        std::cout << "Can't delete variable: VARDIS_STATUS_VARIABLE_DOES_NOT_EXIST" << std::endl;
        return VARDIS_STATUS_VARIABLE_DOES_NOT_EXIST;
    }

    DBEntry entry = lookup(varId);

    if (entry.spec.prodId != helpers.getNodeIdentifier()) {
        std::cout << "Can't delete variable: VARDIS_STATUS_NOT_PRODUCER" << std::endl;
        return VARDIS_STATUS_NOT_PRODUCER;
    }

    if (entry.toBeDeleted) {
        std::cout << "Can't delete variable: VARDIS_STATUS_VARIABLE_BEING_DELETED" << std::endl;
        return VARDIS_STATUS_VARIABLE_BEING_DELETED;
    }

    deleteQ->qAppend(varId);
    createQ->qRemove(varId);
    summaryQ->qRemove(varId);
    updateQ->qRemove(varId);
    reqUpdQ->qRemove(varId);
    reqCreateQ->qRemove(varId);

    entry.toBeDeleted = 1;
    entry.countDelete = entry.spec.repCnt;
    entry.countCreate = 0;
    entry.countUpdate = 0;
    update(entry);

    std::cout << "DBEntry successfully prepared for deletion\n";
    return VARDIS_STATUS_OK;
}

void DatabaseManager::remove(VarId varId) {
    dataBase.unsafe_erase(varId);
    std::cout << "DBEntry deleted successfully\n";
}

std::vector<DBEntry> DatabaseManager::getAllDBEntries() {
    std::vector<DBEntry> allData;
    for (const auto& pair : dataBase) {
        allData.push_back(pair.second);
    }
    std::reverse(allData.begin(), allData.end());
    return allData;
}

DBEntry DatabaseManager::lookup(int varId) {
    DBEntry dbEntry = {
            .spec = VarSpec(),
            .existsInDB = false,
    };
    dbEntry.spec.varId = varId;

    try {
        dbEntry = dataBase.at(varId);
    } catch (const std::out_of_range& e) {
        std::cerr << "No entry found for VarId: " << varId << std::endl;
    }

    return dbEntry;
}


VarId DatabaseManager::getNextAvailableVarId() {
    return nextVarId;
}

std::tuple<VarDisStatusCodes, ReadReturnDetails> DatabaseManager::read(VarId varId) {
    DBEntry entry = lookup(varId);
    if (not entry.existsInDB) {
        return std::make_tuple(VARDIS_STATUS_VARIABLE_DOES_NOT_EXIST, ReadReturnDetails {});
    }

    if (not entry.toBeDeleted) {
        return std::make_tuple(VARDIS_STATUS_VARIABLE_BEING_DELETED, ReadReturnDetails {});
    }

    return std::make_tuple(VARDIS_STATUS_OK, ReadReturnDetails{entry.length, entry.value, entry.tStamp});
}
