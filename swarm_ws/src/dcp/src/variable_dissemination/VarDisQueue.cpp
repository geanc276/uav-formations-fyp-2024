#include "dcp/VarDisQueue.h"


/**
 * @brief Constructs a VarDisQueue Object
 */
VarDisQueue::VarDisQueue(TString queueName) : queueName(std::move(queueName)), databaseManager(nullptr) {

}

/**
 * @brief A simple setter, sets the database instance to use
 * @param dbManager The instance of the database to use
 */
void VarDisQueue::setDatabaseManager(DatabaseManager* dbManager) {
    this->databaseManager = dbManager;
}

/**
 * @brief Prints all elements of the queue.
 *
 * This function prints all elements of the queue to the standard output stream.
 * The elements are printed separated by spaces, followed by a newline character.
 * If the queue is empty, only a newline character is printed.
 */
void VarDisQueue::printQueue() {
    std::cout << "Elements of the queue " << queueName << ": ";
    for (const auto& element : *this) {
        std::cout << element << " "; // Print each element followed by a space
    }
    std::cout << std::endl; // Print a newline after printing all elements
}

/**
 * @brief Removes and returns the first element of the queue.
 * @return The first element of the queue if the queue is not empty, otherwise returns an invalid VarId.
 */
VarId VarDisQueue::qTake() {
    if (this->empty()) {
        return VarId(0); // Return an invalid VarId if the queue is empty
    }
    VarId returnValue = this->at(0); // Get the first element
    this->erase(this->begin()); // Remove the first element
    return returnValue;
}

/**
 * @brief Returns the first element of the queue without removing it.
 * @return The first element of the queue if the queue is not empty, otherwise returns an invalid VarId.
 */
VarId VarDisQueue::qPeek() {
    if (this->empty()) {
        return VarId(0);
    }
    return this->at(0); // Return the first element
}

/**
 * @brief Appends a new element to the end of the queue.
 * @param varId The VarId to be appended to the queue.
 */
void VarDisQueue::qAppend(VarId varId) {
    this->push_back(varId);
}

/**
 * @brief Removes all occurrences of a specific VarId from the queue.
 * @param varId The VarId to be removed from the queue.
 */
void VarDisQueue::qRemove(VarId varId) {
    // Use std::remove algorithm to move all occurrences of varId to the end of the vector
    auto removeEnd = std::remove(this->begin(), this->end(), varId);

    // Erase the removed elements from the vector
    this->erase(removeEnd, this->end());
}

/**
 * @brief Checks if the queue is empty.
 * @return true if the queue is empty, false otherwise.
 */
bool VarDisQueue::qIsEmpty() {
    return this->empty(); // Return true if the queue is empty
}

/**
 * @brief Checks if a specific VarId exists in the queue.
 * @param varId The VarId to be checked.
 * @return true if varId exists in the queue, false otherwise.
 */
bool VarDisQueue::qExists(VarId varId) {
    // Use std::find to check if varId exists in the vector
    return std::find(this->begin(), this->end(), varId) != this->end();
}

/**
 * @brief Returns the number of elements in the queue.
 * @return The number of elements in the queue.
 */
size_t VarDisQueue::qLength() {
    return this->size();
}

/**
 * @brief Drops elements from the queue that don't exist in the DB or are marked as toBeDeleted in the DB.
 *
 * @param varId The VarId to be checked and possibly removed from the queue.
 * @param databaseManager The DatabaseManager instance used to look up the VarId in the database.
 */
void VarDisQueue::qDropNonexistingDeleted() {
    std::vector<VarId> variablesToRemove;
    for (const auto& varId : *this) {
        DBEntry entry = databaseManager->lookup(varId);
        if (not entry.existsInDB or entry.toBeDeleted) {
            variablesToRemove.push_back(varId);
        }
    }
    for (VarId varId: variablesToRemove) {
        std::cout << "qDropNonexistingDeleted VarId (" << varId << ") for: " << this->queueName << "\n";
        this->qRemove(varId);
    }
}

/**
 * @brief Drops elements from the queue that don't exist in the DB.
 *
 * @param varId The VarId to be checked and possibly removed from the queue.
 * @param databaseManager The DatabaseManager instance used to look up the VarId in the database.
 */
void VarDisQueue::qDropNonexisting() {
    std::vector<VarId> variablesToRemove;
    for (const auto& varId : *this) {
        DBEntry entry = databaseManager->lookup(varId);
        if (not entry.existsInDB) {
            variablesToRemove.push_back(varId);
        }
    }
    for (VarId varId: variablesToRemove) {
        std::cout << "qDropNonexisting VarId (" << varId << ") for: " << this->queueName << "\n";
        this->qRemove(varId);
    }
}

/**
 * @brief Drops elements from the queue that are marked as toBeDeleted in the DB.
 *
 * @param varId The VarId to be checked and possibly removed from the queue.
 * @param databaseManager The DatabaseManager instance used to look up the VarId in the database.
 */
void VarDisQueue::qDropDeleted() {
    std::vector<VarId> variablesToRemove;
    for (const auto& varId : *this) {
        DBEntry entry = databaseManager->lookup(varId);
        if (entry.existsInDB && entry.toBeDeleted) {
            variablesToRemove.push_back(varId);
        }
    }
    for (VarId varId: variablesToRemove) {
        std::cout << "qDropDeleted VarId (" << varId << ") for: " << this->queueName << "\n";
        this->qRemove(varId);
    }
}