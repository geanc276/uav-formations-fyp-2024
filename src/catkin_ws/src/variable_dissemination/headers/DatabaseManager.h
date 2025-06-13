#ifndef DATABASE_MANAGER_H
#define DATABASE_MANAGER_H

#include <sqlite3.h>
#include <string>
#include <vector>
#include <cxxabi.h>
#include "Types.h"
#include "Helpers.h"
#include <tins/tins.h>
#include "VarDisQueue.h"
#include <tbb/concurrent_unordered_map.h>

class VarDisQueue;

/**
 * @class DatabaseManager
 * @brief Manages SQLite database operations such as opening, executing SQL queries, and managing tables.
 */
class DatabaseManager {
public:
    /**
     * @brief Constructs a DatabaseManager object
     */
    explicit DatabaseManager();

    /**
     * @brief Sets the VarDisQueue pointers for the DatabaseManager to use
     * This function assigns the provided VarDisQueue pointers to the corresponding member variables
     * within the DatabaseManager. These queues will be used by the DatabaseManager to perform various
     * operations related to queue management.
     * @param createQ Pointer to the VarDisQueue
     * @param deleteQ Pointer to the VarDisQueue
     * @param updateQ Pointer to the VarDisQueue
     * @param summaryQ Pointer to the VarDisQueue
     * @param reqUpdQ Pointer to the VarDisQueue
     * @param reqCreateQ Pointer to the VarDisQueue
     */
    void setQueues(VarDisQueue* createQ, VarDisQueue* deleteQ, VarDisQueue* updateQ,
                   VarDisQueue* summaryQ, VarDisQueue* reqUpdQ, VarDisQueue* reqCreateQ);

    /**
     * @brief Destructor for the DatabaseManager class.
     */
    ~DatabaseManager();

    /**
     * @brief Simple getter for the database instance
     * @return The database instance currently in use
     */
    tbb::concurrent_unordered_map<VarId, DBEntry> getDatabaseInstance();

    /**
     * @brief Create a DBEntry record into the database.
     * @param spec A VarSpec structure containing information about the product ID, repetition count, and description.
     * @param length The length of the variable, corresponding to the size of the binary data stored in the 'Value' column.
     * @param value A vector of bytes (std::byte) containing the binary data to be stored in the database.
     * @return The appropriate VARDIS_STATUS code
     */
    int create(VarSpec spec, VarLen length, std::vector<std::byte> value);

    /**
     * @brief Updates a DBEntry record in the database.
     * @param VarId The unique identifier of the DBEntry record to update.
     * @param VarLen The length of the variable.
     * @param VarRepCnt The repetition count of the variable.
     * @param VarSeqNo The sequence number of the variable. This is used as the identifier for the record to update.
     * @return True if the update is successful, otherwise false.
     */
    bool updateDBEntryById(VarId varId, VarLen varLen, const std::vector<std::byte>& value);

    /**
     * @brief Updates a DBEntry record in the database.
     * @param DBEntry The information of the DBEntry record to update.
     */
    bool update(DBEntry dbEntry);

    /**
     * @brief Prepares to delete a DBEntry record from the database.
     * @param VarId The unique identifier of the DBEntry record to delete.
     * @return The appropriate VARDIS_STATUS code
     */
    bool deleteDBEntryById(VarId varId);

    /**
     * @brief Deletes a DBEntry record from the database.
     * @param VarId The unique identifier of the DBEntry record to delete.
     */
    void remove(VarId varId);

    /**
     * @brief Retrieves all DBEntry records from the database.
     * @return A vector containing all DBEntry records in the database.
     */
    std::vector<DBEntry> getAllDBEntries();

    /**
     * @brief Retrieves a DBEntry from the database based on its VarId.
     * @param VarId The identifier of the DBEntry to retrieve.
     * @return The Variable object if found, otherwise an empty Variable object.
     */
    DBEntry lookup(int VarId);

    /**
     * @brief Gets the next available VarId (unique identifier) from the database
     *        For example if the highest VarId in the database is currently 9, this will return 10.
     *
     * @return The VarId
     */
    VarId getNextAvailableVarId();

    /**
     * @brief Reads the details of a variable from the database.
     *
     * @param varId The VarId of the variable to read.
     * @return A tuple containing the status code and the details of the variable.
     *         - If the variable does not exist, returns VARDIS_STATUS_VARIABLE_DOES_NOT_EXIST along with an empty ReadReturnDetails.
     *         - If the variable is being deleted, returns VARDIS_STATUS_VARIABLE_BEING_DELETED along with an empty ReadReturnDetails.
     *         - If the read operation is successful, returns VARDIS_STATUS_OK along with the details of the variable (length, value, timestamp).
     */
    std::tuple<VarDisStatusCodes, ReadReturnDetails>  read(VarId varId);

    /**
     * @brief Template function to print the results of database queries.
     * @tparam T The type of data to print.
     * @param data The vector containing the data to print.
     */
    template<typename T>
    void printResults(const std::vector<T>& data);

private:
    tbb::concurrent_unordered_map<VarId, DBEntry> dataBase;
    VarId nextVarId = 1;
    std::shared_mutex mapMutex;

    /**
     * @brief A bunch of pointer variables to the custom queues
     */
    VarDisQueue* createQ;
    VarDisQueue* deleteQ;
    VarDisQueue* updateQ;
    VarDisQueue* summaryQ;
    VarDisQueue* reqUpdQ;
    VarDisQueue* reqCreateQ;
};

/**
  * @brief Template function to print the results of database queries.
  * @tparam T The type of data to print.
  * @param data The vector containing the data to print.
  */
template<typename T>
void DatabaseManager::printResults(const std::vector<T>& data) {
    int status;
    std::unique_ptr<char, void (*)(void*)> res{
            abi::__cxa_demangle(typeid(T).name(), NULL, NULL, &status),
            std::free
    };

    std::string typeName = (status == 0) ? res.get() : typeid(T).name();

    // Print the type of the data
    std::cout << typeName << " Results: ";
    for (size_t i = 0; i < data.size(); ++i) {
        std::cout << "{" << data[i] << "}";
        if (i != data.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << std::endl;
}

#endif // DATABASE_MANAGER_H



