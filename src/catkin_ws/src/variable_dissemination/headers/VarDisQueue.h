#ifndef VARDISQUEUE_H
#define VARDISQUEUE_H

#include <utility>
#include <queue>
#include "Types.h"
#include "DatabaseManager.h"

class DatabaseManager;

class VarDisQueue: public std::vector<VarId> {
public:

    /**
     * @brief Constructs a VarDisQueue Object
     */
    explicit VarDisQueue(TString queueName);

    /**
     * @brief Drops elements from the queue that don't exist in the DB or are marked as toBeDeleted in the DB.
     */
    void qDropNonexistingDeleted();

    /**
     * @brief Drops elements from the queue that don't exist in the DB.
     */
    void qDropNonexisting();

/**
     * @brief Removes and returns the first element of the queue.
     * @return The first element of the queue if the queue is not empty, otherwise returns an invalid VarId.
     */
    VarId qTake();

    /**
     * @brief Returns the first element of the queue without removing it.
     * @return The first element of the queue if the queue is not empty, otherwise returns an invalid VarId.
     */
    VarId qPeek();

    /**
     * @brief Appends a new element to the end of the queue.
     * @param varId The VarId to be appended to the queue.
     */
    void qAppend(VarId varId);

    /**
     * @brief Removes all occurrences of a specific VarId from the queue.
     * @param varId The VarId to be removed from the queue.
     */
    void qRemove(VarId varId);

    /**
     * @brief Checks if the queue is empty.
     * @return true if the queue is empty, false otherwise.
     */
    bool qIsEmpty();

    /**
     * @brief Checks if a specific VarId exists in the queue.
     * @param varId The VarId to be checked.
     * @return true if varId exists in the queue, false otherwise.
     */
    bool qExists(VarId varId);

    /**
     * @brief Returns the number of elements in the queue.
     * @return The number of elements in the queue.
     */
    size_t qLength();

    /**
     * @brief Prints all elements of the queue.
     *
     * This function prints all elements of the queue to the standard output stream.
     * The elements are printed separated by spaces, followed by a newline character.
     * If the queue is empty, only a newline character is printed.
     */
    void printQueue();

    /**
     * @brief A simple setter, sets the database instance to use
     * @param dbManager The instance of the database to use
     */
    void setDatabaseManager(DatabaseManager* dbManager);

private:
    /**
     * @brief Manually set identifier.
     * Used in log statements so it's easier to follow what is going on
     */
    TString queueName;

    /**
     * @brief The instance of the database in use
     */
    DatabaseManager* databaseManager;
};

#endif
