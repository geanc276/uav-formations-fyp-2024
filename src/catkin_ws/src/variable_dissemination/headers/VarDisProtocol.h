#ifndef VARDISPROTOCOL_H
#define VARDISPROTOCOL_H

#include "VarDisQueue.h"

class VarDisProtocol {
public:

    /**
     * @brief Constructs a VarDisProtocol instance
     *
     */
    explicit VarDisProtocol();

    /**
     * @brief A bunch of variables for the required custom queues
     */
    VarDisQueue createQ = VarDisQueue("createQ");
    VarDisQueue deleteQ = VarDisQueue("deleteQ");
    VarDisQueue updateQ = VarDisQueue("updateQ");
    VarDisQueue summaryQ = VarDisQueue("summaryQ");
    VarDisQueue reqUpdQ = VarDisQueue("reqUpdQ");
    VarDisQueue reqCreateQ = VarDisQueue("reqCreateQ");

    /**
     * @brief Simple getter for the database instance
     * @return The database instance currently in use
     */
    DatabaseManager& getDatabaseManager();

    /**
     * @brief Processes a vector of raw bytes representing a list of IETYPE-CREATE-VARIABLES elements
     * @param rawIEByteVector A vector of bytes representing the raw IE data.
     */
    void processCreateIEElements(std::vector<std::byte> rawIEByteVector);

    /**
     * @brief Processes a vector of raw bytes representing a list of IETYPE-DELETE-VARIABLES elements
     * @param rawIEByteVector A vector of bytes representing the raw IE data.
     */
    void processDeleteIEElements(std::vector<std::byte> rawIEByteVector);

    /**
     * @brief Processes a vector of raw bytes representing a list of IETYPE-UPDATES elements
     * @param rawIEByteVector A vector of bytes representing the raw IE data.
     */
    void processUpdateIEElements(std::vector<std::byte> rawIEByteVector);

    /**
     * @brief Processes a vector of raw bytes representing a list of IETYPE-SUMMARIES elements
     * @param rawIEByteVector A vector of bytes representing the raw IE data.
     */
    void processSummaryIEElements(std::vector<std::byte> rawIEByteVector);

    /**
     * @brief Processes a vector of raw bytes representing a list of IETYPE-REQUEST-VARUPDATES elements
     * @param rawIEByteVector A vector of bytes representing the raw IE data.
     */
    void processRequestVarUpdsIEElements(std::vector<std::byte> rawIEByteVector);

    /**
     * @brief Processes a vector of raw bytes representing a list of IETYPE-REQUEST-VARCREATES elements
     * @param rawIEByteVector A vector of bytes representing the raw IE data.
     */
    void processRequestVarCreatesIEElements(std::vector<std::byte> rawIEByteVector);

    /**
    * @brief Composes an InformationElement object containing zero or more VarSumm objects for the VarDis payload.
    *
    * @param currentPayloadSize Pointer to the size_t variable tracking the current payload size.
    * @return An InformationElement object containing serialized VarSumm objects if successful; otherwise, an empty InformationElement.
    */
    InformationElement composeSummaryIEElements(size_t* currentPayloadSize);
    /**
    * @brief Composes an InformationElement object containing zero or more VarCreate objects for the VarDis payload.
    *
    * @param currentPayloadSize Pointer to the size_t variable tracking the current payload size.
    * @return An InformationElement object containing serialized VarSumm objects if successful; otherwise, an empty InformationElement.
    */
    InformationElement composeCreateVariablesIEElements(size_t* currentPayloadSize);
    /**
    * @brief Composes an InformationElement object containing zero or more VarUpd objects for the VarDis payload.
    *
    * @param currentPayloadSize Pointer to the size_t variable tracking the current payload size.
    * @return An InformationElement object containing serialized VarSumm objects if successful; otherwise, an empty InformationElement.
    */
    InformationElement composeUpdateIEElements(size_t* currentPayloadSize);
    /**
    * @brief Composes an InformationElement object containing zero or more VarId objects for the VarDis payload.
    *
    * @param currentPayloadSize Pointer to the size_t variable tracking the current payload size.
    * @return An InformationElement object containing serialized VarSumm objects if successful; otherwise, an empty InformationElement.
    */
    InformationElement composeDeleteVariablesIEElements(size_t* currentPayloadSize);
    /**
    * @brief Composes an InformationElement object containing zero or more VarSumm objects for the VarDis payload.
    *
    * @param currentPayloadSize Pointer to the size_t variable tracking the current payload size.
    * @return An InformationElement object containing serialized VarSumm objects if successful; otherwise, an empty InformationElement.
    */
    InformationElement composeRequestVarUpdatesIEElements(size_t* currentPayloadSize);
    /**
    * @brief Composes an InformationElement object containing zero or more VarId objects for the VarDis payload.
    *
    * @param currentPayloadSize Pointer to the size_t variable tracking the current payload size.
    * @return An InformationElement object containing serialized VarSumm objects if successful; otherwise, an empty InformationElement.
    */
    InformationElement composeRequestVarCreatesIEElements(size_t* currentPayloadSize);

private:
    /**
     * @brief The instance of the database in use
     */
    DatabaseManager databaseManager = DatabaseManager();

    /**
     * @brief Needed helper functions
     */
    Helpers helpers = Helpers();

};


#endif // VARDISPROTOCOL_H
