#ifndef VARDISCLIENT_H
#define VARDISCLIENT_H

#include "VarDisQueue.h"
#include "beaconingDataTypes.h"
#include "rclcpp/rclcpp.hpp"
#include "dcp_msgs/msg/register_protocol_request.hpp"
#include "dcp_msgs/msg/transmit_payload_request.hpp"
#include "dcp_msgs/msg/query_number_buffered_payloads_request.hpp"
#include "dcp_msgs/msg/query_number_buffered_payloads_response.hpp"
#include "dcp_msgs/msg/payload_transmitted_indication.hpp"
#include "dcp_msgs/msg/receive_payload_indication.hpp"
#include <random>
#include <fstream>


class VarDisClient {
public:

    /**
     * @brief Constructs a VarDisClient instance
     *
     */
    VarDisClient();

    /**
     * @brief Constructs a VarDisClient instance with specified publishers.
     *
     * @param registerProtocolPublisher Publisher for sending register protocol requests.
     * @param transmitPayloadPublisher Publisher for sending transmit payload requests.
     * @param queryNumberBufferedPayloadsPublisher Publisher for sending query number buffered payload requests.
     */
    VarDisClient(rclcpp::Publisher<dcp_msgs::msg::RegisterProtocolRequest>::SharedPtr,
                 rclcpp::Publisher<dcp_msgs::msg::TransmitPayloadRequest>::SharedPtr,
                 rclcpp::Publisher<dcp_msgs::msg::QueryNumberBufferedPayloadsRequest>::SharedPtr);

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
    void processCreateIEElements(std::vector<uint8_t> rawIEByteVector);

    /**
     * @brief Processes a vector of raw bytes representing a list of IETYPE-DELETE-VARIABLES elements
     * @param rawIEByteVector A vector of bytes representing the raw IE data.
     */
    void processDeleteIEElements(std::vector<uint8_t> rawIEByteVector);

    /**
     * @brief Processes a vector of raw bytes representing a list of IETYPE-UPDATES elements
     * @param rawIEByteVector A vector of bytes representing the raw IE data.
     */
    void processUpdateIEElements(std::vector<uint8_t> rawIEByteVector);

    /**
     * @brief Processes a vector of raw bytes representing a list of IETYPE-SUMMARIES elements
     * @param rawIEByteVector A vector of bytes representing the raw IE data.
     */
    void processSummaryIEElements(std::vector<uint8_t> rawIEByteVector);

    /**
     * @brief Processes a vector of raw bytes representing a list of IETYPE-REQUEST-VARUPDATES elements
     * @param rawIEByteVector A vector of bytes representing the raw IE data.
     */
    void processRequestVarUpdsIEElements(std::vector<uint8_t> rawIEByteVector);

    /**
     * @brief Processes a vector of raw bytes representing a list of IETYPE-REQUEST-VARCREATES elements
     * @param rawIEByteVector A vector of bytes representing the raw IE data.
     */
    void processRequestVarCreatesIEElements(std::vector<uint8_t> rawIEByteVector);

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

    /**
    * @brief Sends a registration message to the Beaconing Protocol.
    *
    * This method is called during the initialization phase of the Beaconing Protocol
    * to ensure that the client registers itself properly with the protocol.
    */
    void registerClientProtocol();

    /**
    * @brief Checks the number of buffered VarDis payloads in the Beaconing Protocol.
    */
    void checkNumberBufferedPayloads();

    /**
    * @brief Processes the response to a buffered payload query.
    *
    * @param response The response message containing the number of buffered payloads.
    */
    void queryBufferResponse(const dcp_msgs::msg::QueryNumberBufferedPayloadsResponse& response);

    /**
    * @brief Handles the payload reception indication.
    *
    * @param indic The indication message received for a payload.
    */
    void receive_payload_callback_indication(const dcp_msgs::msg::ReceivePayloadIndication& indic);

    /**
    * @brief Handles the transmitted payload indication.
    *
    * @param indic The indication message received for transmitted payloads.
    */
    void transmitted_payload_callback_indication(const dcp_msgs::msg::PayloadTransmittedIndication& indic);

    /**
    * @brief Calculates the number of records that can fit in a VarDis payload.
    *
    * @param queue The VarDisQueue object to check for potential records.
    * @param recordSize The size of each record to be checked.
    * @param currentPayloadSize A pointer to a size_t variable to update with the current payload size.
    * @return The number of records that can be accommodated in the payload.
    */
    uint8_t numberFittingRecords(VarDisQueue &queue, const size_t recordSize, size_t* currentPayloadSize);

    /**
    * @brief Generates a payload for transmission.
    * @return A vector of bytes representing the generated payload.
    */
    std::vector<uint8_t> generatePayload();

    /**
    * @brief Generates a new VarDis variable
    *
    * Mainly used for testing
    */
    void generateVariable();

    /**
    * @brief Updates an existing variable.
    *
    * Mainly used for testing
    */
    void updateVariable();

    /**
    * @brief Deletes an existing variable.
    *
    * Mainly used for testing
    */
    void deleteVariable();

    /**
    * @brief Writes results to a file for analysis.
    */
    void writeResultsToFile();


private:
    /**
     * @brief The instance of the database in use
     */
    DatabaseManager databaseManager = DatabaseManager();

    /**
     * @brief Used to track information about the performance (results found in 2024 report)
     */
    std::unordered_map<std::string, double> varDisResults;

    /**
    * @brief Needed helper functions.
    */
    Helpers helpers = Helpers();

    /**
    * @brief Publishers for sending different information
    */
    rclcpp::Publisher<dcp_msgs::msg::RegisterProtocolRequest>::SharedPtr protocolRegisterPub;
    rclcpp::Publisher<dcp_msgs::msg::TransmitPayloadRequest>::SharedPtr protocolTransmitPayload;
    rclcpp::Publisher<dcp_msgs::msg::QueryNumberBufferedPayloadsRequest>::SharedPtr protocolCheckQueueBuffer;

    /**
     * @brief Used to keep track of how many packets have been sent
     */
    uint8_t varSeqNum = 0;

    /**
     * @brief A vector of functions for processing different types of information elements.
     * Each function takes a reference to a vector of bytes and processes it according
     * to the specific type of information element.
     *
     * The order is very important as it changes which elements are processed first.
     * See the provided specification for correct order
     */
     std::vector<std::function<void(std::vector<uint8_t>&)>> processFunctions = {
            [&](std::vector<uint8_t>& informationElementPayload) {
                this->processCreateIEElements(informationElementPayload);
            },
            [&](std::vector<uint8_t>& informationElementPayload) {
                this->processDeleteIEElements(informationElementPayload);
            },
            [&](std::vector<uint8_t>& informationElementPayload) {
                this->processUpdateIEElements(informationElementPayload);
            },
            [&](std::vector<uint8_t>& informationElementPayload) {
                this->processSummaryIEElements(informationElementPayload);
            },
            [&](std::vector<uint8_t>& informationElementPayload) {
                this->processRequestVarCreatesIEElements(informationElementPayload);
            },
            [&](std::vector<uint8_t>& informationElementPayload) {
                this->processRequestVarUpdsIEElements(informationElementPayload);
            }
    };

};


#endif // VARDISCLIENT_H
