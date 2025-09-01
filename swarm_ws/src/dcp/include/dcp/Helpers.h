#ifndef HELPERS_H
#define HELPERS_H


#include "Types.h"
#include "srpDataTypes.h"
#include <tins/tins.h>

class Helpers {
public:
    /**
     * @brief Constructs a Helpers object
     */
    explicit Helpers();

    /**
     * @brief Gets the MAC address of the current NIC. Used as a unique identifier
     * @return NodeIdentifier (std::string) of the MAC address
     */
    NodeIdentifier getNodeIdentifier();

    /**
     * @brief Serializes an integer value into a byte buffer.
     *
     * @tparam T Type of the integer to serialize. Must be an integral type.
     * @param buffer Reference to the byte buffer where the integer will be serialized.
     * @param value The integer value to serialize.
     */
    template <typename T>
    void serializeInt(std::vector<uint8_t>& buffer, T value);

    /**
     * @brief Serializes a string into a byte buffer.
     *
     * @param buffer Reference to the byte buffer where the string will be serialized.
     * @param str The string to serialize.
     */
    void serializeString(std::vector<uint8_t>& buffer, const std::string& str);

    /**
     * @brief Serializes a byte vector into a byte buffer.
     *
     * @param buffer Reference to the byte buffer where the byte vector will be serialized.
     * @param data The byte vector to serialize.
     */
    void serializeBytes(std::vector<uint8_t>& buffer, const std::vector<uint8_t>& data);

    /**
     * @brief Serializes a VarSpec object into a byte buffer.
     *
     * @param buffer Reference to the byte buffer where the VarSpec will be serialized.
     * @param spec The VarSpec object to serialize.
     */
    void serializeVarSpec(std::vector<uint8_t>& buffer, const VarSpec& spec);

    /**
     * @brief Serializes a VarUpd object into a byte buffer.
     *
     * @param buffer Reference to the byte buffer where the VarUpd will be serialized.
     * @param upd The VarUpd object to serialize.
     */
    void serializeVarUpd(std::vector<uint8_t>& buffer, const VarUpd& upd);

    /**
     * @brief Serializes a VarCreate object into a byte buffer.
     *
     * @param buffer Reference to the byte buffer where the VarCreate will be serialized.
     * @param varCreate The VarCreate object to serialize.
     */
    void serializeVarCreate(std::vector<uint8_t>& buffer, const VarCreate& varCreate);

    /**
     * @brief Serializes a VarSumm object into a byte buffer.
     *
     * @param buffer Reference to the byte buffer where the VarSumm will be serialized.
     * @param varCreate The VarSumm object to serialize.
     */
    void serializeVarSumm(std::vector<uint8_t>& buffer, const VarSumm& varSumm);

    /**
     * @brief Serializes a SafetyData object into a byte buffer.
     *
     * @param buffer Reference to the byte buffer where the SafetyData will be serialized.
     * @param safetyData The SafetyData object to serialize.
     */
    void serializeSafetyData(std::vector<uint8_t>& buffer, const SafetyData& safetyData);

    /**
     * @brief Serializes an ExtendedSafetyData object into a byte buffer.
     *
     * @param buffer Reference to the byte buffer where the ExtendedSafetyData will be serialized.
     * @param extendedSafetyData The ExtendedSafetyData object to serialize.
     */
    void serializeExtendedSafetyData(std::vector<uint8_t>& buffer, const ExtendedSafetyData& extenedSafetyData);

    /**
     * @brief Serializes an InformationElement object into a byte buffer.
     *
     * @param buffer Reference to the byte buffer where the InformationElement will be serialized.
     * @param informationElement The InformationElement object to serialize.
     */
    void serializeInformationElement(std::vector<uint8_t>& buffer, const InformationElement& informationElement);

    /**
     * @brief Serializes a list of VarCreate objects into a byte buffer.
     *
     * @param varCreateList The list of VarCreate objects to serialize.
     * @return A byte buffer containing the serialized VarCreate objects.
     */
    std::vector<uint8_t> serializeVarCreateList(const std::vector<VarCreate>& varCreateList);

    /**
     * @brief Deserializes an integer value from a byte buffer.
     *
     * @tparam T Type of the integer to deserialize. Must be an integral type.
     * @param buffer Reference to the byte buffer from which the integer will be deserialized.
     * @param offset Reference to the current offset in the buffer, which will be updated after deserialization.
     * @return The deserialized integer value.
     */
    template <typename T>
    T deserializeInt(const std::vector<uint8_t>& buffer, size_t& offset);

    /**
     * @brief Deserializes a string from a byte buffer.
     *
     * @param buffer Reference to the byte buffer from which the string will be deserialized.
     * @param offset Reference to the current offset in the buffer, which will be updated after deserialization.
     * @return The deserialized string.
     */
    std::string deserializeString(const std::vector<uint8_t>& buffer, size_t& offset);

    /**
     * @brief Deserializes a byte vector from a byte buffer.
     *
     * @param buffer Reference to the byte buffer from which the byte vector will be deserialized.
     * @param offset Reference to the current offset in the buffer, which will be updated after deserialization.
     * @return The deserialized byte vector.
     */
    std::vector<uint8_t> deserializeBytes(const std::vector<uint8_t>& buffer, size_t& offset);

    /**
     * @brief Deserializes a VarSpec object from a byte buffer.
     *
     * @param buffer Reference to the byte buffer from which the VarSpec will be deserialized.
     * @param offset Reference to the current offset in the buffer, which will be updated after deserialization.
     * @return The deserialized VarSpec object.
     */
    VarSpec deserializeVarSpec(const std::vector<uint8_t>& buffer, size_t& offset);

    /**
     * @brief Deserializes a VarUpd object from a byte buffer.
     *
     * @param buffer Reference to the byte buffer from which the VarUpd will be deserialized.
     * @param offset Reference to the current offset in the buffer, which will be updated after deserialization.
     * @return The deserialized VarUpd object.
     */
    VarUpd deserializeVarUpd(const std::vector<uint8_t>& buffer, size_t& offset);

    /**
     * @brief Deserializes a VarCreate object from a byte buffer.
     *
     * @param buffer Reference to the byte buffer from which the VarCreate will be deserialized.
     * @param offset Reference to the current offset in the buffer, which will be updated after deserialization.
     * @return The deserialized VarCreate object.
     */
    VarCreate deserializeVarCreate(const std::vector<uint8_t>& buffer, size_t& offset);

    /**
     * @brief Deserializes a VarSumm object from a byte buffer.
     *
     * @param buffer Reference to the byte buffer from which the VarSumm will be deserialized.
     * @param offset Reference to the current offset in the buffer, which will be updated after deserialization.
     * @return The deserialized VarSumm object.
     */
    VarSumm deserializeVarSumm(const std::vector<uint8_t>& buffer, size_t& offset);

    /**
     * @brief Deserializes a SafetyData object from a byte buffer.
     *
     * @param buffer Reference to the byte buffer from which the SafetyData will be deserialized.
     * @param offset Reference to the current offset in the buffer, which will be updated after deserialization.
     * @return The deserialized SafetyData object.
     */
    SafetyData deserializeSafetyData(const std::vector<uint8_t>& buffer, size_t& offset);

    /**
     * @brief Deserializes an ExtendedSafetyData object from a byte buffer.
     *
     * @param buffer Reference to the byte buffer from which the ExtendedSafetyData will be deserialized.
     * @param offset Reference to the current offset in the buffer, which will be updated after deserialization.
     * @return The deserialized ExtendedSafetyData object.
     */
    ExtendedSafetyData deserializeExtendedSafetyData(const std::vector<uint8_t>& buffer, size_t& offset);

    /**
     * @brief Deserializes an InformationElement object from a byte buffer.
     *
     * @param buffer Reference to the byte buffer from which the InformationElement will be deserialized.
     * @param offset Reference to the current offset in the buffer, which will be updated after deserialization.
     * @return The deserialized InformationElement object.
     */
    InformationElement deserializeInformationElement(const std::vector<uint8_t>& buffer, size_t& offset);

    /**
     * @brief Deserializes a list of VarCreate objects from a byte buffer.
     *
     * @param buffer Reference to the byte buffer from which the VarCreate list will be deserialized.
     * @return The deserialized list of VarCreate objects.
     */
    std::vector<VarCreate> deserializeVarCreateList(const std::vector<uint8_t>& buffer);

    /**
     * @brief Deserializes a vector of VarUpd variables from a byte buffer.
     *
     * @param buffer Reference to the byte buffer from which the VarUpd vector will be deserialized.
     * @param numVarUpds Number of VarUpd objects to deserialize.
     * @return The deserialized vector of VarUpd variables.
     */
    std::vector<VarUpd> deserializeVarUpdList(const std::vector<uint8_t>& buffer);

    /**
     * @brief Deserializes a vector of VarSumm variables from a byte buffer.
     *
     * @param buffer Reference to the byte buffer from which the VarSumm vector will be deserialized.
     * @param numVarSumms Number of VarSumm objects to deserialize.
     * @return The deserialized vector of VarSumm variables.
     */
    std::vector<VarSumm> deserializeVarSummList(const std::vector<uint8_t>& buffer);

    /**
     * @brief Deserializes a vector of VarId (uint8_t) variables from a byte buffer.
     *
     * @param buffer Reference to the byte buffer from which the VarId vector will be deserialized.
     * @return The deserialized vector of VarId variables.
     */
    std::vector<uint8_t> deserializeVarIdList(const std::vector<uint8_t>& buffer);

    /**
     * @brief Adds a VarSumm object to the Information Element (IE) IeList.
     *
     * @param informationElement Pointer to the InformationElement object
     * @param dbEntry Pointer to the DBEntry object containing data to create the VarSumm object.
     * @param currentPayloadSize Pointer to the size_t variable tracking the current payload size.
     */
    void addVarSummToIEElement(InformationElement* informationElement, DBEntry* dbEntry, size_t* currentPayloadSize);

    /**
     * @brief Adds a VarCreate object to the Information Element (IE) IeList.
     *
     * @param informationElement Pointer to the InformationElement object
     * @param dbEntry Pointer to the DBEntry object containing data to create the VarCreate object.
     * @param currentPayloadSize Pointer to the size_t variable tracking the current payload size.
     */
    void addVarCreateToIEElement(InformationElement* informationElement, DBEntry* dbEntry, size_t* currentPayloadSize);

    /**
     * @brief Adds a VarUpdate object to the Information Element (IE) IeList.
     *
     * @param informationElement Pointer to the InformationElement object
     * @param dbEntry Pointer to the DBEntry object containing data to create the VarUpdate object.
     * @param currentPayloadSize Pointer to the size_t variable tracking the current payload size.
     */
    void addVarUpdateToIEElement(InformationElement* informationElement, DBEntry* dbEntry, size_t* currentPayloadSize);

    /**
     * @brief Adds a VarId object to the Information Element (IE) IeList.
     *
     * @param informationElement Pointer to the InformationElement object
     * @param dbEntry Pointer to the DBEntry object containing data to create the VarId object.
     * @param currentPayloadSize Pointer to the size_t variable tracking the current payload size.
     */
    void addVarIdToIEElement(InformationElement* informationElement, DBEntry* dbEntry, size_t* currentPayloadSize);

};


#endif
