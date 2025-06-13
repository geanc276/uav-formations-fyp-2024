#include "headers/Helpers.h"
#include <vector>
#include <string>
#include <cstdint>

/**
 * * @brief Constructs a Helpers object.
 */
Helpers::Helpers() {

}

/**
 * @brief Gets the MAC address of the current NIC. Used as a unique identifier
 * @return NodeIdentifier (std::string) of the MAC address
 */
NodeIdentifier Helpers::getNodeIdentifier() {
    return Tins::NetworkInterface::default_interface().addresses().hw_addr.to_string();
}

// Helper function to serialize an integer to bytes
template <typename T>
void Helpers::serializeInt(std::vector<std::byte>& buffer, T value) {
    for (size_t i = 0; i < sizeof(T); ++i) {
        buffer.push_back(static_cast<std::byte>((value >> (i * 8)) & 0xFF));
    }
}

// Helper function to serialize a string to bytes
void Helpers::serializeString(std::vector<std::byte>& buffer, const std::string& str) {
    serializeInt(buffer, static_cast<uint32_t>(str.size()));
    for (char c : str) {
        buffer.push_back(static_cast<std::byte>(c));
    }
}

// Helper function to serialize a byte vector
void Helpers::serializeBytes(std::vector<std::byte>& buffer, const std::vector<std::byte>& data) {
    serializeInt(buffer, static_cast<uint32_t>(data.size()));
    buffer.insert(buffer.end(), data.begin(), data.end());
}

void Helpers::serializeVarSpec(std::vector<std::byte>& buffer, const VarSpec& spec) {
    serializeInt(buffer, spec.varId);
    serializeString(buffer, spec.prodId);
    serializeInt(buffer, spec.repCnt);
    serializeString(buffer, spec.descr);
}

void Helpers::serializeVarUpd(std::vector<std::byte>& buffer, const VarUpd& upd) {
    serializeInt(buffer, upd.varId);
    serializeInt(buffer, upd.varSeqNo);
    serializeInt(buffer, upd.varLen);
    serializeBytes(buffer, upd.Value);
}

void Helpers::serializeVarCreate(std::vector<std::byte>& buffer, const VarCreate& varCreate) {
    serializeVarSpec(buffer, varCreate.spec);
    serializeVarUpd(buffer, varCreate.upd);
}

void Helpers::serializeVarSumm(std::vector<std::byte>& buffer, const VarSumm& varSumm) {
    serializeInt(buffer, varSumm.varId);
    serializeInt(buffer, varSumm.varSeqNo);
}

void Helpers::serializeInformationElement(std::vector<std::byte>& buffer, const InformationElement& informationElement) {
    serializeInt(buffer, informationElement.ieType);
    serializeInt(buffer, informationElement.ieLen);
    serializeBytes(buffer, informationElement.ieList);
}

std::vector<std::byte> Helpers::serializeVarCreateList(const std::vector<VarCreate>& varCreateList) {
    std::vector<std::byte> buffer;
    for (const auto& varCreate : varCreateList) {
        serializeVarCreate(buffer, varCreate);
    }
    return buffer;
}

// Helper function to deserialize an integer from bytes
template <typename T>
T Helpers::deserializeInt(const std::vector<std::byte>& buffer, size_t& offset) {
    static_assert(std::is_integral<T>::value, "deserializeInt can only deserialize integer types");
    if (offset + sizeof(T) > buffer.size()) {
        throw std::out_of_range("Buffer overflow during integer deserialization");
    }
    T value = 0;
    for (size_t i = 0; i < sizeof(T); ++i) {
        value |= (static_cast<T>(buffer[offset + i]) << (i * 8));
    }
    offset += sizeof(T);
    return value;
}

// Helper function to deserialize a string from bytes
std::string Helpers::deserializeString(const std::vector<std::byte>& buffer, size_t& offset) {
    uint32_t length = deserializeInt<uint32_t>(buffer, offset);
    if (offset + length > buffer.size()) {
        throw std::out_of_range("Buffer overflow during string deserialization");
    }
    std::string str(reinterpret_cast<const char*>(&buffer[offset]), length);
    offset += length;
    return str;
}

// Helper function to deserialize a byte vector from bytes
std::vector<std::byte> Helpers::deserializeBytes(const std::vector<std::byte>& buffer, size_t& offset) {
    uint32_t length = deserializeInt<uint32_t>(buffer, offset);
    if (offset + length > buffer.size()) {
        throw std::out_of_range("Buffer overflow during byte vector deserialization");
    }
    std::vector<std::byte> data(buffer.begin() + offset, buffer.begin() + offset + length);
    offset += length;
    return data;
}

VarSpec Helpers::deserializeVarSpec(const std::vector<std::byte>& buffer, size_t& offset) {
    VarSpec spec;
    spec.varId = deserializeInt<VarId>(buffer, offset);
    spec.prodId = deserializeString(buffer, offset);
    spec.repCnt = deserializeInt<VarRepCnt>(buffer, offset);
    spec.descr = deserializeString(buffer, offset);
    return spec;
}

VarUpd Helpers::deserializeVarUpd(const std::vector<std::byte>& buffer, size_t& offset) {
    VarUpd upd;
    upd.varId = deserializeInt<VarId>(buffer, offset);
    upd.varSeqNo = deserializeInt<VarSeqNo>(buffer, offset);
    upd.varLen = deserializeInt<VarLen>(buffer, offset);
    upd.Value = deserializeBytes(buffer, offset);
    return upd;
}

VarCreate Helpers::deserializeVarCreate(const std::vector<std::byte>& buffer, size_t& offset) {
    VarCreate varCreate;
    varCreate.spec = deserializeVarSpec(buffer, offset);
    varCreate.upd = deserializeVarUpd(buffer, offset);
    return varCreate;
}

VarSumm Helpers::deserializeVarSumm(const std::vector<std::byte>& buffer, size_t& offset) {
    VarSumm varSumm;
    varSumm.varId = deserializeInt<VarId>(buffer, offset);
    varSumm.varSeqNo = deserializeInt<VarSeqNo>(buffer, offset);
    return varSumm;
}

InformationElement Helpers::deserializeInformationElement(const std::vector<std::byte>& buffer, size_t& offset) {
    InformationElement informationElement;
    informationElement.ieType = deserializeInt<uint8_t>(buffer, offset);
    informationElement.ieLen = deserializeInt<uint8_t>(buffer, offset);
    informationElement.ieList = deserializeBytes(buffer, offset);
    return informationElement;
}

std::vector<VarCreate> Helpers::deserializeVarCreateList(const std::vector<std::byte>& buffer) {
    std::vector<VarCreate> varCreateList;
    size_t offset = 0;

    while (offset < buffer.size()) {
        VarCreate varCreate = deserializeVarCreate(buffer, offset);
        varCreateList.push_back(varCreate);
    }

    return varCreateList;
}

std::vector<VarUpd> Helpers::deserializeVarUpdList(const std::vector<std::byte>& buffer) {
    std::vector<VarUpd> varUpds;
    size_t offset = 0;

    while (offset < buffer.size()) {
        VarUpd varUpd = deserializeVarUpd(buffer, offset);
        varUpds.push_back(varUpd);
    }

    return varUpds;
}

std::vector<VarSumm> Helpers::deserializeVarSummList(const std::vector<std::byte>& buffer) {
    std::vector<VarSumm> varSumms;
    size_t offset = 0;

    while (offset < buffer.size()) {
        VarSumm varSumm = deserializeVarSumm(buffer, offset);
        varSumms.push_back(varSumm);
    }

    return varSumms;
}

std::vector<uint8_t> Helpers::deserializeVarIdList(const std::vector<std::byte>& buffer) {
    std::vector<uint8_t> varIds;

    for (size_t i = 0; i < buffer.size(); ++i) {
        varIds.push_back(static_cast<uint8_t>(buffer[i]));
    }

    return varIds;
}

void Helpers::addVarSummToIEElement(InformationElement* informationElement, DBEntry* dbEntry, size_t* currentPayloadSize) {
    VarSumm varSumm = VarSumm(dbEntry);
    *currentPayloadSize += varSumm.Size();
    serializeVarSumm(informationElement->ieList, varSumm);
}

void Helpers::addVarCreateToIEElement(InformationElement* informationElement, DBEntry* dbEntry, size_t* currentPayloadSize) {
    VarCreate varCreate = VarCreate(dbEntry);
    *currentPayloadSize += varCreate.Size();
    serializeVarCreate(informationElement->ieList, varCreate);
}

void Helpers::addVarUpdateToIEElement(InformationElement* informationElement, DBEntry* dbEntry, size_t* currentPayloadSize) {
    VarUpd varUpd = VarUpd(dbEntry);
    *currentPayloadSize += varUpd.Size();
    serializeVarUpd(informationElement->ieList, varUpd);
}

void Helpers::addVarIdToIEElement(InformationElement* informationElement, DBEntry* dbEntry, size_t* currentPayloadSize) {
    VarId varId = dbEntry->spec.varId;
    *currentPayloadSize += sizeof(varId);
    serializeInt(informationElement->ieList, varId);
}