#include "dcp/Helpers.h"

/**
 * * @brief Constructs a Helpers object.
 */
Helpers::Helpers() {

}

NodeIdentifier Helpers::getNodeIdentifier() {
    return Tins::NetworkInterface::default_interface().addresses().hw_addr.to_string();
}

template <typename T>
void Helpers::serializeInt(std::vector<uint8_t>& buffer, T value) {
    for (size_t i = 0; i < sizeof(T); ++i) {
        buffer.push_back(static_cast<uint8_t>((value >> (i * 8)) & 0xFF));
    }
}

void Helpers::serializeString(std::vector<uint8_t>& buffer, const std::string& str) {
    serializeInt(buffer, static_cast<uint32_t>(str.size()));
    for (char c : str) {
        buffer.push_back(static_cast<uint8_t>(c));
    }
}

void Helpers::serializeBytes(std::vector<uint8_t>& buffer, const std::vector<uint8_t>& data) {
    serializeInt<uint32_t>(buffer, static_cast<uint32_t>(data.size()));
    buffer.insert(buffer.end(), data.begin(), data.end());
}

void Helpers::serializeVarSpec(std::vector<uint8_t>& buffer, const VarSpec& spec) {
    serializeInt<VarId>(buffer, spec.varId);
    serializeString(buffer, spec.prodId);
    serializeInt<VarRepCnt>(buffer, spec.repCnt);
    serializeString(buffer, spec.descr);
}

void Helpers::serializeVarUpd(std::vector<uint8_t>& buffer, const VarUpd& upd) {
    serializeInt<VarId>(buffer, upd.varId);
    serializeInt<VarSeqNo>(buffer, upd.varSeqNo);
    serializeInt<VarLen>(buffer, upd.varLen);
    serializeBytes(buffer, upd.Value);
}

void Helpers::serializeVarCreate(std::vector<uint8_t>& buffer, const VarCreate& varCreate) {
    serializeVarSpec(buffer, varCreate.spec);
    serializeVarUpd(buffer, varCreate.upd);
}

void Helpers::serializeVarSumm(std::vector<uint8_t>& buffer, const VarSumm& varSumm) {
    serializeInt<VarId>(buffer, varSumm.varId);
    serializeInt<VarSeqNo>(buffer, varSumm.varSeqNo);
}

void Helpers::serializeSafetyData(std::vector<uint8_t>& buffer, const SafetyData& safetyData) {
    serializeInt<int16_t>(buffer, safetyData.position[0]);
    serializeInt<int16_t>(buffer, safetyData.position[1]);
    serializeInt<int16_t>(buffer, safetyData.position[2]);

    serializeInt<uint8_t>(buffer, safetyData.speed);

    serializeInt<int16_t>(buffer, safetyData.direction[0]);
    serializeInt<int16_t>(buffer, safetyData.direction[1]);
    serializeInt<int16_t>(buffer, safetyData.direction[2]);

    serializeInt<int16_t>(buffer, safetyData.rotation[0]);
    serializeInt<int16_t>(buffer, safetyData.rotation[1]);
    serializeInt<int16_t>(buffer, safetyData.rotation[2]);
}

void Helpers::serializeExtendedSafetyData(std::vector<uint8_t>& buffer, const ExtendedSafetyData& extenedSafetyData) {
    serializeSafetyData(buffer, extenedSafetyData.sData);
    serializeString(buffer, extenedSafetyData.nodeId);
    serializeInt<int64_t>(buffer, extenedSafetyData.tStamp.time.count());
    serializeInt<uint32_t>(buffer, extenedSafetyData.seqNum.sequenceNumber);
}

void Helpers::serializeInformationElement(std::vector<uint8_t>& buffer, const InformationElement& informationElement) {
    serializeInt(buffer, informationElement.ieHeader.ieType);
    serializeInt(buffer, informationElement.ieHeader.ieNumRecords);
    serializeInt<int64_t>(buffer, informationElement.ieHeader.ieTimestamp.time.count());
    serializeInt<uint8_t>(buffer, informationElement.ieHeader.ieSeqNum);
    serializeBytes(buffer, informationElement.ieList);
}

std::vector<uint8_t> Helpers::serializeVarCreateList(const std::vector<VarCreate>& varCreateList) {
    std::vector<uint8_t> buffer;
    for (const auto& varCreate : varCreateList) {
        serializeVarCreate(buffer, varCreate);
    }
    return buffer;
}

template <typename T>
T Helpers::deserializeInt(const std::vector<uint8_t>& buffer, size_t& offset) {
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

std::string Helpers::deserializeString(const std::vector<uint8_t>& buffer, size_t& offset) {
    uint32_t length = deserializeInt<uint32_t>(buffer, offset);
    if (offset + length > buffer.size()) {
        throw std::out_of_range("Buffer overflow during string deserialization");
    }
    std::string str(reinterpret_cast<const char*>(&buffer[offset]), length);
    offset += length;
    return str;
}

std::vector<uint8_t> Helpers::deserializeBytes(const std::vector<uint8_t>& buffer, size_t& offset) {
    uint32_t length = deserializeInt<uint32_t>(buffer, offset);
    if (offset + length > buffer.size()) {
        throw std::out_of_range("Buffer overflow during byte vector deserialization");
    }
    std::vector<uint8_t> data(buffer.begin() + offset, buffer.begin() + offset + length);
    offset += length;
    return data;
}

VarSpec Helpers::deserializeVarSpec(const std::vector<uint8_t>& buffer, size_t& offset) {
    VarSpec spec;
    spec.varId = deserializeInt<VarId>(buffer, offset);
    spec.prodId = deserializeString(buffer, offset);
    spec.repCnt = deserializeInt<VarRepCnt>(buffer, offset);
    spec.descr = deserializeString(buffer, offset);
    return spec;
}

VarUpd Helpers::deserializeVarUpd(const std::vector<uint8_t>& buffer, size_t& offset) {
    VarUpd upd;
    upd.varId = deserializeInt<VarId>(buffer, offset);
    upd.varSeqNo = deserializeInt<VarSeqNo>(buffer, offset);
    upd.varLen = deserializeInt<VarLen>(buffer, offset);
    upd.Value = deserializeBytes(buffer, offset);
    return upd;
}

VarCreate Helpers::deserializeVarCreate(const std::vector<uint8_t>& buffer, size_t& offset) {
    VarCreate varCreate;
    varCreate.spec = deserializeVarSpec(buffer, offset);
    varCreate.upd = deserializeVarUpd(buffer, offset);
    return varCreate;
}

VarSumm Helpers::deserializeVarSumm(const std::vector<uint8_t>& buffer, size_t& offset) {
    VarSumm varSumm;
    varSumm.varId = deserializeInt<VarId>(buffer, offset);
    varSumm.varSeqNo = deserializeInt<VarSeqNo>(buffer, offset);
    return varSumm;
}

SafetyData Helpers::deserializeSafetyData(const std::vector<uint8_t>& buffer, size_t& offset) {
    SafetyData safetyData;
    safetyData.position[0] = deserializeInt<int16_t>(buffer, offset);
    safetyData.position[1] = deserializeInt<int16_t>(buffer, offset);
    safetyData.position[2] = deserializeInt<int16_t>(buffer, offset);

    safetyData.speed = deserializeInt<uint8_t>(buffer, offset);

    safetyData.direction[0] = deserializeInt<int16_t>(buffer, offset);
    safetyData.direction[1] = deserializeInt<int16_t>(buffer, offset);
    safetyData.direction[2] = deserializeInt<int16_t>(buffer, offset);

    safetyData.rotation[0] = deserializeInt<int16_t>(buffer, offset);
    safetyData.rotation[1] = deserializeInt<int16_t>(buffer, offset);
    safetyData.rotation[2] = deserializeInt<int16_t>(buffer, offset);
    return safetyData;
}

ExtendedSafetyData Helpers::deserializeExtendedSafetyData(const std::vector<uint8_t>& buffer, size_t& offset) {
    ExtendedSafetyData extendedSafetyData;
    extendedSafetyData.sData = deserializeSafetyData(buffer, offset);
    extendedSafetyData.nodeId = deserializeString(buffer, offset);
    extendedSafetyData.tStamp = deserializeInt<int64_t>(buffer, offset);
    extendedSafetyData.seqNum.sequenceNumber = deserializeInt<uint32_t>(buffer, offset);
    return extendedSafetyData;
}

InformationElement Helpers::deserializeInformationElement(const std::vector<uint8_t>& buffer, size_t& offset) {
    InformationElement informationElement;
    informationElement.ieHeader.ieType = deserializeInt<uint8_t>(buffer, offset);
    informationElement.ieHeader.ieNumRecords = deserializeInt<uint8_t>(buffer, offset);
    informationElement.ieHeader.ieTimestamp = deserializeInt<int64_t>(buffer, offset);
    informationElement.ieHeader.ieSeqNum = deserializeInt<uint8_t>(buffer, offset);
    informationElement.ieList = deserializeBytes(buffer, offset);
    return informationElement;
}

std::vector<VarCreate> Helpers::deserializeVarCreateList(const std::vector<uint8_t>& buffer) {
    std::vector<VarCreate> varCreateList;
    size_t offset = 0;

    while (offset < buffer.size()) {
        VarCreate varCreate = deserializeVarCreate(buffer, offset);
        varCreateList.push_back(varCreate);
    }

    return varCreateList;
}

std::vector<VarUpd> Helpers::deserializeVarUpdList(const std::vector<uint8_t>& buffer) {
    std::vector<VarUpd> varUpds;
    size_t offset = 0;

    while (offset < buffer.size()) {
        VarUpd varUpd = deserializeVarUpd(buffer, offset);
        varUpds.push_back(varUpd);
    }

    return varUpds;
}

std::vector<VarSumm> Helpers::deserializeVarSummList(const std::vector<uint8_t>& buffer) {
    std::vector<VarSumm> varSumms;
    size_t offset = 0;

    while (offset < buffer.size()) {
        VarSumm varSumm = deserializeVarSumm(buffer, offset);
        varSumms.push_back(varSumm);
    }

    return varSumms;
}

std::vector<uint8_t> Helpers::deserializeVarIdList(const std::vector<uint8_t>& buffer) {
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