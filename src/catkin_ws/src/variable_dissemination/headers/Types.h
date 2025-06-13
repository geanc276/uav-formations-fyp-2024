#ifndef TYPES_H
#define TYPES_H

#include <iostream>
#include <iomanip> // For std::hex, std::setw, std::setfill
#include <utility>
#include <vector>
#include <string>
#include <cstdint> // For std::byte
#include <chrono> // For std::chrono::system_clock::now()
#include <ctime>  // For std::ctime() and To Convert time_t to a human-readable string

#define VARDISPAR_MAX_VALUE_LENGTH 32
#define VARDISPAR_MAX_DESCRIPTION_LENGTH 32
#define VARDISPAR_MAX_REPITITIONS 5
#define VARDISPAR_MAX_PAYLOAD_SIZE 1024
#define VARDISPAR_BUFFER_CHECK_PERIOD 5
#define VARDISPAR_MAX_SUMMARIES 5

// Define data types
using VarId = uint8_t;
using VarLen = uint8_t;
using VarRepCnt = uint8_t;
using VarSeqNo = uint8_t;
using NodeIdentifier = std::string;  // Assuming NodeIdentifier as a 16-bit unsigned integer
using TString = std::string;  // Assuming NodeIdentifier as a 16-bit unsigned integer


enum VarDisStatusCodes {
    VARDIS_STATUS_OK,
    VARDIS_STATUS_VARIABLE_DOES_NOT_EXIST,
    VARDIS_STATUS_VARIABLE_EXISTS,
    VARDIS_STATUS_VARIABLE_BEING_DELETED,
    VARDIS_STATUS_VALUE_TOO_LONG,
    VARDIS_STATUS_INVALID_VALUE,
    VARDIS_STATUS_VARIABLE_DESCRIPTION_TOO_LONG,
    VARDIS_STATUS_ILLEGAL_REPCOUNT,
    VARDIS_STATUS_NOT_PRODUCER
};

enum InformationElements {
    IETYPE_SUMMARIES,
    IETYPE_CREATE_VARIABLES,
    IETYPE_UPDATES,
    IETYPE_DELETE_VARIABLES,
    IETYPE_REQUEST_VARUPDATES,
    IETYPE_REQUEST_VARCREATES,
};

struct InformationElement {
    uint8_t ieType;
    uint8_t ieLen;
    std::vector<std::byte> ieList;

    [[nodiscard]] size_t calculateTotalSize() const {
        size_t size = sizeof(ieType) + sizeof(ieLen);
        size += ieList.size();
        return size;
    }
};

struct TimeStamp {
    std::chrono::milliseconds time;

    // Constructor to initialize the time using current system time
    TimeStamp() : time(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch())) {}

    // Conversion operator from integer to TimeStamp
    TimeStamp(std::int64_t milliseconds) : time(milliseconds) {}
};

// Structure to hold variables for returning in the DatabaseManager:read() function
struct ReadReturnDetails {
    VarLen length;
    std::vector<std::byte> value;
    TimeStamp timeStamp;
};


// Struct representing a Variable Specification
struct VarSpec {
    VarId varId;      // Identifier of the variable
    NodeIdentifier prodId;     // Product identifier associated with the variable
    VarRepCnt repCnt;     // Repetition count of the variable
    TString descr; // Description of the variable

    [[nodiscard]] size_t Size() const {
        size_t size = sizeof(varId) + sizeof(repCnt);
        size += prodId.size() + 1;  // +1 for the null-terminator
        size += descr.size() + 1;   // +1 for the null-terminator
        return size;
    }

    VarSpec(VarId id, NodeIdentifier prod, VarRepCnt count, TString description)
            : varId(id), prodId(std::move(prod)), repCnt(count), descr(std::move(description)) {
    }

    VarSpec() {
        varId = 0;
        prodId = "N/A";
        repCnt = 0;
        descr = "N/A";
    }

};

struct DBEntry {
    VarSpec spec;
    VarLen length;
    std::vector<std::byte> value;
    VarSeqNo seqno;
    TimeStamp tStamp;
    VarRepCnt countUpdate;
    VarRepCnt countCreate;
    VarRepCnt countDelete;
    bool toBeDeleted;
};

// Struct representing a Variable Update
struct VarUpd {
    VarId varId;                      // Identifier of the variable
    VarSeqNo varSeqNo;                   // Sequence number of the update
    VarLen varLen;                     // Length of the update
    std::vector<std::byte> Value;   // Value of the update as a vector of bytes

    [[nodiscard]] size_t Size() const {
        size_t size = sizeof(varId) + sizeof(varSeqNo) + sizeof(varLen);
        size += Value.size();
        return size;
    }

    // Constructor to initialize with explicit parameters
    VarUpd(VarId id, VarSeqNo seqNo, VarLen length, std::vector<std::byte> value)
            : varId(id), varSeqNo(seqNo), varLen(length), Value(std::move(value)) {
    }

    VarUpd() {
        varId = 0;
        varSeqNo = 0;
        varLen = 0;
        Value = {};
    }

    explicit VarUpd(DBEntry* dbEntry) {
        varId = dbEntry->spec.varId;
        varSeqNo = dbEntry->seqno;
        varLen = dbEntry->length;
        Value = dbEntry->value;
    }
};

// Struct representing both a Variable Specification and its corresponding Update
struct VarCreate {
    VarSpec spec;   // Specification of the variable
    VarUpd upd;     // Update of the variable

    [[nodiscard]] size_t Size() const {
        size_t size = spec.Size();
        size += upd.Size();
        return size;
    }

    // Constructor to initialize with explicit parameters
    VarCreate(VarSpec  specification, VarUpd  update)
            : spec(std::move(specification)), upd(std::move(update)) {
    }

    VarCreate() {
        spec = VarSpec();
        upd = VarUpd();
    }

    // Constructor from DBEntry
    explicit VarCreate(DBEntry* dbEntry)
            : upd(dbEntry) {
        spec = dbEntry->spec;
    }

};

// Struct representing a Variable Summary
struct VarSumm {
    VarId varId;      // Identifier of the variable
    VarSeqNo varSeqNo;   // Sequence number of the variable

    size_t Size() {
        return sizeof(varId) + sizeof(varSeqNo);
    }

    VarSumm() {
        varId = 0;
        varSeqNo = 0;
    }

    explicit VarSumm(DBEntry* dbEntry) {
        varId = dbEntry->spec.varId;
        varSeqNo = dbEntry->seqno;
    }
};

inline std::ostream& operator<<(std::ostream& os, const uint8_t& number) {
    os << static_cast<int>(number);
    return os;
}

// Overload the stream insertion operator for the VarSpec Specification struct, allows for printing
inline std::ostream& operator<<(std::ostream& os, const VarSpec& varSpec) {
    os << "VarId: " << varSpec.varId
       << ", ProdId: " << varSpec.prodId
       << ", RepCnt: " << varSpec.repCnt
       << ", Descr: " << varSpec.descr;
    return os;
}

// Overload the stream insertion operator for the VarSpec Specification struct, allows for printing
inline std::ostream& operator<<(std::ostream& os, const VarSumm& varSumm) {
    os << "VarId: " << varSumm.varId
       << ", VarSeqNo: " << varSumm.varSeqNo;
    return os;
}

// Overload the stream insertion operator for the VarUpd Update struct, allows for printing
inline std::ostream& operator<<(std::ostream& os, const VarUpd& varUpd) {
    os << "VarId: " << varUpd.varId << ", "
       << "VarSeqNo: " << varUpd.varSeqNo << ", "
       << "VarLen: " << varUpd.varLen << ", "
       << "Value: [";

    // Print each byte in the vector as hexadecimal
    for (const auto& byte : varUpd.Value) {
        os << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    os << "]";
    std::cout << std::dec; // Reset to decimal format

    os << std::dec;

    return os;
}

// Overload the stream insertion operator for the VarCreate Specification struct, allows for printing
inline std::ostream& operator<<(std::ostream& os, const VarCreate& varCreate) {
    os << "VarSpec: {" << varCreate.spec << "}, VarUpd: {" << varCreate.upd << "}";
    return os;
}

// Overloaded operator<< for printing TimeStamp
inline std::ostream& operator<<(std::ostream& os, const TimeStamp& ts) {
    return os << ts.time.count();
}

// Overloaded operator<< for printing DBEntry
inline std::ostream& operator<<(std::ostream& os, const DBEntry& entry) {
    os << "VarSpec: " << entry.spec << ", "
       << "VarLen: " << entry.length << ", "
       << "Value: [";

    // Print each byte in the vector as hexadecimal
    for (const auto& byte : entry.value) {
        os << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    os << "]" << ", ";
    std::cout << std::dec; // Reset to decimal format

    os << std::dec;
    os << "VarSeqNo: " << entry.seqno << ", "
       << "TimeStamp: " << entry.tStamp << ", "
       << "CountUpdate: " << entry.countUpdate << ", "
       << "CountCreate: " << entry.countCreate << ", "
       << "CountDelete: " << entry.countDelete << ", "
       << "ToBeDeleted: " << entry.toBeDeleted;
    return os;
}

// Overload the stream insertion operator for the VarUpd Update struct, allows for printing
inline std::ostream& operator<<(std::ostream& os, const InformationElement& informationElement) {
    os << "ieType: " << informationElement.ieType << ", "
       << "ieLen: " << informationElement.ieLen << ", "
       << "ieList: [";

    // Print each byte in the vector as hexadecimal
    for (const auto& byte : informationElement.ieList) {
        os << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    os << "]";
    std::cout << std::dec; // Reset to decimal format

    os << std::dec;
    return os;
}


#endif //TYPES_H
