#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <string>
#include <chrono>
#include <ctime>

// Zero terminated string
struct TString {
    std::string data;
};

// WiFi MAC address identifier
struct NodeIdentifier {
    char identifier[18];
};

// Time with 1ms resolution
struct TimeStamp {
    std::chrono::duration<double> time;
};

#endif