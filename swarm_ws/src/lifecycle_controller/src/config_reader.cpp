#include "lifecycle_controller/config_reader.hpp"
#include <fstream>
#include <iostream>
#include "logging/logging.hpp"

ConfigReader::ConfigReader(const std::string &filename) : filename_(filename)
{
    readConfig();
}

bool ConfigReader::readConfig()
{
    std::ifstream file(filename_);
    if (!file)
    {
        LOG_SETUP_GLOBAL_FATAL( "Unable to open file: %s", filename_.c_str());
        throw std::invalid_argument("Could not open file");

    }

    LOG_SETUP_GLOBAL_DEBUG( "Opened file: %s", filename_.c_str());
    nlohmann::json config;


    try
    {
        file >> config;
    }
    catch (nlohmann::json::parse_error &e)
    {
        LOG_SETUP_GLOBAL_FATAL( "Error parsing JSON file: %s", e.what());
        throw std::invalid_argument("Error parsing JSON file: " + std::string(e.what()));
    }

    try
    {
        // Position Constraints
        id_ = config.at("id").get<int>();
        name_ = "drone_" + std::to_string(id_);
        takeoff_height_ = config.at("takeoff_height").get<double>();
    }
    catch (nlohmann::json::exception &e)
    {
        LOG_SETUP_GLOBAL_FATAL( "Error accessing JSON fields: %s", e.what());
        throw std::invalid_argument("Error accessing JSON fields: " + std::string(e.what()));
    }
    return true;
}

std::string ConfigReader::getName()
{
    return name_;
}

double ConfigReader::get_takeoff_height()
{
    return takeoff_height_;
}

int ConfigReader::get_id()
{
    return id_;
}