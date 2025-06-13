#ifndef CONFIG_READER_HPP
#define CONFIG_READER_HPP

#include <string>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

class ConfigReader
{
public:
    ConfigReader(const std::string &filename);
    bool readConfig();
    std::string getName();
    double get_takeoff_height();
    int get_id();
    // Add other member functions as needed

private:
    std::string filename_;
    nlohmann::json config_;

    int id_;
    std::string name_;
    double takeoff_height_;
};

#endif // CONFIG_READER_HPP