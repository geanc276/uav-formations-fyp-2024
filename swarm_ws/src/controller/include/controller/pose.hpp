/*
 *   This is a generic pose class. It handles and contains all 
 *   of the pose information for a pose represented a cartesian frame.   
 *   
 *   File: pose.hpp
 *   Date: 6-8-2024
 *   Author: Benjamin Ireland
 */

#ifndef POSE_HPP
#define POSE_HPP

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <iostream>
#include <iomanip>
#include <vector>
#include <nlohmann/json.hpp>

class Pose
{
public:
    Pose();
    Pose(std::string name);
    
    Pose(double x, double y, double z);
    Pose(double x, double y, double z, std::string name);
    Pose(double x, double y, double z, double heading);
    Pose(double x, double y, double z, double heading, std::string name);
    ~Pose();

    void set_position(double x, double y, double z);
    void set_heading(double heading);
    void set_heading(Pose &heading_vector);
    void set_terrain_offset(double terrain_offset);
    std::vector<double> get_position() const;
    double get_heading() const;
    double get_terrain_offset() const;


    // Pose assumes coordinate frame of form: [x, y, z]
    std::vector<double> position_;
    double heading_;
    // On initialisation drones will have an altitude offset above sea level.
    double terrain_offset_;

    friend std::ostream &operator<<(std::ostream &os, const Pose &pos);
    friend Pose operator-(Pose self, const Pose &other);
    friend Pose operator+(Pose self, const Pose &other);


    nlohmann::json jsonify() const;
private:
    std::string type = "pose";
protected:
    std::string name_;
};
#endif // POSE_HPP
