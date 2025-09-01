/*
 *   This is a derived class of the Pose class. It handles and contains all 
 *   of the pose information for a pose represented by GPS coordinates in a global
 *   context.
 *   
 *   File: global_pose.hpp
 *   Date: 6-8-2024
 *   Author: Benjamin Ireland
 */

#ifndef GLOBAL_POSE_HPP
#define GLOBAL_POSE_HPP

#include <iostream>
#include <algorithm>
#include <math.h>
#include <stdexcept>
#include "controller/pose.hpp"

// MACROS //

#define STRAIGHT_ANGLE 180
#define COMPLETE_ANGLE 360

constexpr double EARTHS_RADIUS = 6378137.0;
constexpr double FLATTENING = 1 / 298.257223563;

#define DEG_TO_RAD(degrees) (degrees * M_PI / STRAIGHT_ANGLE)
#define RAD_TO_DEG(radians) (radians * STRAIGHT_ANGLE / M_PI)

class GlobalPose : public Pose
{
private:
    std::vector<int> variances_;
    std::string type = "global";
public:
    // Constructors
    GlobalPose();

    GlobalPose(std::string name);

    GlobalPose(double lat, double lng, double alt);

    GlobalPose(double lat, double lng, double alt, int var[3]);

    // Friend function to print user-friendly GlobalPose
    friend std::ostream &operator<<(std::ostream &os, const GlobalPose &coord);

    // Static method to create a GlobalPose from NMEA strings
    static GlobalPose from_nmea(const char *nmea_lat, const char *nmea_long, double alt);

    // Static method to convert NMEA format to decimal degrees
    static double nmea_to_dd(const char *coord);

    // Method to calculate distance between two GPS coordinates
    double distance(const GlobalPose &other) const;

    // Method to calculate the x distance between to GPS coordinates
    double x_distance(const GlobalPose &other) const;

    // Method to calculate the y distance between to GPS coordinates
    double y_distance(const GlobalPose &other) const;

    // Method to calculate vertical distance (altitude difference)
    double z_distance(const GlobalPose &other) const;

    // Method to calculate the bearing between two GPS coordinates
    double get_bearing(const GlobalPose &other) const;

    // Methods to offset the altered gps position
    void x_offset(double x);
    void y_offset(double y);

    // Setters for global pose
    void set_position(double lat, double lng, double alt);
    // Heading can be set specifically or calculated from a heading vector.
    void set_heading(const GlobalPose &heading_vector);
    void set_heading(double heading);
    
    // Getter for position in radians.
    std::vector<double> get_position_rad() const;
    std::vector<double> position_radians_;

    friend GlobalPose operator-(GlobalPose self, const GlobalPose &other);
    friend GlobalPose operator+(GlobalPose self, const GlobalPose &other);
};

#endif // GLOBAL_POSE_HPP