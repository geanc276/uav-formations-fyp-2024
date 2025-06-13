/*
 *   This is a derived class of the Pose class. It handles and contains all 
 *   of the pose information for a pose represented by Local (N, E, D) coordinates
 *   in a local context.
 *   
 *   File: local_pose.hpp
 *   Date: 6-8-2024
 *   Author: Benjamin Ireland
 */

#ifndef LOCAL_POSITION_HPP
#define LOCAL_POSITION_HPP

#include "controller/pose.hpp"

class LocalPose : public Pose
{
private:
    std::string type = "local";
public:
    LocalPose();
    LocalPose(std::string name);
    LocalPose(double N, double E, double D);
    LocalPose(double N, double E, double D, std::string name);
    LocalPose(double N, double E, double D, double heading);
    LocalPose(double N, double E, double D, double heading, std::string name);
    ~LocalPose();

    friend LocalPose operator-(LocalPose self, const LocalPose &other);
    friend LocalPose operator+(LocalPose self, const LocalPose &other);
};

#endif // LOCAL_POSITION_HPP