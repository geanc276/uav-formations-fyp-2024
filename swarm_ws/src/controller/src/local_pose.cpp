/*
 *   This is a derived class of the Pose class. It handles and contains all 
 *   of the pose information for a pose represented by Local (N, E, D) coordinates
 *   in a local context.
 *   
 *   File: local_pose.cpp
 *   Date: 6-8-2024
 *   Author: Benjamin Ireland
 */

#include "controller/local_pose.hpp"
#include <math.h>


//  CONSTRUCTORS  //


LocalPose::LocalPose() 
    : Pose()
{
}

LocalPose::LocalPose(std::string name) 
    : Pose(name)
{
    name_ = name;
}

LocalPose::LocalPose(double N, double E, double D) 
    : Pose(N, E, D)
{
}

LocalPose::LocalPose(double N, double E, double D, double heading) 
    : Pose(N, E, D, heading)
{
}

//  OPERATORS  //

LocalPose operator-(LocalPose self, const LocalPose &other)
{
    LocalPose result;
    result.position_ = {self.position_[0] - other.position_[0],
                        self.position_[1] - other.position_[1],
                        self.position_[2] - other.position_[2]};
    result.heading_ = self.heading_ - other.heading_;
    return result;
}

LocalPose operator+(LocalPose self, const LocalPose &other)
{
    LocalPose result;
    result.position_ = {self.position_[0] + other.position_[0],
                        self.position_[1] + other.position_[1],
                        self.position_[2] + other.position_[2]};
    result.heading_ = self.heading_ + other.heading_;
    return result;
}

LocalPose::~LocalPose() = default;