#include "target/libs/pose.hpp"
#include <rclcpp/rclcpp.hpp>


Pose::Pose()
{
    position_ = {0, 0, 0};
    heading_ = 0;
    terrain_offset_ = 0;
}

Pose::Pose(double x, double y, double z)
{
    position_ = {x, y, z};
    heading_ = 0;
    terrain_offset_ = 0;
}

Pose::Pose(double x, double y, double z, double heading)
{
    position_ = {x, y, z};
    heading_ = heading;
    terrain_offset_ = 0;
}

Pose::~Pose() = default;

void Pose::set_position(double x, double y, double z)
{
    position_ = {x, y, z};
}


std::vector<double> Pose::get_position() const
{
    return position_;
}

void Pose::set_heading(double heading)
{
    heading_ = heading;
}

void Pose::set_heading(Pose &heading_vector)
{
    heading_ = atan2(heading_vector.position_[1], heading_vector.position_[0]);
}

double Pose::get_heading() const
{
    return heading_;
}

void Pose::set_terrain_offset(double terrain_offset)
{
    terrain_offset_ = terrain_offset;
}

double Pose::get_terrain_offset() const
{
    return terrain_offset_;
}

std::ostream &operator<<(std::ostream &os, const Pose &pos)
{
    os << "X" << std::fixed << std::setprecision(3) << pos.position_[0]
       << "\tY" << pos.position_[1]
       << "\tZ" << pos.position_[2]
       << "\theading: " << pos.heading_;
    return os;
}

Pose operator-(Pose self, const Pose &other)
{
    Pose result;
    result.position_ = {self.position_[0] - other.position_[0],
                        self.position_[1] - other.position_[1],
                        self.position_[2] - other.position_[2]};
    result.heading_ = self.heading_ - other.heading_;
    return result;
}

Pose operator+(Pose self, const Pose &other)
{
    Pose result;
    result.position_ = {self.position_[0] + other.position_[0],
                        self.position_[1] + other.position_[1],
                        self.position_[2] + other.position_[2]};
    result.heading_ = self.heading_ + other.heading_;
    return result;
}