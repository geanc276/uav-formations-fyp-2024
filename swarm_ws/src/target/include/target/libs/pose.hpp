#ifndef POSE_HPP
#define POSE_HPP

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <iostream>
#include <iomanip>
#include <vector>

class Pose
{
public:
    Pose();
    Pose(double x, double y, double z);
    Pose(double x, double y, double z, double heading);
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
    double terrain_offset_;

    friend std::ostream &operator<<(std::ostream &os, const Pose &pos);
    friend Pose operator-(Pose self, const Pose &other);
    friend Pose operator+(Pose self, const Pose &other);
private:
};

#endif // POSE_HPP
