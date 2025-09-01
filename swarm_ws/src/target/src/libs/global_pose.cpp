#include "target/libs/global_pose.hpp"
#include <cstring>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

GlobalPose::GlobalPose()
    : Pose()
{
    position_radians_ = {0, 0, 0};
    variances_ = {0, 0, 0};
}

GlobalPose::GlobalPose(double lat, double lng, double alt)
    : Pose(lat, lng, alt)
{
    position_radians_ = {DEG_TO_RAD(lat), DEG_TO_RAD(lng), alt};
    variances_ = {0, 0, 0};
}

GlobalPose::GlobalPose(double lat, double lng, double alt, int var[3])
    : Pose(lat, lng, alt)
{
    position_radians_ = {DEG_TO_RAD(lat), DEG_TO_RAD(lng), alt};
    variances_ = {var[0], var[1], var[2]};
}

std::ostream &operator<<(std::ostream &os, const GlobalPose &coord)
{
    os << "GlobalPose(Latitude: " << coord.position_[0]
       << ", Longitude: " << coord.position_[1]
       << ", Altitude: " << coord.position_radians_[2]
       << ", Variances: [" << coord.variances_[0] << ", "
       << coord.variances_[1] << ", " << coord.variances_[2] << "])";
    return os;
}

GlobalPose GlobalPose::from_nmea(const char *nmea_lat, const char *nmea_long, double alt)
{
    return GlobalPose(GlobalPose::nmea_to_dd(nmea_lat), GlobalPose::nmea_to_dd(nmea_long), alt);
}

// TODO investigate necessity of funct
double GlobalPose::nmea_to_dd(const char *coord)
{
    const char *point_ptr = strchr(coord, '.');
    if (!point_ptr)
        throw std::invalid_argument("Invalid NMEA coordinate format");

    int degrees_length = point_ptr - coord - 2;
    char degrees_str[10];
    strncpy(degrees_str, coord, degrees_length);
    degrees_str[degrees_length] = '\0';

    double degrees = std::atof(degrees_str);
    double minutes = std::atof(point_ptr - 2);
    double result = degrees + minutes / 60.0;

    char direction = coord[strlen(coord) - 1];
    if (direction == 'S' || direction == 'W')
        result = -result;

    return result;
}

/*
 * Calculates the distance in meters between two GPS
 * coordinates using the equirectangular approximation.
 */
double GlobalPose::distance(const GlobalPose &other) const
{
    double x = (position_radians_[1] - other.position_radians_[1]) * cos((position_radians_[0] + other.position_radians_[0]) / 2.0) * EARTHS_RADIUS;
    double y = (position_radians_[0] - other.position_radians_[0]) * EARTHS_RADIUS;
    return std::sqrt(x * x + y * y);
}

// TODO Check conversion method with Le.

/*
 * Calculates the x component of the distance in meters between two GPS
 * coordinates using the equirectangular approximation, x represents the
 * north-south axis and thus the latitude.
 */
double GlobalPose::x_distance(const GlobalPose &other) const
{
    double self_lat;
    double other_lat;
    double x;

    self_lat = DEG_TO_RAD(this->position_[0]);
    other_lat = DEG_TO_RAD(other.position_[0]);
    x = (other_lat - self_lat) * EARTHS_RADIUS;
    return x;
}

/*
 * Calculates the y component of the distance in meters between two GPS
 * coordinates using the equirectangular approximation y represents the east
 * axis and thus the longitude.
 */
double GlobalPose::y_distance(const GlobalPose &other) const
{
    double self_lat;
    double other_lat;
    double self_lon;
    double other_lon;
    double y;

    self_lat = DEG_TO_RAD(this->position_[0]);
    other_lat = DEG_TO_RAD(other.position_[0]);
    self_lon = DEG_TO_RAD(this->position_[1]);
    other_lon = DEG_TO_RAD(other.position_[1]);
    y = (other_lon - self_lon) * cos((self_lat + other_lat) / 2) * EARTHS_RADIUS;
    return y;
}

/*
 * Returns the change in altitude, z represents the down axis.
 */
double GlobalPose::z_distance(const GlobalPose &other) const
{
    return other.position_[2] - position_[2];
}

/*
 *  Shifts a GlobalPose by the given offset y in meters added to
 *  the longitude.
 */
void GlobalPose::y_offset(double y)
{
    double delta_long_rad;
    delta_long_rad = y / EARTHS_RADIUS / cos(DEG_TO_RAD(this->position_[0]));
    position_[1] = position_[1] + RAD_TO_DEG(delta_long_rad);
    position_radians_[1] = position_radians_[1] + delta_long_rad;
}

/*
 *  Shifts a GlobalPose by the given offset x in meters added to
 *  the latitude.
 */
void GlobalPose::x_offset(double x)
{
    double delta_lat_rad;
    delta_lat_rad = x / EARTHS_RADIUS;


    position_radians_[0] = get_position_rad()[0] + delta_lat_rad;
    position_[0] = get_position()[0] + RAD_TO_DEG(delta_lat_rad);


}

void GlobalPose::set_position(double lat, double lng, double alt)
{
    position_ = {lat, lng, alt};
    position_radians_ = {DEG_TO_RAD(lat), DEG_TO_RAD(lng), alt};
}


void GlobalPose::set_heading(double heading)
{
    heading_ = heading;
}

void GlobalPose::set_heading(const GlobalPose &heading_vector)
{
    heading_ = atan2(heading_vector.position_[1], heading_vector.position_[0]);
}

std::vector<double> GlobalPose::get_position_rad() const
{
    return position_radians_;
}

GlobalPose operator-(GlobalPose self, const GlobalPose &other)
{
    GlobalPose result;
    result.position_ = {self.position_[0] - other.position_[0],
                        self.position_[1] - other.position_[1],
                        self.position_[2] - other.position_[2]};
    result.position_radians_ = {self.position_radians_[0] - other.position_radians_[0],
                                self.position_radians_[1] - other.position_radians_[1],
                                self.position_radians_[2] - other.position_radians_[2]};
    result.heading_ = self.heading_ - other.heading_;
    return result;
}

GlobalPose operator+(GlobalPose self, const GlobalPose &other)
{
    GlobalPose result;
    result.position_ = {self.position_[0] + other.position_[0],
                        self.position_[1] + other.position_[1],
                        self.position_[2] + other.position_[2]};
    result.position_radians_ = {self.position_radians_[0] + other.position_radians_[0],
                                self.position_radians_[1] + other.position_radians_[1],
                                self.position_radians_[2] + other.position_radians_[2]};
    result.heading_ = self.heading_ + other.heading_;
    return result;
}