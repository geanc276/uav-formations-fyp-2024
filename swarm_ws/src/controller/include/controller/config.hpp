/*
 *   Handles all config related information i.e.
 *   drone constraints w.r.t the reference point for each drone.
 *   reference point configuration.   
 *   
 *   File: config.hpp
 *   Date: 3-6-2024
 *   Author: Benjamin Ireland, Finlay Cross
 */

#ifndef CONFIG_HPP
#define CONFIG_HPP

#include "control_toolbox/pid.hpp"

// MACROS //

#define INTEGRAL_MAX 0.1
#define INTEGRAL_MIN 0
#define ANTI_WINDUP 0

// STRUCTS //
typedef struct
{
    double dist;
    double pos_angle;
    double dist_x;
    double dist_y;
    double dist_z;
} position_config_t;

typedef struct
{
    double min_alt;
    double height_displacement;
} altitude_config_t;

typedef enum
{
    ALTITUDE,
    POSITION,
    HEADING,
    ANGULAR
} pid_type_t;

/*
 *  Constraints class. Holds individual drone position, alt, and 
 *  heading constraint information.
 */
class Constraints
{
public:
    Constraints(const std::string &file_path);
    Constraints();
    ~Constraints();

    int get_id(void) const;
    altitude_config_t get_alt_cnstr(void) const;
    position_config_t get_pos_cnstr(void) const;
    double get_heading_cnstr(void) const;
    control_toolbox::Pid::Gains get_gains(pid_type_t type) const;

    control_toolbox::Pid::Gains get_alt_gains(void) const;
    control_toolbox::Pid::Gains get_pos_gains(void) const;
    control_toolbox::Pid::Gains get_ang_gains(void) const;

private:
    int id_;
    std::string name_;
    altitude_config_t alt_;
    position_config_t position_;
    double heading_;
    control_toolbox::Pid::Gains alt_gains_;
    control_toolbox::Pid::Gains pos_gains_;
    control_toolbox::Pid::Gains ang_gains_;
};

/*
 *  RPConfig class. Used by the reference point, holds all drone constraints 
 *  for reference point set up.
 */
class RPConfig
{
public:
    RPConfig(const std::string &folder_path);
    RPConfig();
    ~RPConfig();


    // Returns the parsed constraints of a given drone.
    Constraints get_drone_constraint(void) const;

private:
    uint32_t num_of_drones_;
    Constraints drone_constraints_;
};

#endif // CONFIG_HPP