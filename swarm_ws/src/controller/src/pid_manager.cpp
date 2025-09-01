/*
 *  PID manager utilizes the ros2_control library to run three PID controllers
 *  that maintain the drones configured position, altitude, and heading
 *  w.r.t the reference point.
 *
 *   File: pid_manager.cpp
 *   Date: 10-7-2024
 *   Author: Benjamin Ireland, Finlay Cross
 */

#include "controller/pid_manager.hpp"

#include <cmath>

#define ACCEL_FACTOR 0.1


// CLASS METHODS //


// TODO Make sure previous time is properly init
PidManager::PidManager(const Constraints &constraints)
    : logger_(rclcpp::get_logger("pid_manager_" + std::to_string(constraints.get_id() + 1))),
      constraints_(constraints),

      position_pid_(constraints_.get_pos_gains().p_gain_, constraints_.get_pos_gains().i_gain_,
                    constraints_.get_pos_gains().d_gain_, INTEGRAL_MAX, INTEGRAL_MIN, ANTI_WINDUP),
      altitude_pid_(constraints_.get_alt_gains().p_gain_, constraints_.get_alt_gains().i_gain_,
                    constraints_.get_alt_gains().d_gain_, INTEGRAL_MAX, INTEGRAL_MIN, ANTI_WINDUP),
      heading_pid_(constraints_.get_ang_gains().p_gain_, constraints_.get_ang_gains().i_gain_,
                   constraints_.get_ang_gains().d_gain_, INTEGRAL_MAX, INTEGRAL_MIN, ANTI_WINDUP)

{
}





PidManager::PidManager()
: logger_(rclcpp::get_logger("pid_manager")),
  constraints_(Constraints()),
  position_pid_(0, 0, 0, INTEGRAL_MAX, INTEGRAL_MIN, ANTI_WINDUP),
  altitude_pid_(0, 0, 0, INTEGRAL_MAX, INTEGRAL_MIN, ANTI_WINDUP),
  heading_pid_(0, 0, 0, INTEGRAL_MAX, INTEGRAL_MIN, ANTI_WINDUP)
{
}


/**
 *  @brief  Logs the computed error of a given PID controller.
 *  
 *  @param  error       Computed error.
 *  @param  pid_name    String representing the pid_type.
 */
void PidManager::log_error(double error, std::string pid_name)
{
}


// GETTERS/SETTERS //


void PidManager::set_gains(struct control_toolbox::Pid::Gains gains, pid_type_t type)
{
    switch (type)
    {
    case ALTITUDE:
        altitude_pid_.setGains(gains);
        break;
    case POSITION:
        position_pid_.setGains(gains);
        break;
    case HEADING:
        heading_pid_.setGains(gains);
        break;
    default:
        break;
    }
}

control_toolbox::Pid::Gains PidManager::get_gains(pid_type_t type)
{
    switch (type)
    {
    case ALTITUDE:
        return altitude_pid_.getGains();
    case POSITION:
        return position_pid_.getGains();
    case HEADING:
        return heading_pid_.getGains();
    default:
 
        return {};
    }
}

std::array<double, 7> PidManager::get_corrections(void)
{
    return corrections_;
}


// ERROR GETTERS //


/**
 *  @brief  Uses defined drone configuration to compute the drones goal position w.r.t
 *          the reference point.
 * 
 *  @param  goal_pose               The goal pose.
 *  @param  ref_point_local_pose    Local pose of the reference point.
 * 
 */
void PidManager::find_goal_position(LocalPose &goal_pose, LocalPose &ref_point_local_pose)
{
    altitude_config_t alt_constraint = constraints_.get_alt_cnstr();
    position_config_t pos_constraint = constraints_.get_pos_cnstr();
    double heading_constraint = constraints_.get_heading_cnstr();
    double rp_altitude;
    double x, y, z, heading;

    // Invert to get positive altitude.
    rp_altitude = -(ref_point_local_pose.get_position()[2]);

    x = ref_point_local_pose.get_position()[0] + pos_constraint.dist_x;
    y = ref_point_local_pose.get_position()[1] + pos_constraint.dist_y;
    heading = ref_point_local_pose.get_heading() + DEG_TO_RAD(heading_constraint);


    if (rp_altitude <= alt_constraint.min_alt)
    {
        z = -(alt_constraint.min_alt + alt_constraint.height_displacement);
    }
    else
    {
        z  = ref_point_local_pose.get_position()[2] + (-alt_constraint.height_displacement);
    }
    goal_pose.set_position(x, y, z);
 
    goal_pose.set_heading(heading);
}

/**
 * @brief   Computes the error vector for the drone.
 * 
 * @param   drone_pose  Current Local pose of the drone.
 * @param   goal_pose   Desired Goal pose.
 * @return  LocalPose   The error pose for position and heading.
 */
LocalPose PidManager::get_error(LocalPose &drone_pose, LocalPose &goal_pose)
{
    LocalPose error_vect;
    error_vect = goal_pose - drone_pose;
 
    if (error_vect.get_heading() < -M_PI)
    {
        error_vect.set_heading(error_vect.get_heading() + 2 * M_PI);
    }
    else if (error_vect.get_heading() >= M_PI)
    {
    	error_vect.set_heading(error_vect.get_heading() - 2 * M_PI);
    }
    return error_vect;
}

/**
 * @brief   Computes the correction to the drones altitude.
 * 
 * @param   error_vect  The error pose.
 * @param   delta_t     Change in time for the PID controllers.
 */
void PidManager::get_altitude_correction(LocalPose error_vect, long delta_t)
{
    double alt_correction;

    alt_correction = altitude_pid_.computeCommand(error_vect.get_position()[2], delta_t);

    // Invert all z-axis corrections to match the NED coordinate system PX4 uses.
    corrections_[2] = alt_correction;
    corrections_[5] = alt_correction * ACCEL_FACTOR;
}

/**
 * @brief   Computes the correction to the drones position w.r.t. the RP.
 * 
 * @param   error_vect  The error pose.
 * @param   delta_t     Change in time for the PID controllers.
 */
void PidManager::get_position_correction(LocalPose error_vect, long delta_t)
{
    double pos_correction;
    double error_magnitude;
    std::array<double, 2> direction_vect;

    error_magnitude = sqrt(error_vect.get_position()[0] * error_vect.get_position()[0]  + 
                            error_vect.get_position()[1] * error_vect.get_position()[1]);
    direction_vect = {error_vect.get_position()[0] / error_magnitude,
                    error_vect.get_position()[1] / error_magnitude};

    pos_correction = position_pid_.computeCommand(error_magnitude, delta_t);

    // Velocity Corrections
    corrections_[0] = pos_correction * direction_vect[0];
    corrections_[1] = pos_correction * direction_vect[1];

    // Acceleration Corrections
    corrections_[3] = (pos_correction * ACCEL_FACTOR) * direction_vect[0];
    corrections_[4] = (pos_correction * ACCEL_FACTOR) * direction_vect[1];
}

/**
 * @brief   Computes the correction to the drones heading w.r.t. the RP.
 * 
 * @param   error_vect  The error pose.
 * @param   delta_t     Change in time for the PID controllers.
 */
void PidManager::get_heading_correction(LocalPose error_vect, long delta_t)
{
    double heading_correction;

    heading_correction = heading_pid_.computeCommand(error_vect.get_heading(), delta_t);

    corrections_[6] = heading_correction;
}

/**
 * @brief   Updates the three PID controllers, returning the overall velocity corrections.
 * 
 * @param   drone       Local drone Pose.
 * @param   goal_pos    Goal pose.
 */
void PidManager::update_pids(LocalPose &drone, LocalPose &goal_pos)
{
    rclcpp::Time time; 
    double delta_t;
    LocalPose error_vect;

    time = clock_.now();
    delta_t = (time - prev_time_).nanoseconds();
    error_vect = get_error(drone, goal_pos);

    this->get_position_correction(error_vect, delta_t);
    this->get_altitude_correction(error_vect, delta_t);
    this->get_heading_correction(error_vect, delta_t);

    prev_time_ = clock_.now();
}


PidManager::~PidManager() = default;
