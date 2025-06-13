// ALL DESCRIPTIONS ARE SUBJECT TO CHANGE AS UNDERSTANDING IS OBTAINED //
/*
 *  Holds all controller state information.
 * 
 *   File: state.hpp
 *   Date: 7-7-2024
 *   Author: Benjamin Ireland
 */

#ifndef STATE_HPP
#define STATE_HPP

#include <string>

typedef enum 
{
    CONFIG = 1,
    LANDED,
    TAKEOFF,
    RENDEZVOUS,
    MOVE,
    PAUSE,
    PX4_HOLD,
    RETURN,
    LAND
} state_t;

std::string state_to_string(state_t state);

#endif /* STATE_HPP */