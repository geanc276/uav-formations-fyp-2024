// ALL DESCRIPTIONS ARE SUBJECT TO CHANGE AS UNDERSTANDING IS OBTAINED //
/*
 *
 *  Contains helper functions related to the drones lifecycle.
 * 
 *   File: state.cpp
 *   Date: 19-7-2024
 *   Author: Benjamin Ireland, Finlay Cross
 */

#include "target/libs/state.hpp"

std::string state_to_string(state_t state)
{
    std::string str_state;
    switch (state)
    {
        case CONFIG:
            str_state = "Config";
            break;
        case LANDED:
            str_state = "Landed";
            break;
        case TAKEOFF:
            str_state = "Takeoff";
            break;
        case RENDEZVOUS:
            str_state = "Rendezvous";
            break;
        case MOVE:
            str_state = "Move";
            break;
        case PAUSE:
            str_state = "Pause";
            break;
        case PX4_HOLD:
            str_state = "PX4 Hold";
            break;
        case RETURN:
            str_state = "Return";
            break;
        case LAND:
            str_state = "Land";
            break;
        default:
            str_state = "UNKNOWN";
    }
    return str_state;
}
