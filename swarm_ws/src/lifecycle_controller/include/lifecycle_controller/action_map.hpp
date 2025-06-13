#ifndef ACTION_MAP_HPP
#define ACTION_MAP_HPP

#include "lifecycle_controller/lifecycle_class.hpp"

class ActionMap
{
private:
    std::shared_ptr<Lifecycle> lifecycle_;
    void make_action_map();

    void read_states_from_config(const std::string &config_file);
    // enter exits
    void enter_launch();
    void exit_launch();
    void enter_rendezvous();
    void exit_rendezvous();
    void enter_pause();
    void exit_pause();
    void enter_configure();
    void exit_configure();
    void enter_start();
    void exit_start();
    void enter_takeoff();
    void exit_takeoff();
    void enter_move();
    void exit_move();
    void enter_px4_hold();
    void exit_px4_hold();
    void enter_return();
    void exit_return();
    void enter_land();
    void exit_land();
    void logEnter(const std::string &state);
    void logExit(const std::string &state);

    std::map<std::string, std::function<void()>> action_map;

public:
    ActionMap(std::shared_ptr<Lifecycle> lifecycle, const std::string &config_file);
    ~ActionMap();
    std::map<std::string, std::function<void()>> get_action_map();
};

#endif // ACTION_MAP_HPP