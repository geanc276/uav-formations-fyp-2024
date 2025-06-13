#include "lifecycle_controller/state.hpp"

#include <unordered_map>
#include <string>
#include <cstdint>


State::State(const std::string &name,
             const std::vector<std::string> &transitions,
             const std::vector<std::function<void()>> &on_enter_funcs,
             const std::vector<std::function<void()>> &on_exit_funcs,
             const std::function<void(const std::string &)> &logEnter,
             const std::function<void(const std::string &)> &logExit,
             bool off_board)
    : name_(name),
      off_board_(off_board),
      valid_transitions_(transitions),
      on_enter_(on_enter_funcs),
      on_exit_(on_exit_funcs),
      log_enter_(logEnter),
      log_exit_(logExit)
{
}

State::State()
    : off_board_(false) // Initialize other members as needed
{
}

State::~State() = default;

const std::string &State::get_name() const
{
    return name_;
}

bool State::is_off_board() const
{
    return off_board_;
}

bool State::is_valid_transition(const std::string &nextState) const
{
    return std::find(valid_transitions_.begin(), valid_transitions_.end(), nextState) != valid_transitions_.end();
}

std::vector<std::string> State::get_valid_transitions() const
{
    return valid_transitions_;
}

void State::enter()
{
    log_enter_(name_);
    if (on_enter_.empty())
        return;
    for (auto &func : on_enter_)
        if (func)
            func();
}

void State::exit()
{
    log_exit_(name_);
    if (on_exit_.empty())
        return;
    for (auto &func : on_exit_)
    {
        if (func)
            func();
    }
}


uint8_t convert_state_to_int(const std::string& state)
{
    static const std::unordered_map<std::string, uint8_t> state_map = {
        {"configure", 1},
        {"start", 2},
        {"takeoff", 3},
        {"rendezvous", 4},
        {"move", 5},
        {"pause", 6},
        {"px4_hold", 7},
        {"return", 8},
        {"land", 9}
    };

    auto it = state_map.find(state);
    if (it != state_map.end())
    {
        return it->second;
    }

    // Default value for unrecognized state
    return 0;
}
