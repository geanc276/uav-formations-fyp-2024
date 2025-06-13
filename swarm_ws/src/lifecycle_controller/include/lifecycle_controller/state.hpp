#ifndef STATE_HPP
#define STATE_HPP

#include <iostream>
#include <string>
#include <vector>
#include <functional>

class State
{
private:
    std::string name_;
    bool off_board_;
    std::vector<std::string> valid_transitions_;
    std::vector<std::function<void()>> on_enter_;
    std::vector<std::function<void()>> on_exit_;
    std::function<void(const std::string &)> log_enter_;
    std::function<void(const std::string &)> log_exit_;

public:
    State(const std::string &name,
          const std::vector<std::string> &transitions,
          const std::vector<std::function<void()>> &on_enter_funcs,
          const std::vector<std::function<void()>> &on_exit_funcs,
          const std::function<void(const std::string &)> &logEnter,
          const std::function<void(const std::string &)> &logExit,
          bool off_board);

    State();
    ~State();
    const std::string &get_name() const;
    bool is_off_board() const;
    bool is_valid_transition(const std::string &next_state) const;
    std::vector<std::string> get_valid_transitions() const;
    void enter();
    void exit();
};

uint8_t convert_state_to_int(const std::string& state);

#endif // STATE_HPP
