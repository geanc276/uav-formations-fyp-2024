# Lifecycle Node User Input Command Callback Commands

## Overview

commadn structure:
{drone number or all}! --{type (command, state, coop)} {sub commands/ info}
This document provides a comprehensive list of commands that can be processed by the `user_input_command_cb` method in the `Lifecycle` class. Each command performs a specific action within the system.

## states

read states file that is passed

## coop

- launch
- formation
- return
  either 1(true) or 0(false)

## Commands

### Hello Command

- **Command**: `hello`
- **Action**: Logs a "Hello" message.

### Log Dropouts Control

- **Command**: `disable_log_dropouts`
  - **Action**: Disables logging of dropouts.
- **Command**: `enable_log_dropouts`
  - **Action**: Enables logging of dropouts.

### Log Coops Control

- **Command**: `enable_log_coops`
  - **Action**: Enables logging for launch, formation, and return coops.
- **Command**: `disable_log_coops`
  - **Action**: Disables logging for launch, formation, and return coops.

### Reset Coops

- **Command**: `reset_launch_coop`
  - **Action**: Resets the launch coop.
- **Command**: `reset_formation_coop`
  - **Action**: Resets the formation coop.
- **Command**: `reset_return_coop`
  - **Action**: Resets the return coop.
- **Command**: `reset_all_coops`
  - **Action**: Resets all coops (launch, formation, and return).

### Coop Input Only Control

- **Command**: `launch_coop_input_only_enable`
  - **Action**: Enables input-only mode for launch coop.
- **Command**: `launch_coop_input_only_disable`
  - **Action**: Disables input-only mode for launch coop.
- **Command**: `formation_coop_input_only_enable`
  - **Action**: Enables input-only mode for formation coop.
- **Command**: `formation_coop_input_only_disable`
  - **Action**: Disables input-only mode for formation coop.
- **Command**: `return_coop_input_only_enable`
  - **Action**: Enables input-only mode for return coop.
- **Command**: `return_coop_input_only_disable`
  - **Action**: Disables input-only mode for return coop.

### Error Reset Control

- **Command**: `reset_takeoff_error`
  - **Action**: Resets the takeoff error timer.
- **Command**: `reset_land_error`
  - **Action**: Resets the land error timer.
- **Command**: `reset_ang_error`
  - **Action**: Resets the angular error timer.
- **Command**: `reset_all_errors`
  - **Action**: Resets all error timers (takeoff, land, and angular).
