# 🚁 **Lifecycle Controller Documentation**

---

## 📚 **Table of Contents**

- [Introduction](#introduction)
- [Technical Overview](#technical-overview)
  - [Main Program Flow](#main-program-flow)
  - [Configuration Reader (`config_reader`)](#configuration-reader-config_reader)
  - [Lifecycle Class (`lifecycle_class`)](#lifecycle-class-lifecycle_class)
  - [Action Map (`action_map`)](#action-map-action_map)
  - [State Management (`state`)](#state-management-state)
  - [User Input Handler (`input`)](#user-input-handler-input)
  - [Flight Control Unit Interface (`fcu`)](#flight-control-unit-interface-fcu)
  - [Error Monitoring (`error`)](#error-monitoring-error)
  - [Dropout Detection (`dropout`)](#dropout-detection-dropout)
- [Getting Started as a Developer](#getting-started-as-a-developer)
  - [Prerequisites](#prerequisites)
  - [Understanding the Codebase](#understanding-the-codebase)
  - [Extending the Lifecycle Controller](#extending-the-lifecycle-controller)

---

## 🔰 **Introduction**

Welcome to the **Lifecycle Controller Documentation**! 🎉

This comprehensive guide provides an in-depth overview of the Lifecycle Controller software for a drone system using **ROS 2** and **PX4**. The software is designed to manage the different states of a drone's lifecycle—such as configuration, takeoff, rendezvous, movement, and landing—while ensuring synchronization and error handling in a multi-drone setup.

---

## 🛠️ **Technical Overview**

### 🌀 **Main Program Flow**

**File:** `main.cpp`

The entry point of the software is the `main` function in `main.cpp`. It performs the following steps:

1. **Initialization**

   - Initializes the ROS 2 node.
   - Reads configuration parameters such as the number of drones and the configuration file path.
   - Initializes the list of drone names based on the number of drones.

2. **Configuration Reading**

   - Uses `ConfigReader` to parse the configuration file and obtain drone-specific parameters like the drone's name, ID, and takeoff height. _(Note: This does not read the controller swarm formation.)_

3. **Lifecycle Initialization**

   - Creates an instance of the `Lifecycle` class, passing in the configuration parameters.
   - Sets up the `UserInput` and `ActionMap` components for handling user interactions and state actions.

4. **Main Loop**

   - Runs the lifecycle's `run` method in a loop, which handles state transitions and actions.
   - Uses `rclcpp::spin_some` to process incoming messages and callbacks.

5. **Shutdown**

   - Properly shuts down ROS 2 before exiting.

---

### 📖 **Configuration Reader (`config_reader`)**

**Files:** `config_reader.cpp`, `config_reader.hpp`

The `ConfigReader` class is responsible for reading and parsing the drone's configuration from a JSON file. It extracts critical parameters like:

- **Drone ID**
- **Drone Name** (e.g., `"drone_1"`)
- **Takeoff Height**

**Key Functions:**

- `readConfig()`: Parses the JSON file and populates the class members.
- **Accessor Methods:**
  - `getName()`
  - `get_id()`
  - `get_takeoff_height()`

---

### 🔄 **Lifecycle Class (`lifecycle_class`)**

**Files:** `lifecycle_class.cpp`, `lifecycle_class.hpp`

The `Lifecycle` class manages the drone's lifecycle states and transitions. It encapsulates the state machine logic and integrates with other components like error checking, cooperative synchronization, and the flight control unit (FCU).

**Key Components:**

- **State Management**

  - Maintains the current state and valid transitions.
  - Uses the `State` class to represent each state.

- **Cooperative Components**

  - **Cooperative Launch (`launch_coop_`):** Synchronizes the takeoff among multiple drones.
  - **Cooperative Formation (`formation_coop_`):** Synchronizes movement into formation.
  - **Cooperative Return (`return_coop_`):** Synchronizes the return to launch.

- **Error Handling** _(Currently unused; error is handled on the controller side now.)_

  - **Takeoff Error (`takeoff_error_`):** Monitors takeoff position errors.
  - **Landing Error (`land_error_`):** Monitors landing position errors.
  - **Angular Error (`ang_error_`):** Monitors orientation position errors.

- **Dropout Detection**

  - **Dropout Checker (`dropout_checker_`):** Detects communication dropouts with other drones.

- **Flight Control**

  - **FCU Interface (`fcu_`):** Communicates with the PX4 flight controller for commands like arming, takeoff, and landing. _(In the future, it will also send controller info passed from the controller.)_

**Key Functions:**

- `init()`: Initializes subscriptions, publishers, and components.
- `run()`: Main loop that checks for state transitions and runs state-specific logic.
- `transition(nextState, force)`: Handles state transitions with an optional force override.
- `get_current_state()`: Returns the name of the current state.
- `is_any_data_stale()`: Checks if any critical data is stale (e.g., due to dropout). _(This is a legacy function that is no longer in use.)_

---

### 🗺️ **Action Map (`action_map`)**

**Files:** `action_map.cpp`, `action_map.hpp`

The `ActionMap` class maps state transitions to specific actions. It reads the state configurations from a JSON file and binds actions to state entries and exits.

**Key Functions:**

- `read_states_from_config(config_file)`: Parses the JSON configuration to set up states and transitions.
- `make_action_map()`: Creates a map of action names to function pointers.

**Action Functions:**

- `enter_launch()`, `exit_launch()`
- `enter_takeoff()`, `exit_takeoff()`
- _(Additional state actions)_

---

### 📊 **State Management (`state`)**

**Files:** `state.cpp`, `state.hpp`

The `State` class represents a state in the drone's lifecycle. It includes information about:

- **State Name**
- **Valid Transitions** to other states
- **Actions** to perform upon entering and exiting the state
- Whether the state requires **Offboard Control Mode**

**Key Functions:**

- `enter()`: Executes all functions associated with entering the state.
- `exit()`: Executes all functions associated with exiting the state.
- `is_valid_transition(nextState)`: Checks if transitioning to `nextState` is valid.

---

### 🎮 **User Input Handler (`input`)**

**Files:** `input.cpp`, `input.hpp`

The `UserInput` class handles external inputs and is the main communication point during drone runtime.

**Key Subscriptions:**

- `/user_input/state`: Receives state transition commands.
- `/user_input/PID`: Receives PID parameter updates.
- Specific drone commands like `/drone_1/user_input/command`.

**Key Functions:**

- `state_cb(msg)`: Callback for handling state transition commands.
- `command_cb(msg)`: Callback for handling custom commands.
- `PID_cb(msg)`: Callback for handling PID parameter updates.

---

### ✈️ **Flight Control Unit Interface (`fcu`)**

**Files:** `fcu.cpp`, `fcu.hpp`

The `FCU` class interfaces with the PX4 flight controller to send commands and receive status updates.

**Key Functions:**

- `publish_vehicle_command(command, params...)`: Sends a vehicle command to PX4.
- **Commands:**
  - `publish_arm()`
  - `publish_disarm()`
  - `publish_takeoff()`
  - `publish_land()`
  - `publish_return()`
- `publish_off_board()`: Enables offboard control mode.
- `publish_off_board_control_mode()`: Sends periodic messages to maintain offboard control mode.

**Key Callbacks:**

- `state_cb(msg)`: Receives vehicle status updates.
- `vehicle_command_ack_cb(msg)`: Receives acknowledgments for commands sent to PX4.

---

### ⚠️ **Error Monitoring (`error`)** _(Deprecated)_

**Files:** `error.cpp`, `error.hpp`

The `Error` class monitors specific errors related to the drone's operation, such as distance, altitude, and angular errors.

**Key Functions:**

- `error_cb(topic, msg)`: Callback for error messages from various topics.
- `convergence_cb()`: Checks if errors have converged to acceptable levels.
- `converged()`: Returns whether all monitored errors have converged.

---

### 📉 **Dropout Detection (`dropout`)**

**Files:** `dropout.cpp`, `dropout.hpp`

The `Dropout` class detects communication dropouts with other drones. It uses ROS services to check the liveliness of other drones at regular intervals. _(This currently only prints dropouts; in the future, it should land the drone.)_

**Key Functions:**

- `checkForDropouts()`: Periodically checks if other drones are responsive.
- `handleDropout(drone)`: Handles the case where a drone is unresponsive.
- `getDropoutFlag()`: Returns whether any dropout has been detected.

---

## 🎓 **Getting Started as a Developer**

### 📝 **Prerequisites**

- **Knowledge of C++** and object-oriented programming.
- Familiarity with **ROS 2** (Robot Operating System 2) and its concepts like nodes, topics, services, and messages.
- Understanding of **PX4 Autopilot**.

---

### 🧐 **Understanding the Codebase**

1. **Start with `main.cpp`**

   - This is the entry point of the application. It initializes the ROS 2 node and sets up the lifecycle controller.

2. **Configuration**

   - Look into `config_reader.cpp` to understand how configurations are loaded and parsed.

3. **Lifecycle Management**

   - The `Lifecycle` class in `lifecycle_class.cpp` is the core of the application. It manages states and transitions.

4. **State Actions**

   - `ActionMap` in `action_map.cpp` links states to their corresponding actions. This is where you can see what happens when the drone enters or exits a state.

5. **State Definitions**

   - The `State` class in `state.cpp` defines what a state is, including its valid transitions and associated actions.

6. **User Input**

   - `UserInput` in `input.cpp` handles external commands and can be extended to support more user interactions.

7. **Flight Control Interface**

   - `FCU` in `fcu.cpp` is responsible for sending commands to the PX4 flight controller.

8. **Error Handling and Dropout Detection**

   - `Dropout` detects if any drones in the swarm are no longer "alive."

---

### 🚀 **Extending the Lifecycle Controller**

Here are some recommendations for extending the **Lifecycle Controller**:

1. **Adding New States**

   - Define a new state in the JSON configuration file with its valid transitions.
   - Implement any enter and exit actions in `ActionMap`.

2. **Implementing Custom Actions**

   - Add new action functions in `action_map.cpp` and map them in `make_action_map()`.

3. **Handling New Inputs**

   - Extend `UserInput` to subscribe to new topics or handle additional commands.

4. **FCU Update**

   - Move trajectory setpoints from the controller to this class. You'll need a forwarding-style message callback that listens to the controller or have the nodes use shared memory—whichever is fastest for the loop.

5. **Improving Dropout Detection**

   - Enhance the `Dropout` class to include more sophisticated checks or recovery mechanisms. _(This will likely need to wait until the new network protocol is implemented.)_

---
