

# Drone Management GUI

This project provides a desktop graphical user interface to discover, monitor, and control a swarm of drones. It leverages CustomTkinter for the frontend, Zeroconf for network-based drone discovery, and ZeroMQ for real-time status and mission updates.

## Prerequisites

- **Python 3.11** (or compatible)  
- Python packages (install via `pip install -r requirements.txt`):
  - customtkinter
  - pillow
  - matplotlib
  - zeroconf
  - pyzmq
  - tkinter (usually bundled with Python)
- A network environment where drones advertise themselves over `_http._tcp.local.`

## Installation

1. **Clone the repository**  
   ```bash
   git clone <repo-url>
   cd uav-formations-fyp-2024/drone_gui
   ```
2. **Set up a virtual environment**  
   ```bash
   python3.11 -m venv gui_venv
   source gui_venv/bin/activate
   ```
3. **Install dependencies**  
   ```bash
   pip install -r requirements.txt
   ```

## Running the Application

From the `drone_gui` directory, start the GUI with to auto set up the Venv:
```bash
python3 start_gui.py
```
This will open the main window, auto-discover drones on your LAN, and let you interact with them.

---

## Project Layout

```
drone_gui/
├── frontend/
│   ├── app.py
│   ├── menu.py
│   └── pages.py
├── backend/
│   └── backend.py
├── drone_package/
│   ├── drone_finder.py
│   ├── drone_status_listener.py
│   └── swarm.py
├── utils/
│   └── helpers.py
└── _docs/
    └── README.md      ← you are here
```

### 1. Frontend

- **app.py**  
  - Entry point: constructs the main window, header bar, side menu, and page container.  
  - Manages navigation history and shows/hides pages.  
  - Wires the `Backend` into the GUI layers.

- **menu.py**  
  - Defines `Menu` (side-bar) and `PageButton` items.  
  - Generates clickable buttons for each major page, with color theming.

- **pages.py**  
  - Base `Page` class encapsulates common layout, loading, and update behaviors.  
  - Subclasses for each major GUI page (see details below).

#### HomePage
- **Purpose**: Main dashboard listing all discovered drones.
- **Key widgets/components**:
  - Header ("Home Page").
  - A `Scrollbox` containing a grid of drone buttons, each showing info via a `DroneButtonInfoBox`.
- **Special behaviors**:
  - Dynamically populates drone buttons based on backend's current drone list.
  - Supports adding/removing drone buttons in real time.
  - Reloads drone list when updated.

#### SettingsPage
- **Purpose**: General settings and preferences for the application.
- **Key widgets/components**:
  - Header ("Settings").
  - Toggle switch for enabling/disabling a feature.
  - Slider for adjusting a parameter.
  - Button to navigate to the `SettingsChildPage`.
- **Special behaviors**:
  - Logs changes to toggles and sliders.
  - Demonstrates navigation to child settings pages.

#### SettingsChildPage
- **Purpose**: Example child settings page for nested configuration.
- **Key widgets/components**:
  - Header ("Settings - Child Page").
  - Entry box for user input.
  - Save button to persist settings.
- **Special behaviors**:
  - Displays confirmation dialog on save.
  - Logs saved settings.

#### AboutPage
- **Purpose**: Provides information about the application.
- **Key widgets/components**:
  - Header ("About").
  - Informational label.
  - "More Info" button that triggers a dialog.
- **Special behaviors**:
  - Shows a message box with additional information.

#### DronePage
- **Purpose**: Shows detailed view and controls for a single drone.
- **Key widgets/components**:
  - Info section and command section, built by the drone's `page` object.
  - SSH terminal launch capability.
- **Special behaviors**:
  - Dynamically reloads when switching drones.
  - Handles opening an SSH terminal appropriate to the OS.

#### BuildPage
- **Purpose**: Allows the user to build and zip the drone software workspace.
- **Key widgets/components**:
  - Header ("Build and Zip").
  - Progress bar for build status.
  - Status label.
  - Buttons to trigger `colcon build` and zip script.
- **Special behaviors**:
  - Runs build and zip commands in background threads.
  - Updates progress and status in real time.
  - Prompts for ARM install option for the zip script.

#### DroneBuildPage
- **Purpose**: Build and deploy applications to individual drones.
- **Key widgets/components**:
  - Header ("Drone Build").
  - Instructions label.
  - Scrollbox listing all supported drones, each with:
    - Build status, progress bar, and start build button.
- **Special behaviors**:
  - Each drone's build runs in its own thread.
  - Per-drone progress and status are tracked and updated.

#### LaunchPage
- **Purpose**: Select and launch multiple drones.
- **Key widgets/components**:
  - Header ("Launch Page").
  - `MultiSelectScrollbox` with selectable drone buttons.
  - "Select All", "Deselect All", and "Launch Selected" buttons.
- **Special behaviors**:
  - Launches selected drones or navigates to configuration for further setup.
  - Supports batch selection/deselection.

#### LaunchConfigPage
- **Purpose**: Configure launch parameters and assign configuration files to selected drones.
- **Key widgets/components**:
  - Header ("Launch Configuration").
  - JSON directory selection and display.
  - Auto-assign configuration button.
  - Info widgets for each selected drone showing config status.
  - "Launch" and "Stop" control buttons.
- **Special behaviors**:
  - Allows browsing for configuration files and auto-assigning them to drones.
  - Updates drone config widgets in real time.
  - Handles launching and stopping drones as a group.

#### UserInputPage
- **Purpose**: Provides user controls for sending commands or user input to drones.
- **Key widgets/components**:
  - Header ("User Input Page").
  - Buttons for "Start User Input", "Start", "Land", "Pause", and "Resume".
- **Special behaviors**:
  - Triggers backend user input windows and sends commands to drones.

#### CurrentlyLaunchedPage
- **Purpose**: Displays all currently launched drones with live control.
- **Key widgets/components**:
  - Header ("Currently Launched Drones").
  - Scrollbox listing drones, each with Stop and SSH buttons (`LaunchedDroneWidget`).
  - "Stop All" button to halt all drones.
- **Special behaviors**:
  - Refreshes list in real time.
  - Supports per-drone and batch stopping.
  - Provides SSH access to each drone.

#### ConfigPage
- **Purpose**: Advanced configuration editor for drone formations and constraints.
- **Key widgets/components**:
  - Header ("Configuration").
  - Table for editing node (drone) positions (ID, X, Y, Z).
  - Buttons to add/remove nodes and save config.
  - 3D visualization of node positions (Matplotlib).
  - Sliders for adjusting X, Y, Z of selected node.
- **Special behaviors**:
  - Updates visualization and config in real time.
  - Allows interactive editing and saving of per-drone configuration JSON files.


### 2. Backend

- **backend.py**  
  - Acts as a bridge between the GUI and the drone swarm logic.  
  - Maintains a `Swarm` instance and handles add/remove/update requests for each added drone.  
  - Provides high-level actions: launch/stop drones, open SSH terminals, open user-input windows.

### 3. Drone Discovery & Status

- **drone_finder.py**  
  - Implements `DroneFinder` using Zeroconf.  
  - Listens for `_http._tcp.local.` services, parses metadata (name, IP, ports, custom properties), and notifies the backend of new, updated, or removed drones.
  - Debounces transient removals to avoid flicker.

- **drone_status_listener.py**  
  - Defines a generic ZeroMQ `Subscriber` class and two concrete listeners:
    - **DroneStatusListener**: subscribes to `<hostname>/status` messages.
    - **DroneMissionListener**: subscribes to `<hostname>/mission/status` messages.
  - Parses JSON payloads into `DroneStatus` and `MissionStatus` models, triggering backend callbacks on changes.

### 4. Drone Domain Model

- **swarm.py**  
  - Defines the `Drone` and `DroneConnection` classes plus `DronePageInfo` for UI integration.
  - **Drone**  
    - Holds connection info, ZMQ listeners, and command objects (launch, stop, state, connection).
    - Detects status/mission changes and pushes updates through the `Backend`.
  - **DroneConnection**  
    - Encapsulates network parameters (hostname, IP, ports) and supported commands.  
    - Parses a comma-separated list of protocols into executable command instances.
  - **DronePageInfo**  
    - Builds the per-drone detail view inside the GUI (labels, grids, SSH button).

### 5. Utilities

- **helpers.py** (in `utils/`)  
  - Common helper functions for:
    - Running builds (`run_colcon_build`)
    - Creating zip archives (`run_zip_script`)
    - Parsing command-support strings
    - Directory-selection and text-input dialogs

---

## How It All Fits Together

1. **Launch** `start_gui.py` → Main window appears.
2. **Discovery** by `DroneFinder` → backend receives `add_drone` calls.
3. **HomePage** populates drone list from `Backend.get_all_drones()`.
4. **Select a drone** → navigates to its detail page built by `DronePageInfo`.
5. **Realtime updates**  
   - StatusListener and MissionListener push changes via ZeroMQ → `Drone.set_drone_status` / `set_drone_mission_status` → `Backend` → GUI refresh.
6. **Interaction buttons** allow launching, stopping, or SSH’ing into drones.

---

