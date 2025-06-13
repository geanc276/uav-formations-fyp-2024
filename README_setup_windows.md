# Windows Setup Guide

This guide walks you through installing and configuring everything you need to run the UAV Formations project on Windows.

## Prerequisites

- **Git**  
- **Python 3.11**  
- **Windows Subsystem for Linux (WSL)**  
- **PowerShell** (to run setup scripts)

---

## 1. Install Git

Open PowerShell **as Administrator** and run:

```powershell
winget install --id Git.Git -e --source winget
```

---

## 2. Install Python 3.11

1. Open the **Microsoft Store** and search for **Python 3.11** (PREFERED as it handles venv and pip installs),

    or use winget:

   ```powershell
   winget install --id Python.Python.3.11
   ```

2. Ensure you select **Install for me only** if you do not have admin rights.

---

## 3. Set Up WSL

1. **Run** the WSL setup script **as Administrator**:

   ```powershell
   .\setup_windows.ps1
   ```

2. If prompted, **rerun** the script to finish installing WSL components.

3. Launch your newly installed WSL distribution, then in its shell **link your project directory**:

   ```bash
   cd \{your path\}/uav-formations-fyp-2024
   ./install_auto
   ```

4. **Line-ending fix**  
   If you see “unknown line ending” errors, install and use `dos2unix` in WSL:

   ```bash
   sudo apt-get update
   sudo apt-get install dos2unix
   dos2unix install_auto
   ```

---

## 4. Set Up the GUI

1. In PowerShell or WSL, **activate Python 3** and navigate to the GUI folder:

   ```bash
   cd drone_gui
   python3 start_gui.py
   ```

2. If you encounter pip/venv issues uninstall and make sure pip and venv are installed.



---


cd swarm_ws
source /opt/ros/humble/setup.bash
make input 
source 
ros2 run user_input user_input_node 1