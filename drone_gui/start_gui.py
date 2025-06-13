#!/usr/bin/env python3
"""
Cross-platform launcher for the drone GUI.
Creates (if needed) a virtual environment, installs dependencies,
and launches gui.py in a new terminal window.
"""
import os
import sys
import subprocess
import platform
import shutil
import argparse

def run(cmd, **kwargs):
    subprocess.check_call(cmd, **kwargs, shell=isinstance(cmd, str))

def setup_venv(venv_dir):
    if not os.path.isdir(venv_dir):
        print(f"Creating virtual environment at {venv_dir}")
        run([sys.executable, "-m", "venv", venv_dir])

    # Default paths for python executable
    possible_paths = [
        os.path.join(venv_dir, "Scripts", "python.exe"),  # Windows
        os.path.join(venv_dir, "bin", "python"),          # Unix
        os.path.join(venv_dir, "python.exe"),             # winget fallback
        os.path.join(venv_dir, "python")                  # other fallback
    ]

    for path in possible_paths:
        if os.path.exists(path):
            return path

    raise FileNotFoundError("Python executable not found in virtual environment.")

def install_requirements(python_exe, requirements):
    if os.path.isfile(requirements):
        print("Installing requirements...")
        run([python_exe, "-m", "pip", "install", "--upgrade", "pip"])
        run([python_exe, "-m", "pip", "install", "-r", requirements])
    else:
        print("No requirements.txt found; skipping.")

def launch_in_terminal(cmd, cwd):
    system = platform.system()
    if system == "Windows":
        subprocess.Popen(f'start cmd /k "{cmd}"', cwd=cwd, shell=True)
    elif system == "Darwin":
        # macOS: run in the user's login shell instead of AppleScript
        subprocess.Popen(
            ["bash", "-lc", f'cd "{cwd}" && {cmd}'],
            cwd=cwd
        )
    else:
        term = (
            shutil.which("gnome-terminal")
            or shutil.which("x-terminal-emulator")
            or shutil.which("xterm")
        )
        if term:
            if "gnome-terminal" in term:
                subprocess.Popen([term, "--", "bash", "-lc", f'cd "{cwd}" && {cmd}'])
            else:
                subprocess.Popen([
                    term,
                    "-hold",
                    "-e",
                    f'bash -lc "cd \\"{cwd}\\" && {cmd}"'
                ])
        else:
            print("No terminal emulator found; running inline.")
            run(cmd, cwd=cwd, executable="/bin/bash")

def main():
    parser = argparse.ArgumentParser(description="Launch the drone GUI with virtualenv.")
    parser.add_argument(
        "--no-download", action="store_true",
        help="Skip virtualenv setup and package installation."
    )
    args = parser.parse_args()

    base = os.path.dirname(os.path.abspath(__file__))
    venv_dir = os.path.join(base, "gui_venv")

    if not args.no_download:
        py = setup_venv(venv_dir)
        install_requirements(py, os.path.join(base, "requirements.txt"))

    launcher_py = os.path.join(
        venv_dir,
        "Scripts" if platform.system() == "Windows" else "bin",
        "python"
    )
    script = os.path.join(base, "gui.py")
    launch_cmd = f'\"{launcher_py}\" \"{script}\"'
    print(f"Launching GUI: {launch_cmd}")
    launch_in_terminal(launch_cmd, base)

if __name__ == "__main__":
    main()
