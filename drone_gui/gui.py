# main.py

import threading
import time
from backend.backend import Backend
from frontend.app import App
from frontend.menu import PageButton
from drone_package import DroneFinder
from frontend.pages import *
def main():
    # Define main menu items
    menu_items = [
        PageButton("Home", "Home","blue"),
        PageButton("Build", "Build","purple"),
        PageButton("DBuild", "DBuild_key","orange"),
        PageButton("Launch", "Launch","red"),
        PageButton("Settings", "Settings","green"),
        PageButton("User Input", "UserInputPage","blue"),
        PageButton("Currently Launched", "CurrentlyLaunched","purple"),
        PageButton("Draw", "Draw","orange")
    ]

    # Initialize backend and app
    backend = Backend(None)  # Initialize Backend with no app yet
    app = App(backend, menu_items)
    backend.app = app  # Assign the app to backend after initialization to avoid circular reference

    # Start drone discovery simulation in a separate thread
    # discovery_thread = threading.Thread(target=simulate_drone_discovery, args=(backend,), daemon=True)
    discovery_thread = threading.Thread(target=DroneFinder(backend.add_drone, backend.request_remove_drone, backend.request_update_of_drone).run, daemon=True)
    discovery_thread.start()

    # Run the application
    app.after(100, backend.check_updates)
    app.mainloop()

if __name__ == "__main__":
    main()