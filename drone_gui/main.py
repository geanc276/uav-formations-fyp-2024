# main.py

import threading
from backend import Backend
from frontend.app import App
from frontend.menu import PageButton
from utils.utils import simulate_drone_discovery

def main():
    # Define main menu items
    menu_items = [
        PageButton("Home", "Home", "blue"),
        PageButton("Settings", "Settings", "green"),
        PageButton("About", "About", "red")
    ]

    # Initialize backend and app
    backend = Backend(None)  # Initialize Backend with no app yet
    app = App(backend, menu_items)
    backend.app = app  # Assign the app to backend after initialization to avoid circular reference

    # Start drone discovery simulation in a separate thread
    discovery_thread = threading.Thread(target=simulate_drone_discovery, args=(backend,), daemon=True)
    discovery_thread.start()

    # Run the application
    app.mainloop()

if __name__ == "__main__":
    main()