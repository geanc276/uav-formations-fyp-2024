import time
import random
import logging
from tkinter import Toplevel, filedialog
from tkinter import StringVar  # Still using StringVar from tkinter
import customtkinter as ctk

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)


def simulate_drone_discovery(backend):
    """
    Simulates the discovery of drones by randomly adding, removing, or updating
    drones on the given backend.

    This function runs indefinitely, sleeping for a random interval between 2
    and 5 seconds, and then randomly performing one of:
      - 'add': Creates a new drone with incremented numbering.
      - 'remove': Randomly removes one of the discovered drones (if any).
      - 'update': Randomly updates a drone's battery and status.
    """
    drone_count = 0
    while True:
        sleep_time = random.randint(2, 5)
        logger.debug("Sleeping for %d seconds", sleep_time)
        time.sleep(sleep_time)

        action = random.choice(['add', 'remove', 'update'])
        logger.debug("Simulated action: %s", action)
        if action == 'add':
            drone_count += 1
            drone_data = {
                'name': f'Drone{drone_count}',
                'ip': f'192.168.1.{drone_count}',
                'port': 8000 + drone_count,
                'properties': {
                    'hostname': f'drone{drone_count}.local',
                    'status': random.choice(['Active', 'Idle', 'Charging']),
                    'battery': f'{random.randint(20, 100)}%'
                },
                'commands_supported': (
                    'upload_new_json:json,upload_new_executable:obj,'
                    'start_executable:command,request_log:request,invalid_command:invalid'
                )
            }
            logger.info("Adding drone: %s", drone_data['name'])
            backend.add_drone(drone_data)

        elif action == 'remove' and backend.swarm.discovered_drones:
            drone = random.choice(backend.swarm.discovered_drones)
            logger.info("Removing drone: %s", drone.name)
            backend.remove_drone(drone.name)

        elif action == 'update' and backend.swarm.discovered_drones:
            drone = random.choice(backend.swarm.discovered_drones)
            new_battery = f'{random.randint(20, 100)}%'
            drone_data = {
                'name': drone.name,
                'ip': drone.ip,
                'port': drone.port,
                'properties': {
                    'hostname': drone.hostname,
                    'status': random.choice(['Active', 'Idle', 'Charging']),
                    'battery': new_battery
                },
                'commands_supported': drone.commands_supported
            }
            logger.info("Updating drone: %s with new battery %s", drone.name, new_battery)
            backend.update_drone(drone_data, drone.name)


# =============================================================================
# Dialog Base and Derived Classes
# =============================================================================

class BaseDialog:
    """
    A base class for modal dialogs. Creates a Toplevel window with a container
    using CustomTkinter for a consistent look.
    """
    def __init__(self):
        self.dialog = Toplevel()
        self.dialog.resizable(False, False)
        # Center the dialog (optional: adjust geometry as needed)
        self.dialog.grab_set()
        self.result = None

        # Create a container frame inside the Toplevel using CustomTkinter
        self.container = ctk.CTkFrame(self.dialog)
        self.container.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        self.dialog.grid_columnconfigure(0, weight=1)
        self.dialog.grid_rowconfigure(0, weight=1)
        self.container.grid_columnconfigure(0, weight=1)

    def _submit(self):
        """Called when the dialog is submitted."""
        self.dialog.destroy()

    def _cancel(self):
        """Called when the dialog is cancelled."""
        self.result = None
        self.dialog.destroy()

    def wait_for_result(self):
        """
        Blocks execution until the dialog is closed, then returns the result.
        """
        self.dialog.wait_window()
        return self.result


class TextInputDialog(BaseDialog):
    """
    A dialog prompting the user for text input.
    """
    def __init__(self, title, prompt, private=False):
        self.private = private
        self.title = title
        self.prompt = prompt
        super().__init__()
        self._build_ui()

    def _build_ui(self):
        self.dialog.title(self.title)
        # Use CustomTkinter widgets for a modern look.
        prompt_label = ctk.CTkLabel(self.container, text=self.prompt)
        prompt_label.grid(row=0, column=0, pady=10, sticky="w")

        self.text_input = StringVar()
        entry = ctk.CTkEntry(
            self.container,
            textvariable=self.text_input,
            show="*" if self.private else "",
            width=200
        )
        entry.grid(row=1, column=0, pady=5, sticky="ew")
        entry.bind("<Return>", lambda event: self._handle_submit())

        submit_button = ctk.CTkButton(self.container, text="Submit", command=self._handle_submit)
        submit_button.grid(row=2, column=0, pady=10, sticky="ew")

        cancel_button = ctk.CTkButton(self.container, text="Cancel", command=self._cancel)
        cancel_button.grid(row=3, column=0, pady=10, sticky="ew")

        self.container.grid_rowconfigure(4, weight=1)

    def _handle_submit(self):
        self.result = self.text_input.get()
        super()._submit()

    def get_input(self):
        """Displays the dialog and returns the user input."""
        return self.wait_for_result()


class ExecutableSelectorDialog(BaseDialog):
    """
    A dialog for selecting an executable. Offers a list of predefined options as
    well as an entry for a custom executable.
    """
    def __init__(self, drone, predefined_options):
        self.drone = drone
        self.predefined_options = predefined_options
        super().__init__()
        self._build_ui()

    def _build_ui(self):
        self.dialog.title("Select Executable")

        title_label = ctk.CTkLabel(self.container, text="Select an executable:")
        title_label.grid(row=0, column=0, pady=10, sticky="w")

        row_index = 1
        for option in self.predefined_options:
            btn = ctk.CTkButton(
                self.container,
                text=option,
                command=lambda opt=option: self._select_predefined(opt)
            )
            btn.grid(row=row_index, column=0, pady=5, sticky="ew")
            row_index += 1

        custom_label = ctk.CTkLabel(self.container, text="Or enter a custom executable:")
        custom_label.grid(row=row_index, column=0, pady=10, sticky="w")
        row_index += 1

        self.custom_executable = StringVar()
        custom_entry = ctk.CTkEntry(self.container, textvariable=self.custom_executable, width=200)
        custom_entry.grid(row=row_index, column=0, pady=5, sticky="ew")
        row_index += 1

        submit_custom_btn = ctk.CTkButton(self.container, text="Submit Custom", command=self._submit_custom)
        submit_custom_btn.grid(row=row_index, column=0, pady=10, sticky="ew")
        row_index += 1

        cancel_btn = ctk.CTkButton(self.container, text="Cancel", command=self._cancel)
        cancel_btn.grid(row=row_index, column=0, pady=10, sticky="ew")

        self.container.grid_rowconfigure(row_index + 1, weight=1)

    def _select_predefined(self, option):
        self.result = option
        super()._submit()

    def _submit_custom(self):
        self.result = self.custom_executable.get()
        super()._submit()

    def get_selection(self):
        """Displays the dialog and returns the selected executable."""
        return self.wait_for_result()


class DirectorySelectorDialog(BaseDialog):
    """
    A dialog for selecting a directory.
    """
    def __init__(self, initial_directory=None):
        self.initial_directory = initial_directory
        super().__init__()
        self._build_ui()

    def _build_ui(self):
        self.dialog.title("Select Directory")

        label = ctk.CTkLabel(self.container, text="Select a directory:")
        label.grid(row=0, column=0, pady=10, sticky="w")

        self.directory_var = StringVar()
        entry = ctk.CTkEntry(self.container, textvariable=self.directory_var, width=200)
        entry.grid(row=1, column=0, pady=5, sticky="ew")

        browse_btn = ctk.CTkButton(self.container, text="Browse", command=self._browse)
        browse_btn.grid(row=2, column=0, pady=10, sticky="ew")

        submit_btn = ctk.CTkButton(self.container, text="Submit", command=self._handle_submit)
        submit_btn.grid(row=3, column=0, pady=5, sticky="ew")

        cancel_btn = ctk.CTkButton(self.container, text="Cancel", command=self._cancel)
        cancel_btn.grid(row=4, column=0, pady=5, sticky="ew")

        self.container.grid_rowconfigure(5, weight=1)

    def _browse(self):
        directory = filedialog.askdirectory(initialdir=self.initial_directory or '/')
        if directory:
            self.directory_var.set(directory)

    def _handle_submit(self):
        self.result = self.directory_var.get()
        super()._submit()

    def get_selection(self):
        """Displays the dialog and returns the selected directory."""
        return self.wait_for_result()
