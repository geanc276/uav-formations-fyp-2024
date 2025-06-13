# pages.py

import customtkinter as ctk
import tkinter as tk
from .command_scrollbox import CommandScrollbox
from .scrollbox import Scrollbox, MultiSelectScrollbox, InfoWidget, InfoWidgetScrollbox, LaunchedDroneWidget
from backend.commands import get_command_class
from .buttons import DroneButton, CommandButton, SelectableButton, SelectableDroneButton, DroneButtonInfoBox, InfoBox
import subprocess
from tkinter import messagebox
import threading
import os
from utils.helpers import run_colcon_build, run_zip_script
import logging
import shutil
import time
import platform
from PIL import Image  # Ensure Pillow is installed: pip install pillow
from utils import DirectorySelectorDialog, TextInputDialog
# Matplotlib imports for 3D visualization
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Configure logging
logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

# Set global appearance and theme
ctk.set_appearance_mode("System")  # Options: "Light", "Dark", "System"
ctk.set_default_color_theme("green")  # Options: "blue", "green", "dark-blue"


def not_implemented(func):
    def wrapper(*args, **kwargs):
        logger.info(f"Page: {args[0]}: {func.__name__} not implemented yet.")
        func(*args, **kwargs)
    return wrapper


class Page(ctk.CTkFrame):
    def __init__(self, master, name, backend):
        super().__init__(master)
        self.name = name
        self.backend = backend
        self.configure(fg_color="transparent")
        self.accessed_drones = []

        # Use grid instead of pack for main layout
        self.grid(row=0, column=0, sticky="nsew")
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)

        self.main_frame = ctk.CTkScrollableFrame(self, fg_color="transparent")
        self.main_frame.grid(row=0, column=0, sticky="nsew", padx=20, pady=20)
        self.main_frame.grid_columnconfigure(0, weight=1)
        self.main_frame.grid_rowconfigure(0, weight=1)

        self.create_widgets()

    def delete_widgets(self):
        for widget in self.main_frame.winfo_children():
            widget.destroy()
            logger.debug("Widget destroyed.")

    def print_not_implemented(self):
        logger.info("Not implemented yet.")

    def get_drones(self):
        return self.backend.get_all_drones()

    def create_header(self, text, row=0, pady=(0, 20)):
        """Helper to create a header section consistently across pages."""
        header_frame = ctk.CTkFrame(self.main_frame, fg_color="transparent")
        header_frame.grid(row=row, column=0, sticky="ew", pady=pady)
        header_frame.grid_columnconfigure(0, weight=1)
        header_label = ctk.CTkLabel(
            header_frame,
            text=text,
            font=ctk.CTkFont(size=24, weight="bold")
        )
        header_label.grid(row=0, column=0, pady=10, sticky="ew")
        return header_frame, header_label

    @not_implemented
    def create_widgets(self):
        # To be overridden by subclasses
        pass

    @not_implemented
    def reload_drones(self):
        # To be overridden by subclasses
        pass

    def update(self):
        self.accessed_drones = self.get_drones()
        self.accessed_drones.sort(key=lambda x: x.name)
        self.reload_drones()


class HomePage(Page):
    def create_widgets(self):
        try:
            logger.debug(f"Creating widgets for {self.name}")
            # Create header using the helper method
            self.create_header("Home Page", row=0)

            # Create a Scrollbox for drones at row 1
            scrollbox_frame = ctk.CTkFrame(self.main_frame, corner_radius=8)
            scrollbox_frame.grid(row=1, column=0, sticky="nsew", pady=(0, 20))
            scrollbox_frame.grid_columnconfigure(0, weight=1)
            scrollbox_frame.grid_rowconfigure(0, weight=1)

            self.scrollbox = Scrollbox(
                scrollbox_frame,
                width=600,
                height=400,
                max_columns=8,
                min_button_width=120,
                min_button_height=60,
                padding_x=10,
                padding_y=10
            )
            self.scrollbox.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

            # Add drone buttons dynamically
            for drone in self.get_drones():
                self.add_drone_button(drone)

            logger.info("HomePage widgets created successfully.")

        except Exception as e:
            logger.error(f"Error creating HomePage widgets: {e}")
            messagebox.showerror("Error", f"Failed to create HomePage widgets:\n{e}")

    def reload_drones(self):
        self.scrollbox.clear()
        for drone in self.accessed_drones:
            self.add_drone_button(drone)

    def add_drone_button(self, drone):
        try:
            drone_button_info = DroneButtonInfoBox(
                master=self.scrollbox.scrollable_frame,
                drone=drone,
                action=self.backend.navigate_to_drone_page,
                fg_color="#4CAF50",       # Example color
                width=300,
                height=100  # Increased height to accommodate info box
            )
            self.scrollbox.add_button(drone_button_info)
            logger.info(f"Drone button added for {drone.name}.")
        except Exception as e:
            logger.error(f"Error adding drone button for {drone.name}: {e}")

    def remove_drone_button(self, drone):
        try:
            for button in self.scrollbox.scrollable_frame.winfo_children():
                if hasattr(button, 'drone') and button.drone == drone:
                    button.destroy()
                    logger.info(f"Drone button for {drone.name} removed.")
                    break
        except Exception as e:
            logger.error(f"Error removing drone button for {drone.name}: {e}")


class SettingsPage(Page):
    def create_widgets(self):
        try:
            self.create_header("Settings", row=0)

            # Settings Controls Frame at row 1
            controls_frame = ctk.CTkFrame(self.main_frame, corner_radius=8)
            controls_frame.grid(row=1, column=0, sticky="nsew", pady=(0, 20), padx=10)
            controls_frame.grid_columnconfigure(0, weight=1)

            self.enable_feature_toggle = ctk.CTkSwitch(
                controls_frame,
                text="Enable Feature",
                command=self.toggle_feature
            )
            self.enable_feature_toggle.grid(row=0, column=0, pady=10, padx=20, sticky="w")

            self.settings_slider = ctk.CTkSlider(
                controls_frame,
                from_=0,
                to=100,
                number_of_steps=100,
                command=self.slider_changed
            )
            self.settings_slider.grid(row=1, column=0, pady=10, padx=20, sticky="ew")

            child_button = ctk.CTkButton(
                controls_frame,
                text="Go to Settings Child Page",
                command=lambda: self.backend.navigate_to("SettingsChild"),
                fg_color="#32CD32",
                hover_color="#228B22",
                corner_radius=10,
                text_color="white"
            )
            child_button.grid(row=2, column=0, pady=20, padx=20, sticky="ew")

            logger.info("SettingsPage widgets created successfully.")

        except Exception as e:
            logger.error(f"Error creating SettingsPage widgets: {e}")
            messagebox.showerror("Error", f"Failed to create SettingsPage widgets:\n{e}")

    def toggle_feature(self):
        state = self.enable_feature_toggle.get()
        logger.info(f"Feature enabled: {state}")

    def slider_changed(self, value):
        logger.info(f"Slider value changed to: {value}")


class SettingsChildPage(Page):
    def create_widgets(self):
        try:
            self.create_header("Settings - Child Page", row=0)

            # Content Frame at row 1
            content_frame = ctk.CTkFrame(self.main_frame, corner_radius=8)
            content_frame.grid(row=1, column=0, sticky="nsew", pady=(0, 20), padx=10)
            content_frame.grid_columnconfigure(0, weight=1)

            self.settings_entry = ctk.CTkEntry(
                content_frame,
                placeholder_text="Enter some settings",
                width=300
            )
            self.settings_entry.grid(row=0, column=0, pady=10, padx=20, sticky="ew")

            save_button = ctk.CTkButton(
                content_frame,
                text="Save Settings",
                command=self.save_settings,
                fg_color="#32CD32",
                hover_color="#228B22",
                corner_radius=10,
                text_color="white"
            )
            save_button.grid(row=1, column=0, pady=10, padx=20, sticky="ew")

            logger.info("SettingsChildPage widgets created successfully.")

        except Exception as e:
            logger.error(f"Error creating SettingsChildPage widgets: {e}")
            messagebox.showerror("Error", f"Failed to create SettingsChildPage widgets:\n{e}")

    def save_settings(self):
        settings = self.settings_entry.get()
        logger.info(f"Settings saved: {settings}")
        messagebox.showinfo("Settings", "Settings have been saved successfully.")


class AboutPage(Page):
    def create_widgets(self):
        try:
            self.create_header("About", row=0)

            # Content Frame at row 1
            content_frame = ctk.CTkFrame(self.main_frame, corner_radius=8)
            content_frame.grid(row=1, column=0, sticky="nsew", pady=(0, 20), padx=10)
            content_frame.grid_columnconfigure(0, weight=1)

            info_label = ctk.CTkLabel(
                content_frame,
                text="This is a sample application using CustomTkinter.",
                font=ctk.CTkFont(size=14),
                wraplength=500
            )
            info_label.grid(row=0, column=0, pady=10, padx=20, sticky="ew")

            more_info_button = ctk.CTkButton(
                content_frame,
                text="More Info",
                command=self.more_info,
                fg_color="#32CD32",
                hover_color="#228B22",
                corner_radius=10,
                text_color="white"
            )
            more_info_button.grid(row=1, column=0, pady=10, padx=20, sticky="ew")

            logger.info("AboutPage widgets created successfully.")

        except Exception as e:
            logger.error(f"Error creating AboutPage widgets: {e}")
            messagebox.showerror("Error", f"Failed to create AboutPage widgets:\n{e}")

    def more_info(self):
        logger.info("More Info button clicked.")
        messagebox.showinfo("More Information", "Additional information can be displayed here.")


class DronePage(Page):
    def __init__(self, master, name, backend, drone=None):
        self.drone = drone
        super().__init__(master, name, backend)

    def change_drone(self, drone):
        self.drone = drone
        self.create_widgets()

    def create_widgets(self):
        if not self.drone:
            logger.warning("DronePage created without a drone.")
            return
        try:
            # Clear existing widgets using the base class method (instead of grid_remove)
            # if self.main_frame.winfo_children():
            #     self.delete_widgets()
            
            for widget in self.main_frame.winfo_children():
                widget.grid_remove()


            # Build info section and commands section using methods provided by drone.page
            self.drone.page.build_info_section(self.main_frame, self.open_ssh_terminal)
            self.drone.page.build_commands_section(self.main_frame)

            logger.info(f"DronePage for {self.drone.name} created successfully.")
        except Exception as e:
            logger.error(f"Error creating DronePage widgets: {e}")
            messagebox.showerror("Error", f"Failed to create DronePage widgets:\n{e}")

    def open_ssh_terminal(self):
        drone_ip = self.drone.connection.ip
        if not drone_ip:
            messagebox.showerror("Error", "Drone IP address is missing.")
            return
        user_name = getattr(self.drone.connection, 'user_name', 'drone_swarm')
        ssh_command = f"ssh {user_name}@{drone_ip}"
        try:
            if os.name == 'nt':  # Windows
                subprocess.Popen(["cmd.exe", "/c", ssh_command])
            elif platform.system() == "Darwin":  # macOS
                subprocess.Popen([
                    "osascript",
                    "-e",
                    f'tell application "Terminal" to do script "{ssh_command}"'
                ])
            elif os.name == 'posix':  # Linux/other POSIX
                terminal_emulators = [
                    'gnome-terminal', 'konsole', 'xfce4-terminal',
                    'xterm', 'mate-terminal', 'tilix'
                ]
                for term in terminal_emulators:
                    if shutil.which(term):
                        subprocess.Popen([term, "--", "bash", "-c", ssh_command + "; exec bash"])
                        break
                else:
                    subprocess.Popen(["xterm", "-e", ssh_command])
            else:
                messagebox.showerror("Error", "Unsupported operating system for SSH.")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to open SSH terminal: {e}")
            logger.error(f"Failed to open SSH terminal for drone {self.drone.name}: {e}")

    # Moved out of open_ssh_terminal to be proper methods of DronePage
    def reload_drones(self, drone_name):
        if self.drone.name == drone_name:
            self.create_widgets()

    def update(self):
        if self.drone:
            self.reload_drones(self.drone.name)


class BuildPage(Page):
    def create_widgets(self):
        try:
            self.create_header("Build and Zip", row=0)

            # Content Frame at row 1
            content_frame = ctk.CTkFrame(self.main_frame, corner_radius=8)
            content_frame.grid(row=1, column=0, sticky="nsew", pady=(0, 20), padx=10)
            content_frame.grid_columnconfigure(0, weight=1)

            # Progress Bar
            self.progress_var = ctk.DoubleVar()
            self.progress_bar = ctk.CTkProgressBar(
                content_frame,
                variable=self.progress_var,
                width=400,
                height=20
            )
            self.progress_bar.grid(row=0, column=0, pady=10, padx=20, sticky="ew")
            self.progress_bar.set(0)

            # Status Label
            self.status_label = ctk.CTkLabel(
                content_frame,
                text="Idle.",
                font=ctk.CTkFont(size=14)
            )
            self.status_label.grid(row=1, column=0, pady=10, padx=20, sticky="ew")

            # Buttons Frame at row 2
            buttons_frame = ctk.CTkFrame(content_frame, fg_color="transparent")
            buttons_frame.grid(row=2, column=0, pady=10, padx=20, sticky="ew")
            buttons_frame.grid_columnconfigure(0, weight=1)
            buttons_frame.grid_columnconfigure(1, weight=1)

            self.build_button = ctk.CTkButton(
                buttons_frame,
                text="Run colcon build",
                command=self.start_colcon_build,
                fg_color="#32CD32",
                hover_color="#228B22",
                corner_radius=10,
                text_color="white"
            )
            self.build_button.grid(row=0, column=0, padx=10, pady=10, sticky="ew")

            self.zip_button = ctk.CTkButton(
                buttons_frame,
                text="Run zip script",
                command=self.start_zip_script,
                fg_color="#1E90FF",
                hover_color="#1C86EE",
                corner_radius=10,
                text_color="white"
            )
            self.zip_button.grid(row=0, column=1, padx=10, pady=10, sticky="ew")

            logger.info("BuildPage widgets created successfully.")
        except Exception as e:
            logger.error(f"Error creating BuildPage widgets: {e}")
            messagebox.showerror("Error", f"Failed to create BuildPage widgets:\n{e}")

    def start_colcon_build(self):
        self.build_button.configure(state="disabled")
        self.status_label.configure(text="Starting colcon build...")
        self.progress_bar.set(0)
        threading.Thread(target=self.run_colcon_build_thread, daemon=True).start()

    def run_colcon_build_thread(self):
        success = run_colcon_build(self.update_progress_with_status)
        if success:
            self.status_label.configure(text="colcon build completed successfully.")
            self.zip_button.configure(state="normal")
        else:
            self.status_label.configure(text="colcon build failed.")
        self.build_button.configure(state="normal")

    def update_progress_with_status(self, progress, status_message=None):
        self.progress_var.set(progress / 100)
        self.progress_bar.update_idletasks()
        if status_message:
            self.status_label.configure(text=status_message)

    def start_zip_script(self):
        self.zip_button.configure(state="disabled")
        self.status_label.configure(text="Running zip script...")
        threading.Thread(target=self.run_zip_script_thread, daemon=True).start()

    def run_zip_script_thread(self):
        input_dialog = TextInputDialog("Zip Script", "ARM install? (Y/N)")
        arm = input_dialog.get_input()

        if arm == "Y":
            success = run_zip_script("arm_install")
        else:
            success = run_zip_script("install")

        if success:
            self.status_label.configure(text="Zip script completed successfully.")
        else:
            self.status_label.configure(text="Zip script failed.")
        self.zip_button.configure(state="normal")


class DroneBuildPage(Page):
    def __init__(self, master, name, backend, drone=None):
        self.current_supported_drones = []
        self.progress_vars = {}
        self.status_labels = {}
        self.build_buttons = {}
        self.drone_frames = {}
        super().__init__(master, name, backend)

    def create_widgets(self):
        try:
            self.create_header("Drone Build", row=0)

            # Instructions Label at row 1
            instructions_label = ctk.CTkLabel(
                self.main_frame,
                text="Select drones to build and deploy the application.",
                font=ctk.CTkFont(size=14),
                wraplength=600
            )
            instructions_label.grid(row=1, column=0, pady=10, sticky="ew")

            # Scrollbox Frame at row 2
            scrollbox_frame = ctk.CTkFrame(self.main_frame, corner_radius=8)
            scrollbox_frame.grid(row=2, column=0, sticky="nsew", pady=(0, 20), padx=10)
            scrollbox_frame.grid_columnconfigure(0, weight=1)
            scrollbox_frame.grid_rowconfigure(0, weight=1)

            self.drone_scrollbox = Scrollbox(
                scrollbox_frame,
                width=600,
                height=400,
                max_columns=1,
                min_button_width=150,
                min_button_height=40,
                padding_x=10,
                padding_y=10
            )
            self.drone_scrollbox.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

            self.current_supported_drones = list(self.backend.get_swarm_builders())

            for drone in self.get_drones():
                self.create_drone_widgets(drone)

            logger.info("DroneBuildPage widgets created successfully.")

        except Exception as e:
            logger.error(f"Error creating DroneBuildPage widgets: {e}")
            messagebox.showerror("Error", f"Failed to create DroneBuildPage widgets:\n{e}")

    def reload_drones(self):
        self.drone_scrollbox.clear()
        for frame in self.drone_frames.values():
            frame.destroy()
        self.drone_frames = {}
        for drone in self.accessed_drones:
            self.create_drone_widgets(drone)

    def create_drone_widgets(self, drone):
        try:
            drone_frame = ctk.CTkFrame(self.drone_scrollbox.scrollable_frame, corner_radius=8)
            drone_frame.grid(sticky="ew", padx=5, pady=5)
            drone_frame.grid_columnconfigure(0, weight=1)

            drone_label = ctk.CTkLabel(
                drone_frame,
                text=f"Drone: {drone.name}",
                font=ctk.CTkFont(size=16, weight="bold")
            )
            drone_label.grid(row=0, column=0, pady=(5, 2), padx=10, sticky="w")

            status_label = ctk.CTkLabel(
                drone_frame,
                text="Ready to build.",
                font=ctk.CTkFont(size=14)
            )
            status_label.grid(row=1, column=0, pady=(0, 5), padx=10, sticky="w")
            self.status_labels[drone.name] = status_label

            progress_var = ctk.DoubleVar()
            progress_bar = ctk.CTkProgressBar(drone_frame, variable=progress_var, width=400)
            progress_bar.grid(row=2, column=0, padx=10, pady=(0, 5), sticky="ew")
            progress_bar.set(0)
            self.progress_vars[drone.name] = progress_var

            build_button = ctk.CTkButton(
                drone_frame,
                text="Start Build",
                command=lambda d=drone: self.start_drone_build(d),
                fg_color="#32CD32",
                hover_color="#228B22",
                corner_radius=10,
                text_color="white"
            )
            build_button.grid(row=3, column=0, pady=(0, 10), sticky="ew")
            self.build_buttons[drone.name] = build_button

            self.drone_frames[drone.name] = drone_frame
            logger.info(f"Build controls created for drone {drone.name}.")

        except Exception as e:
            logger.error(f"Error creating build widgets for drone {drone.name}: {e}")

    def start_drone_build(self, drone):
        try:
            self.build_buttons[drone.name].configure(state='disabled')
            self.status_labels[drone.name].configure(text="Starting build...")
            self.progress_vars[drone.name].set(0)
            threading.Thread(target=self.run_drone_build_thread, args=(drone,), daemon=True).start()
            logger.info(f"Build process started for drone {drone.name}.")
        except Exception as e:
            logger.error(f"Error starting build for drone {drone.name}: {e}")

    def run_drone_build_thread(self, drone):
        try:
            logger.info(f"Starting build for drone {drone.name}")
            drone.connection.build_pls_command.execute()
            logger.info(f"Build command executed for drone {drone.name}")

            stages = [
                ("Synchronizing files...", 0, 33),
                ("Building application...", 33, 66),
                ("Downloading artifacts...", 66, 100)
            ]
            for stage_text, start, end in stages:
                self.update_status(drone, stage_text)
                for progress in range(start, end + 1):
                    self.update_progress(drone, progress)
                    time.sleep(0.05)

            self.update_status(drone, "Build completed successfully.")
            logger.info(f"Build completed for drone {drone.name}")
        except Exception as e:
            self.update_status(drone, "Build failed.")
            logger.error(f"Build failed for drone {drone.name}: {e}")
            messagebox.showerror("Error", f"Build failed for {drone.name}:\n{e}")
        finally:
            self.enable_build_button(drone)

    def update_status(self, drone, status_text):
        def update():
            self.status_labels[drone.name].configure(text=status_text)
        self.main_frame.after(0, update)

    def update_progress(self, drone, progress):
        def update():
            self.progress_vars[drone.name].set(progress / 100)
        self.main_frame.after(0, update)

    def enable_build_button(self, drone):
        def update():
            self.build_buttons[drone.name].configure(state='normal')
        self.main_frame.after(0, update)


class LaunchPage(Page):
    def __init__(self, master, name, backend):
        super().__init__(master, name, backend)

    def create_widgets(self):
        try:
            self.create_header("Launch Page", row=0)

            # Launch Commands Section at row 1
            scrollbox_frame = ctk.CTkFrame(self.main_frame, corner_radius=8)
            scrollbox_frame.grid(row=1, column=0, sticky="nsew", pady=(0, 20))
            scrollbox_frame.grid_columnconfigure(0, weight=1)
            scrollbox_frame.grid_rowconfigure(0, weight=1)

            self.launch_scrollbox = MultiSelectScrollbox(
                scrollbox_frame,
                width=580,
                height=300,
                max_columns=1,
                min_button_width=180,
                min_button_height=50,
                padding_x=15,
                padding_y=15
            )
            self.launch_scrollbox.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

            # Control Buttons Section at row 2
            controls_frame = ctk.CTkFrame(self.main_frame, fg_color="transparent")
            controls_frame.grid(row=2, column=0, sticky="ew", pady=(0, 20))
            controls_frame.grid_columnconfigure(0, weight=1)
            controls_frame.grid_columnconfigure(1, weight=1)

            try:
                select_all_icon = ctk.CTkImage(Image.open("icons/select_all.png").resize((20, 20)))
                deselect_all_icon = ctk.CTkImage(Image.open("icons/deselect_all.png").resize((20, 20)))
                launch_icon = ctk.CTkImage(Image.open("icons/launch.png").resize((25, 25)))
            except Exception as icon_error:
                logger.warning(f"Icon loading failed: {icon_error}")
                select_all_icon = None
                deselect_all_icon = None
                launch_icon = None

            select_all_button = ctk.CTkButton(
                controls_frame,
                text="Select All",
                command=self.launch_scrollbox.select_all,
                fg_color="#32CD32",
                hover_color="#228B22",
                width=140,
                height=40,
                corner_radius=10,
                text_color="white",
                image=select_all_icon,
                compound="left"
            )
            select_all_button.grid(row=0, column=0, padx=10, pady=10, sticky="ew")

            deselect_all_button = ctk.CTkButton(
                controls_frame,
                text="Deselect All",
                command=self.launch_scrollbox.deselect_all,
                fg_color="#FF4500",
                hover_color="#FF6347",
                width=140,
                height=40,
                corner_radius=10,
                text_color="white",
                image=deselect_all_icon,
                compound="left"
            )
            deselect_all_button.grid(row=0, column=1, padx=10, pady=10, sticky="ew")

            launch_button = ctk.CTkButton(
                self.main_frame,
                text="Launch Selected",
                command=self.navigate_to_config,
                fg_color="#1E90FF",
                hover_color="#1C86EE",
                width=200,
                height=50,
                corner_radius=10,
                text_color="white",
                font=ctk.CTkFont(size=16, weight="bold"),
                image=launch_icon,
                compound="left"
            )
            launch_button.grid(row=3, column=0, pady=10, sticky="ew")

            logger.info("LaunchPage widgets created successfully.")

        except Exception as e:
            logger.error(f"Error creating LaunchPage widgets: {e}")
            messagebox.showerror("Error", f"Failed to create LaunchPage widgets:\n{e}")

    def add_drone(self, drone):
        try:
            drone_button = SelectableDroneButton(self.launch_scrollbox.scrollable_frame, drone)
            self.launch_scrollbox.add_button(drone_button)
            logger.info(f"SelectableDroneButton added for {drone.name}.")
        except Exception as e:
            logger.error(f"Error adding SelectableDroneButton for {drone.name}: {e}")

    def launch_selected(self):
        try:
            drones = [drone_button.drone for drone_button in self.launch_scrollbox.get_selected_items()]
            logger.info(f"Launching drones: {[drone.name for drone in drones]}")
            self.backend.launch_drones(drones)
            messagebox.showinfo("Launch", f"Launching {len(drones)} drones successfully.")
        except Exception as e:
            logger.error(f"Error launching selected drones: {e}")
            messagebox.showerror("Error", f"Failed to launch selected drones:\n{e}")

    def navigate_to_config(self):
        selected_drones = [drone_button.drone for drone_button in self.launch_scrollbox.get_selected_items()]
        logger.debug(f"Selected drones: {selected_drones}")
        self.backend.app.pages["LaunchConfig"].change_drones(selected_drones)
        self.backend.navigate_to("LaunchConfig")

    def reload_drones(self):
        self.launch_scrollbox.clear()
        # self.launch_scrollbox.clear_selection()
        self.launch_scrollbox.deselect_all()
        for drone in self.accessed_drones:
            self.add_drone(drone)


class LaunchConfigPage(Page):
    """A page for configuring drone launches."""
    def __init__(self, master, name, backend, selected_drones=None):
        # No need to reassign master, name, or backend here because the base __init__ does that.
        self.selected_drones = selected_drones or []
        self.config_widgets = {}
        self.json_dir = None
        self.json_buttons_frame = None
        super().__init__(master, name, backend)

    def create_widgets(self):
        try:
            self.create_header("Launch Configuration", row=0)

            # JSON Directory Selection at row 1
            json_dir_frame = ctk.CTkFrame(self.main_frame, fg_color="transparent")
            json_dir_frame.grid(row=1, column=0, sticky="ew", pady=(0, 20))
            json_dir_frame.grid_columnconfigure(0, weight=1)

            browse_button = ctk.CTkButton(
                json_dir_frame,
                text="Browse JSON Directory",
                command=self.browse_json_dir,
                fg_color="#1E90FF",
                hover_color="#1C86EE",
                width=200,
                height=40,
                corner_radius=10,
                text_color="white",
                font=ctk.CTkFont(size=14)
            )
            browse_button.grid(row=0, column=0, pady=10, padx=10, sticky="w")

            self.json_buttons_frame = ctk.CTkFrame(json_dir_frame, fg_color="transparent")
            self.json_buttons_frame.grid(row=1, column=0, sticky="ew", pady=10, padx=10)
            self.json_buttons_frame.grid_columnconfigure(0, weight=1)
            self.update_json_display()

            auto_assign_button = ctk.CTkButton(
                json_dir_frame,
                text="Auto Assign Configurations",
                command=self.auto_assign_configs,
                fg_color="#32CD32",
                hover_color="#228B22",
                width=200,
                height=40,
                corner_radius=10,
                text_color="white",
                font=ctk.CTkFont(size=14)
            )
            auto_assign_button.grid(row=2, column=0, pady=10, padx=10, sticky="w")

            # Drone Configuration at row 2
            drone_config_frame = ctk.CTkFrame(self.main_frame, fg_color="transparent")
            drone_config_frame.grid(row=2, column=0, sticky="nsew", pady=(0, 20))
            drone_config_frame.grid_columnconfigure(0, weight=1)
            drone_config_frame.grid_rowconfigure(1, weight=1)

            drone_config_label = ctk.CTkLabel(
                drone_config_frame,
                text="Drone Configurations:",
                font=ctk.CTkFont(size=18, weight="bold")
            )
            drone_config_label.grid(row=0, column=0, pady=(0, 10), padx=10, sticky="w")

            self.drone_info_scrollbox = InfoWidgetScrollbox(
                drone_config_frame,
                width=580,
                height=300,
                max_columns=4,
                padding_x=15,
                padding_y=15
            )
            self.drone_info_scrollbox.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

            if self.selected_drones:
                for drone in self.selected_drones:
                    self.create_drone_config_widgets(drone)

            # Control Buttons at row 3
            controls_frame = ctk.CTkFrame(self.main_frame, fg_color="transparent")
            controls_frame.grid(row=3, column=0, sticky="ew", pady=(20, 0))
            controls_frame.grid_columnconfigure(0, weight=1)

            submit_button = ctk.CTkButton(
                controls_frame,
                text="Launch",
                command=self.launch,
                fg_color="#28a745",
                hover_color="#218838",
                width=100,
                height=40,
                corner_radius=10,
                text_color="white",
                font=ctk.CTkFont(size=14)
            )
            submit_button.grid(row=0, column=0, padx=20, sticky="w")

            stop_button = ctk.CTkButton(
                controls_frame,
                text="Stop",
                command=self.stop,
                fg_color="#dc3545",
                hover_color="#c82333",
                width=100,
                height=40,
                corner_radius=10,
                text_color="white",
                font=ctk.CTkFont(size=14)
            )
            stop_button.grid(row=0, column=1, padx=20, sticky="w")

            logger.info("LaunchConfigPage widgets created successfully.")

        except Exception as e:
            logger.error(f"Error creating LaunchConfigPage widgets: {e}")
            messagebox.showerror("Error", f"Failed to create LaunchConfigPage widgets:\n{e}")

    def auto_assign_configs(self):
        if not self.selected_drones:
            messagebox.showwarning("No Drones Selected", "Please select drones before auto-assigning configurations.")
            return
        if not self.json_dir:
            messagebox.showwarning("No JSON Directory", "Please select a JSON directory before auto-assigning configurations.")
            return
        sorted_jsons = sorted(os.listdir(self.json_dir))
        if len(sorted_jsons) < len(self.selected_drones):
            messagebox.showwarning("Insufficient JSON Files", "Please provide enough JSON files for each selected drone.")
            return

        full_json_paths = [os.path.join(self.json_dir, json_file) for json_file in sorted_jsons]

        for drone in sorted(self.selected_drones, key=lambda x: x.name):
            self.backend.assign_config(drone, full_json_paths.pop(0))

    def browse_json_dir(self):
        try:
            dir_selector = DirectorySelectorDialog(initial_directory="../swarm_ws/configs")
            selected_dir = dir_selector.get_selection()
            if selected_dir:
                self.json_dir = selected_dir
                logger.info(f"Selected JSON directory: {self.json_dir}")
                self.update_json_display()
            else:
                logger.info("No JSON directory selected.")
                self.json_dir = None
                self.update_json_display()
        except Exception as e:
            logger.error(f"Error browsing JSON directory: {e}")
            messagebox.showerror("Error", f"Failed to browse JSON directory:\n{e}")

    def update_json_display(self):
        for widget in self.json_buttons_frame.winfo_children():
            widget.destroy()

        if self.json_dir and os.path.isdir(self.json_dir):
            json_files = [f for f in os.listdir(self.json_dir) if f.endswith(".json")]
            if json_files:
                row_index = 0
                for file in json_files:
                    file_path = os.path.join(self.json_dir, file)
                    file_button = ctk.CTkButton(
                        self.json_buttons_frame,
                        text=file,
                        command=lambda fp=file_path: self.handle_json_file(fp),
                        width=300,
                        height=50,
                        corner_radius=5,
                        fg_color="#6c757d",
                        hover_color="#5a6268",
                        text_color="white",
                        font=ctk.CTkFont(size=12)
                    )
                    file_button.grid(row=row_index, column=0, pady=2, padx=10, sticky="w")
                    row_index += 1
            else:
                no_files_label = ctk.CTkLabel(
                    self.json_buttons_frame,
                    text="No JSON files found in the selected directory.",
                    font=ctk.CTkFont(size=14),
                    text_color="red"
                )
                no_files_label.grid(row=0, column=0, pady=5, padx=10, sticky="w")
        else:
            no_dir_label = ctk.CTkLabel(
                self.json_buttons_frame,
                text="No JSON directory selected.",
                font=ctk.CTkFont(size=14),
                text_color="blue"
            )
            no_dir_label.grid(row=0, column=0, pady=5, padx=10, sticky="w")

    def handle_json_file(self, file_path):
        try:
            messagebox.showinfo("JSON File Selected", f"Selected JSON file:\n{file_path}")
            logger.info(f"JSON file selected: {file_path}")
        except Exception as e:
            logger.error(f"Error handling JSON file selection: {e}")
            messagebox.showerror("Error", f"Failed to handle JSON file selection:\n{e}")

    def create_drone_config_widgets(self, drone):
        widget = InfoWidget(
            self.drone_info_scrollbox.scrollable_frame,
            name=drone.name,
            info_lines=[
                f"IP: {drone.connection.ip}",
                f"Config: {drone.drone_status.config_file}",
                f"Launched: {drone.drone_status.launched}"
            ],
            width=10,
            height=150,
            font_size=16,
            fg_color="white",
            corner_radius=10
        )
        self.drone_info_scrollbox.add_info_widget(widget)
        self.config_widgets[drone.name + "_entry"] = widget

    def stop(self):
        self.backend.stop_drones(self.selected_drones)

    def launch(self):
        try:
            if not self.selected_drones:
                messagebox.showwarning("No Drones Selected", "Please select at least one drone to launch.")
                return
            self.backend.launch_drones(self.selected_drones)
        except Exception as e:
            logger.error(f"Error during launch configurations: {e}")
            messagebox.showerror("Error", f"Failed to launch configurations:\n{e}")

    def change_drones(self, selected_drones):
        logger.debug(f"Selected drones updated: {[drone.name for drone in selected_drones]}")
        self.selected_drones = selected_drones
        self.drone_info_scrollbox.clear_all_info_widgets()
        self.config_widgets.clear()
        for drone in self.selected_drones:
            self.create_drone_config_widgets(drone)
        logger.info("Drones updated on LaunchConfigPage.")

    def reload_drones(self):
        self.drone_info_scrollbox.clear()
        for widget in self.config_widgets.values():
            widget.destroy()
        for drone in self.selected_drones:
            self.create_drone_config_widgets(drone)

    def update(self):
        logger.debug("LaunchConfigPage update called.")
        self.create_widgets()


class UserInputPage(Page):
    def create_widgets(self):
        try:
            self.create_header("User Input Page", row=0)

            start_user_input_button = ctk.CTkButton(
                self.main_frame,
                text="Start User Input",
                command=self.start_user_input,
                fg_color="#32CD32",
                hover_color="#228B22",
                width=200,
                height=50,
                corner_radius=10,
                text_color="white",
                font=ctk.CTkFont(size=16, weight="bold")
            )
            start_user_input_button.grid(row=1, column=0, pady=10, sticky="ew")

            start_button = ctk.CTkButton(
                self.main_frame,
                text="Start",
                command=lambda: self.send_state("start"),
                fg_color="#32CD32",
                hover_color="#228B22",
                width=200,
                height=50,
                corner_radius=10,
                text_color="white",
                font=ctk.CTkFont(size=16, weight="bold")
            )
            start_button.grid(row=2, column=0, pady=10, sticky="ew")

            land_button = ctk.CTkButton(
                self.main_frame,
                text="Land",
                command=lambda: self.send_state("land"),
                fg_color="#32CD32",
                hover_color="#228B22",
                width=200,
                height=50,
                corner_radius=10,
                text_color="white",
                font=ctk.CTkFont(size=16, weight="bold")
            )
            land_button.grid(row=3, column=0, pady=10, sticky="ew")

            pause_button = ctk.CTkButton(
                self.main_frame,
                text="Pause",
                command=lambda: self.send_state("pause"),
                fg_color="#32CD32",
                hover_color="#228B22",
                width=200,
                height=50,
                corner_radius=10,
                text_color="white",
                font=ctk.CTkFont(size=16, weight="bold")
            )
            pause_button.grid(row=4, column=0, pady=10, sticky="ew")

            resume_button = ctk.CTkButton(
                self.main_frame,
                text="Resume",
                command=lambda: self.send_state("resume"),
                fg_color="#32CD32",
                hover_color="#228B22",
                width=200,
                height=50,
                corner_radius=10,
                text_color="white",
                font=ctk.CTkFont(size=16, weight="bold")
            )
            resume_button.grid(row=5, column=0, pady=10, sticky="ew")

        except Exception as e:
            logger.error(f"Error creating UserInputPage widgets: {e}")
            messagebox.showerror("Error", f"Failed to create UserInputPage widgets:\n{e}")

    def start_user_input(self):
        logger.info("Start User Input")
        self.backend.open_user_input_window()

    def send_state(self, state):
        logger.info("send_state called")
        self.backend.send_state(state)





class CurrentlyLaunchedPage(Page):
    """
    A page that displays currently launched drones in a scrollable list.
    
    Each drone is represented by a LaunchedDroneWidget (with individual Stop and SSH buttons).
    A "Stop All" button is provided at the bottom. The page uses the common header style
    and layout of your other pages.
    """
    def create_widgets(self):
        try:
            # Create a header using the common Page helper.
            self.create_header("Currently Launched Drones", row=0)

            # Create a container frame for the scrollbox.
            container = ctk.CTkFrame(self.main_frame, corner_radius=8)
            container.grid(row=1, column=0, sticky="nsew", padx=20, pady=(0, 20))
            container.grid_columnconfigure(0, weight=1)
            container.grid_rowconfigure(0, weight=1)

            # Create the scrollable box to hold LaunchedDroneWidget items.
            self.drone_scrollbox = Scrollbox(
                container,
                width=800,
                height=400,
                max_columns=1,
                padding_x=10,
                padding_y=10
            )
            self.drone_scrollbox.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)

            # Create the "Stop All" button at the bottom.
            self.stop_all_button = ctk.CTkButton(
                self.main_frame,
                text="Stop All",
                fg_color="#dc3545",
                hover_color="#c82333",
                command=self.stop_all
            )
            self.stop_all_button.grid(row=2, column=0, pady=10, padx=20, sticky="ew")

            self.refresh()
            logger.info("CurrentlyLaunchedPage widgets created successfully.")
        except Exception as e:
            logger.error("Error creating CurrentlyLaunchedPage widgets: %s", e)

    def refresh(self):
        """
        Refresh the scrollbox with the current list of launched drones.
        """
        self.drone_scrollbox.clear()
        launched_drones = self.backend.get_currently_launched_drones()
        logger.debug("Refreshing launched drones; count: %d", len(launched_drones))
        for drone in launched_drones:
            widget = LaunchedDroneWidget(
                master=self.drone_scrollbox.scrollable_frame,
                drone=drone,
                stop_callback=self.stop_drone,
                ssh_callback=self.open_ssh_for_drone
            )
            self.drone_scrollbox.add_button(widget)

    def stop_drone(self, drone):
        """
        Callback to stop a specific drone.
        """
        try:
            drone.stop()
            logger.info("Stopped drone: %s", drone.name)
        except Exception as e:
            logger.error("Error stopping drone %s: %s", drone.name, e)
        finally:
            self.refresh()

    def open_ssh_for_drone(self, drone):
        """
        Callback to open an SSH terminal for a specific drone.
        """
        try:
            drone.open_ssh_terminal()
        except Exception as e:
            logger.error("Error opening SSH for drone %s: %s", drone.name, e)

    def stop_all(self):
        """
        Stops all currently launched drones and refreshes the list.
        """
        try:
            for drone in self.backend.get_currently_launched_drones():
                drone.stop()
            logger.info("Stopped all drones.")
        except Exception as e:
            logger.error("Error stopping all drones: %s", e)
        finally:
            self.refresh()

    def update(self):
        """
        Update method called periodically or after backend changes.
        """
        self.refresh()











import customtkinter as ctk
from tkinter import messagebox
import math
import os
import shutil
import json

class ConfigPage(Page):
    def __init__(self, master, name, backend):
        self.points = []  # List of node dicts with keys 'id', 'x', 'y', 'z'
        super().__init__(master, name, backend)

    def create_widgets(self):
        # Header
        self.create_header("Configuration", row=0)

        # Table header
        table_frame = ctk.CTkFrame(self.main_frame, fg_color="transparent")
        table_frame.grid(row=1, column=0, sticky="nsew", padx=20, pady=10)
        for col, text in enumerate(["ID", "X", "Y", "Z"]):
            lbl = ctk.CTkLabel(table_frame, text=text, font=ctk.CTkFont(size=14, weight="bold"))
            lbl.grid(row=0, column=col, padx=5, pady=5)

        # Table rows
        self.row_widgets = {}
        for i, pt in enumerate(self.points):
            row = i + 1
            id_lbl = ctk.CTkLabel(table_frame, text=str(pt["id"]))
            id_lbl.grid(row=row, column=0, padx=5, pady=2)

            x_entry = ctk.CTkEntry(table_frame, width=80)
            x_entry.insert(0, str(pt["x"]))
            x_entry.grid(row=row, column=1, padx=5, pady=2)

            y_entry = ctk.CTkEntry(table_frame, width=80)
            y_entry.insert(0, str(pt["y"]))
            y_entry.grid(row=row, column=2, padx=5, pady=2)

            z_entry = ctk.CTkEntry(table_frame, width=80)
            z_entry.insert(0, str(pt["z"]))
            z_entry.grid(row=row, column=3, padx=5, pady=2)

            self.row_widgets[pt["id"]] = (x_entry, y_entry, z_entry)

        # Control buttons
        btn_frame = ctk.CTkFrame(self.main_frame, fg_color="transparent")
        btn_frame.grid(row=2, column=0, sticky="ew", padx=20, pady=(0, 20))
        btn_frame.grid_columnconfigure((0,1,2), weight=1)

        add_btn = ctk.CTkButton(btn_frame, text="Add Node", command=self.add_node)
        add_btn.grid(row=0, column=0, padx=10, sticky="ew")

        del_btn = ctk.CTkButton(btn_frame, text="Remove Last Node", command=self.remove_node)
        del_btn.grid(row=0, column=1, padx=10, sticky="ew")

        save_btn = ctk.CTkButton(btn_frame, text="Save Config", command=self.save_config)
        save_btn.grid(row=0, column=2, padx=10, sticky="ew")

        # 3D Visualization and Sliders
        viz_frame = ctk.CTkFrame(self.main_frame, fg_color="transparent")
        viz_frame.grid(row=3, column=0, sticky="nsew", padx=20, pady=10)
        viz_frame.grid_columnconfigure(1, weight=1)

        # Matplotlib 3D figure
        fig = Figure(figsize=(4, 4))
        ax = fig.add_subplot(111, projection='3d')
        self.plot_canvas = FigureCanvasTkAgg(fig, master=viz_frame)
        self.plot_canvas.get_tk_widget().grid(row=0, column=0, rowspan=3, sticky="nsew")
        self.fig = fig
        self.ax = ax
        # Connect pick event
        self.plot_canvas.mpl_connect('pick_event', self.on_plot_pick)

        # Sliders for X, Y, Z
        self.x_slider = ctk.CTkSlider(viz_frame, from_=-10, to=10, number_of_steps=200, command=self.on_x_slider_change)
        self.x_slider.grid(row=0, column=1, padx=10, sticky="ew")
        self.y_slider = ctk.CTkSlider(viz_frame, from_=-10, to=10, number_of_steps=200, command=self.on_y_slider_change)
        self.y_slider.grid(row=1, column=1, padx=10, sticky="ew")
        self.z_slider = ctk.CTkSlider(viz_frame, from_=-10, to=10, number_of_steps=200, command=self.on_z_slider_change)
        self.z_slider.grid(row=2, column=1, padx=10, sticky="ew")

        # Initial plot
        self.selected_node_id = self.points[0]["id"] if self.points else None
        self.update_plot3d()

    def add_node(self):
        new_id = len(self.points) + 1
        self.points.append({"id": new_id, "x": 0, "y": 0, "z": 0})
        self.create_widgets()

    def remove_node(self):
        if self.points:
            self.points.pop()
            self.create_widgets()

    def save_config(self):
        # Update point values from entry widgets
        for node_id, (x_e, y_e, z_e) in self.row_widgets.items():
            for pt in self.points:
                if pt["id"] == node_id:
                    pt["x"] = float(x_e.get())
                    pt["y"] = float(y_e.get())
                    pt["z"] = float(z_e.get())
                    break
        # Save each node config as JSON
        folder = "config_output"
        if os.path.exists(folder):
            shutil.rmtree(folder)
        os.makedirs(folder)
        for pt in self.points:
            config = {
                "id": pt["id"],
                "takeoff_height": 4,
                "alt_constraint": {"min_altitude": 6, "height_displacement": pt["z"], "gains": [2.4, 0.1, 0.2]},
                "pos_constraint": {"distance": math.hypot(pt["x"], pt["y"]), "angle": math.degrees(math.atan2(pt["y"], pt["x"])), "gains": [3.5, 0.5, 0]},
                "heading_constraint": {"angle": 0, "gains": [0.2, 0.01, 0]}
            }
            filename = os.path.join(folder, f"drone_{pt['id']}_config.json")
            with open(filename, 'w') as f:
                json.dump(config, f, indent=2)
        messagebox.showinfo("Saved", f"Configuration saved to '{folder}'")
        self.update_plot3d()

    # ---- 3D Visualization Methods ----
    def update_plot3d(self):
        self.ax.clear()
        if not self.points:
            self.plot_canvas.draw()
            return
        xs = [pt['x'] for pt in self.points]
        ys = [pt['y'] for pt in self.points]
        zs = [pt['z'] for pt in self.points]
        self.scatter3d = self.ax.scatter(xs, ys, zs, picker=5, s=50, c='red')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.plot_canvas.draw()

    def on_plot_pick(self, event):
        ind = event.ind[0]
        pt = self.points[ind]
        self.selected_node_id = pt['id']
        # Update sliders to selected point
        self.x_slider.set(pt['x'])
        self.y_slider.set(pt['y'])
        self.z_slider.set(pt['z'])

    def on_x_slider_change(self, value):
        # Update selected node's x value
        if self.selected_node_id is not None:
            for pt in self.points:
                if pt["id"] == self.selected_node_id:
                    pt["x"] = float(value)
                    break
            self.create_widgets()
            self.update_plot3d()

    def on_y_slider_change(self, value):
        if self.selected_node_id is not None:
            for pt in self.points:
                if pt["id"] == self.selected_node_id:
                    pt["y"] = float(value)
                    break
            self.create_widgets()
            self.update_plot3d()

    def on_z_slider_change(self, value):
        if self.selected_node_id is not None:
            for pt in self.points:
                if pt["id"] == self.selected_node_id:
                    pt["z"] = float(value)
                    break
            self.create_widgets()
            self.update_plot3d()