# frontend/app.py

import threading
import backend
from frontend.menu import Menu, PageButton
from frontend.pages import Page, HomePage, SettingsPage, AboutPage, DronePage, BuildPage, DroneBuildPage, LaunchPage, LaunchConfigPage, UserInputPage,CurrentlyLaunchedPage, ConfigPage
from frontend.scrollbox import Scrollbox
from frontend.command_scrollbox import CommandScrollbox
from backend.commands import JSONCommand, ObjCommand, CommandCommand, RequestCommand, UnknownCommand
from frontend.buttons import DroneButton  # Import DroneButton

import customtkinter as ctk

class App(ctk.CTk):
    def __init__(self, backend, menu_items):
        super().__init__()

        self.title("Drone Management GUI")
        self.geometry("1000x700")
        self.resizable(True, True)

        self.backend = backend
        self.menu_items = menu_items
        self.current_page = None
        # Navigation history stack
        self.navigation_history = []

        # Configure grid layout with two columns: menu and main content
        self.grid_rowconfigure(1, weight=1)
        self.grid_columnconfigure(1, weight=1)

        # Header frame for the menu and back buttons
        self.header_frame = ctk.CTkFrame(self, height=50, fg_color="transparent")
        self.header_frame.grid(row=0, column=0, columnspan=2, sticky="ew")
        self.header_frame.grid_columnconfigure(0, weight=1)
        self.header_frame.grid_columnconfigure(1, weight=0)

        # Create a button to toggle the menu
        self.menu_button = ctk.CTkButton(
            self.header_frame, 
            text="☰", 
            command=self.toggle_menu, 
            width=40, 
            height=40
        )
        self.menu_button.grid(row=0, column=0, padx=10, pady=5, sticky="w")

        # Create a back button, initially hidden
        self.back_button = ctk.CTkButton(
            self.header_frame, 
            text="← Back", 
            command=self.go_back, 
            width=80, 
            height=40
        )
        self.back_button.grid(row=0, column=1, padx=10, pady=5, sticky="e")
        self.back_button.grid_remove()  # Hide initially

        # Create a frame for the menu
        self.menu_frame = Menu(self, self.backend, self.menu_items)
        self.menu_frame.grid(row=1, column=0, sticky="ns")
        self.menu_frame.grid_propagate(False)  # Prevent frame from resizing to fit contents

        # Container for pages
        self.page_container = ctk.CTkFrame(self, corner_radius=0, fg_color="transparent")
        self.page_container.grid(row=1, column=1, sticky="nsew")
        self.page_container.grid_rowconfigure(0, weight=1)
        self.page_container.grid_columnconfigure(0, weight=1)

        # Initialize pages
        self.pages = {}
        self.initialize_pages()

        # # Show the first page by default
        # if menu_items:
        #     initial_page = menu_items[0].page_name
        #     self.show_page(initial_page, add_to_history=False)
        #     print(f"Initial page set to: {initial_page}")  # Debug statement

        self.menu_visible = True

        # Start polling for Swarm changes
        self.previous_drone_names = set()

    def initialize_pages(self):
        """
        Initialize all main pages and register them in the pages dictionary.
        """
        try:
            # Ensure self.pages is initialized
            if not hasattr(self, "pages"):
                self.pages = {}
            print("Pages dictionary initialized.")  # Debug statement

            # Initialize the DronePage first
            self.drone_page = DronePage(self.page_container, "Drone", self.backend)
            self.drone_page.grid(row=0, column=0, sticky="nsew")
            self.pages["Drone"] = self.drone_page
            print("Initialized DronePage.")  # Debug statement

            # Initialize other main pages
            main_pages = {
                "Home": HomePage,
                "Build": BuildPage,
                "DBuild_key": DroneBuildPage,
                "Launch": LaunchPage,
                "Settings": SettingsPage,
                "About": AboutPage,
                "UserInputPage": UserInputPage,
                "CurrentlyLaunched": CurrentlyLaunchedPage,
                "Draw": ConfigPage
            }



            sub_pages = {
                "LaunchConfig": LaunchConfigPage,
            }

            #init home page to display straight away
            home_page = HomePage(self.page_container, "Home", self.backend)
            self.pages["Home"] = home_page
            home_page.grid(row=0, column=0, sticky="nsew")
            print("Initialized Home Page.")  # Debug statement
            initial_page = "Home"
            self.show_page(initial_page, add_to_history=False)

            for page_name, page_class in main_pages.items():
                if page_name == "Home":
                    continue
                try:
                    page = page_class(self.page_container, page_name, self.backend)
                    self.pages[page_name] = page
                    # page.grid(row=0, column=0, sticky="nsew")
                    print(f"Initialized page: {page_name}")  # Debug statement
                except Exception as e:
                    print(f"Failed to initialize page {page_name}: {e}")  # Debug statement


            for page_name, page_class in sub_pages.items():
                try:
                    page = page_class(self.page_container, page_name, self.backend)
                    self.pages[page_name] = page
                    # page.grid(row=0, column=0, sticky="nsew")
                    print(f"Initialized page: {page_name}")  # Debug statement
                except Exception as e:
                    print(f"Failed to initialize page {page_name}: {e}")
            

            self.show_page(initial_page, add_to_history=False)


            # Debugging: Print all initialized pages
            print(f"All pages initialized: {list(self.pages.keys())}")

        except Exception as e:
            print(f"Error during page initialization: {e}")

    def toggle_menu(self):
        if self.menu_visible:
            self.menu_frame.grid_remove()
        else:
            self.menu_frame.grid()
        self.menu_visible = not self.menu_visible


    def make_or_get_drone_page(self, drone):
        """
        Create a new DronePage for the given drone if it doesn't exist. Otherwise, return the existing page.

        :param drone: The drone to create a page for.
        :return: The DronePage instance for the given drone.
        """
        if not hasattr(self, "drone_pages"):
            self.drone_pages = {}

        if not drone.name in self.drone_pages:
            drone_page = DronePage(self.page_container, drone.name, self.backend)
            self.drone_pages[drone.name] = drone_page
            print(f"Created new DronePage for {drone.name}.")
        
        return self.drone_pages[drone.name]

    def change_drone_page(self, drone):
        """
        Change the current drone page to the one associated with the given drone.

        :param drone: The drone to display the page for.
        """
        self.drone_page = self.make_or_get_drone_page(drone)

    def show_page(self, page_name, add_to_history=True):
        """
        Show a specific page. Optionally add the current page to the history.

        :param page_name: The name of the page to show.
        :param add_to_history: Whether to add the current page to the navigation history.
        """
        
        if add_to_history and hasattr(self, 'current_page'):
            self.navigation_history.append(self.current_page)
            self.back_button.grid()  # Show back button
            print(f"Added {self.current_page} to navigation history.")  # Debug statement

        page = self.pages.get(page_name)
        page.grid(row=0, column=0, sticky="nsew")
        if page:
            page.update()
            page.tkraise()
            self.current_page = page_name
            print(f"Showing page: {page_name}")  # Debug statement
        else:
            print(f"Page {page_name} not found in pages.")  # Debug statement

    def update_all_pages(self):
        """
        Update all pages.
        """
        for page in self.pages.values():
            page.update()
        print("Updated all pages.")
        
    def update_current_page(self):
        """
        Update the current page.
        """
        current_page = self.get_current_page()
        if current_page:
            current_page.update()
            print(f"Updated current page: {self.current_page}")

    def get_current_page(self) -> Page:
        """
        Get the name of the current page.

        :return: The name of the current page.
        """
        return self.pages.get(self.current_page)
    
    def removed_drone(self, name):
        if self.current_page == "Drone" and self.drone_page.drone.name == name:
            self.show_page("Home", add_to_history=False)
            print(f"Removed drone {name}. Navigated to Home page.")

    def updated_drone(self, name, **kwargs):
        
        mission = kwargs.get("mission", False)    
        if mission:
                if self.current_page == "CurrentlyLaunched":
                    self.pages["CurrentlyLaunched"].update()
                    print(f"Updated drone {name} on CurrentlyLaunched page.")
        else:
            if self.current_page == "Drone" and self.drone_page.drone.name == name:
                self.drone_page.update()
                print(f"Updated drone {name} on Drone page.")
            if self.current_page == "LaunchConfig":
                self.pages["LaunchConfig"].update()
                print(f"Updated drone {name} on LaunchConfig page.")


    def navigate_to(self, page_name):
        """ 
        Navigate to a child page, adding the current page to the history.

        :param page_name: The name of the child page to navigate to.
        """
        self.show_page(page_name, add_to_history=True)

    def go_back(self):
        """
        Navigate back to the previous page in the history.
        """
        if not self.navigation_history:
            return

        previous_page = self.navigation_history.pop()
        if not self.navigation_history:
            self.back_button.grid_remove()  # Hide back button if no more history
            print("Navigation history is empty. Hiding back button.")  # Debug statement

        page = self.pages.get(previous_page)
        if page:
            page.tkraise()
            self.current_page = previous_page
            print(f"Going back to page: {previous_page}")  # Debug statement

    def reset_navigation(self):
        """
        Reset the navigation history and hide the back button.
        """
        self.navigation_history.clear()
        self.back_button.grid_remove()
        print("Navigation history reset and back button hidden.")  # Debug statement

    def change_colors(self):
        # Example: Change colors dynamically
        self.menu_items[0].color = "purple"
        self.menu_items[1].color = "yellow"
        self.menu_items[2].color = "cyan"
        self.menu_frame.update_menu_colors()
