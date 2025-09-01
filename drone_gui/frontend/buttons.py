import customtkinter as ctk
from tkinter import Label
from typing import Callable, Optional
  

# =============================================================================
# Base Command and Button Classes
# =============================================================================

class BaseCommand:
    """A minimal command interface used by buttons."""
    def get_display_name(self) -> str:
        return "Command"

    def get_fg_color(self) -> str:
        return "green"

    def get_hover_color(self) -> str:
        return "darkgreen"

    def execute(self) -> None:
        print("Executing command...")


class BaseButton(ctk.CTkButton):
    """
    A base button class that wraps a CTkButton with default appearance and
    an update method.
    """
    DEFAULT_WIDTH: int = 150
    DEFAULT_HEIGHT: int = 40
    DEFAULT_CORNER_RADIUS: int = 8
    DEFAULT_FG_COLOR: str = "blue"
    DEFAULT_HOVER_COLOR: str = "darkblue"
    DEFAULT_COMPOUND: str = "left"

    def __init__(
        self,
        master: ctk.CTkBaseClass,
        text: str,
        command: Optional[Callable] = None,
        fg_color: Optional[str] = None,
        hover_color: Optional[str] = None,
        compound: Optional[str] = None,
        width: int = DEFAULT_WIDTH,
        height: int = DEFAULT_HEIGHT,
        corner_radius: int = DEFAULT_CORNER_RADIUS,
        **kwargs
    ):
        """
        Initialize the BaseButton with default styling.
        """
        super().__init__(
            master=master,
            text=text,
            command=command,
            width=width,
            height=height,
            corner_radius=corner_radius,
            fg_color=fg_color or self.DEFAULT_FG_COLOR,
            hover_color=hover_color or self.DEFAULT_HOVER_COLOR,
            compound=compound or self.DEFAULT_COMPOUND,
            **kwargs
        )

    def update_appearance(
        self,
        text: Optional[str] = None,
        fg_color: Optional[str] = None,
        hover_color: Optional[str] = None,
        **kwargs
    ):
        """
        Update the appearance of the button.
        """
        if text is not None:
            self.configure(text=text)
        if fg_color is not None:
            self.configure(fg_color=fg_color)
        if hover_color is not None:
            self.configure(hover_color=hover_color)
        for key, value in kwargs.items():
            self.configure(**{key: value})
        self.update_idletasks()


# =============================================================================
# Drone Button and Composite Widgets
# =============================================================================

class DroneButton(BaseButton):
    """
    A button that represents a drone. When clicked, it executes a provided action
    with the drone as argument.
    """
    def __init__(
        self,
        master: ctk.CTkBaseClass,
        drone,
        action,
        **kwargs
    ):
        self.drone = drone
        self.action = action
        super().__init__(
            master=master,
            text=drone.name,
            command=self.do_action,
            **kwargs
        )

    def do_action(self, *args, **kwargs) -> None:
        """Execute the provided action with this drone."""
        self.action(self.drone)

    def update_button(self) -> None:
        """Update the button’s appearance based on the drone’s current name."""
        self.update_appearance(text=self.drone.name)


class SplitPane(ctk.CTkFrame):
    """
    A simple container that splits its area into a top and a bottom section
    separated by a horizontal line.
    """
    def __init__(self, master, **kwargs):
        super().__init__(master, **kwargs)
        self.configure(
            corner_radius=10,
            fg_color="#FFFFFF",
            border_width=1,
            border_color="#CCCCCC"
        )
        self.grid_columnconfigure(0, weight=1)
        
        self.top_frame = ctk.CTkFrame(self, fg_color="transparent")
        self.top_frame.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 5))
        self.top_frame.grid_columnconfigure(0, weight=1)
        
        self.separator = ctk.CTkFrame(self, height=1, fg_color="#DDDDDD")
        self.separator.grid(row=1, column=0, sticky="ew", padx=10)
        
        self.bottom_frame = ctk.CTkFrame(self, fg_color="transparent")
        self.bottom_frame.grid(row=2, column=0, sticky="ew", padx=10, pady=(5, 10))
        self.bottom_frame.grid_columnconfigure(0, weight=1)


class InfoBox(ctk.CTkFrame):
    """
    A widget that displays key-value pairs in a simple box with a light background.
    """
    def __init__(self, master, info_dict: dict, **kwargs):
        super().__init__(master=master, **kwargs)
        self.info_dict = info_dict
        self.labels = []
        self.grid_columnconfigure(0, weight=1)
        self.configure(fg_color="#F7F7F7", corner_radius=8)
        self._build_ui()

    def _build_ui(self):
        """Build or rebuild the label UI from info_dict."""
        for lbl in self.labels:
            lbl.destroy()
        self.labels.clear()
        pad_y = 3
        for i, (key, value) in enumerate(self.info_dict.items()):
            lbl = ctk.CTkLabel(
                self,
                text=f"{key}: {value}",
                anchor="w",
                font=ctk.CTkFont(size=14)
            )
            lbl.grid(row=i, column=0, sticky="ew", pady=(pad_y, pad_y), padx=5)
            self.labels.append(lbl)

    def update_info(self, info_dict: dict):
        """Update the info displayed in the InfoBox."""
        self.info_dict = info_dict
        self._build_ui()


class DroneInfoBoxMixin:
    """
    Mixin providing methods for initializing and updating an InfoBox
    that shows a drone's details.
    """
    def init_info_box(self, master):
        info = self.get_drone_info()
        self.info_box = InfoBox(master=master, info_dict=info, corner_radius=5, fg_color="#F0F0F0")
        # Place the info box in row 2 (assuming row 0 is a button and row 1 a separator)
        self.info_box.grid(row=2, column=0, pady=(5, 0), sticky="ew")

    def get_drone_info(self) -> dict:
        return {
            "Name": self.drone.connection.name,
            "IP": self.drone.connection.ip,
            "Hostname": self.drone.connection.hostname,
            "Status": self.drone.connection.properties.get('status', 'Unknown'),
            "Battery": self.drone.connection.properties.get('battery', 'Unknown'),
        }

    def update_info_box(self):
        if hasattr(self, 'info_box'):
            self.info_box.update_info(self.get_drone_info())


class DroneButtonInfoBox(ctk.CTkFrame):
    """
    A composite widget that displays a drone button on the top and an InfoBox below.
    It uses a SplitPane for a clean, consistent look.
    """
    def __init__(self, master, drone, action, **kwargs):
        super().__init__(master=master, **kwargs)
        self.grid_columnconfigure(0, weight=1)
        self.drone = drone
        self.action = action
        
        # Create the split pane container.
        self.pane = SplitPane(self)
        self.pane.grid(sticky="ew", padx=5, pady=5)
        
        # Create the drone button in the top frame.
        self.drone_button = ctk.CTkButton(
            self.pane.top_frame,
            text=self.drone.name,
            command=self.do_action,
            fg_color="#4CAF50",
            hover_color="#45A049",
            text_color="white",
            font=ctk.CTkFont(size=16, weight="bold"),
            corner_radius=8
        )
        self.drone_button.grid(row=0, column=0, sticky="ew", pady=5)
        
        # Create the InfoBox in the bottom frame.
        self.info_box = InfoBox(self.pane.bottom_frame, info_dict=self.get_drone_info())
        self.info_box.grid(row=0, column=0, sticky="ew", pady=5)

    def do_action(self):
        """Invoke the provided action with the drone."""
        self.action(self.drone)

    def get_drone_info(self) -> dict:
        return {
            "Name": self.drone.connection.name,
            "IP": self.drone.connection.ip,
            "Hostname": self.drone.connection.hostname,
            "Status": self.drone.connection.properties.get('status', 'Unknown'),
            "Battery": self.drone.connection.properties.get('battery', 'Unknown'),
        }

    def update_info_box(self):
        self.info_box.update_info(self.get_drone_info())


class CommandButton(BaseButton):
    """
    A button that represents a command. It displays the command's name and uses its
    defined colors.
    """
    def __init__(
        self,
        master: ctk.CTkBaseClass,
        command_instance: BaseCommand,
        **kwargs
    ):
        self.command_instance = command_instance
        super().__init__(
            master=master,
            text=self.command_instance.get_display_name(),
            command=self.do_action,
            fg_color=self.command_instance.get_fg_color(),
            hover_color=self.command_instance.get_hover_color(),
            **kwargs
        )

    def do_action(self, *args, **kwargs) -> None:
        self.command_instance.execute()

    def update_button(self) -> None:
        self.update_appearance(
            text=self.command_instance.get_display_name(),
            fg_color=self.command_instance.get_fg_color(),
            hover_color=self.command_instance.get_hover_color()
        )


# =============================================================================
# Selectable Button Classes
# =============================================================================

class SelectableMixin:
    """
    Mixin to add selection behavior to a button. It defines selected/unselected colors,
    toggles the state, and calls an external callback if provided.
    """
    SELECTED_FG_COLOR: str = "red"
    SELECTED_HOVER_COLOR: str = "darkred"
    UNSELECTED_FG_COLOR: str = "blue"
    UNSELECTED_HOVER_COLOR: str = "darkblue"

    def __init__(self, *args, **kwargs):
        self.selected: bool = False
        # Set default external command and arguments
        self.external_command: Optional[Callable] = kwargs.pop('external_command', None)
        self.external_args = ()
        self.external_kwargs = {}
        super().__init__(*args, **kwargs)
        self.update_appearance(
            fg_color=self.UNSELECTED_FG_COLOR,
            hover_color=self.UNSELECTED_HOVER_COLOR
        )

    def toggle_selection(self) -> None:
        self.selected = not self.selected
        if self.selected:
            self.configure(
                fg_color=self.SELECTED_FG_COLOR,
                hover_color=self.SELECTED_HOVER_COLOR
            )
        else:
            self.configure(
                fg_color=self.UNSELECTED_FG_COLOR,
                hover_color=self.UNSELECTED_HOVER_COLOR
            )
        self.update_idletasks()
        self.run_external_command()

    def set_external_command(self, external_command: Callable, *args, **kwargs) -> None:
        self.external_command = external_command
        if args or kwargs:
            self.external_args = args
            self.external_kwargs = kwargs

    def run_external_command(self) -> None:
        if self.external_command:
            self.external_command(*self.external_args, **self.external_kwargs)

    def is_selected(self) -> bool:
        return self.selected

    def select(self) -> None:
        if not self.selected:
            self.toggle_selection()

    def deselect(self) -> None:
        if self.selected:
            self.toggle_selection()


class SelectableButton(SelectableMixin, BaseButton):
    """
    A button that supports selection. On click, it toggles its selection state and
    then calls its external command if provided.
    """
    def __init__(
        self,
        master: ctk.CTkBaseClass,
        text: str,
        command: Optional[Callable] = None,
        **kwargs
    ):
        self.external_command = command
        super().__init__(
            master=master,
            text=text,
            command=self.on_click,
            **kwargs
        )

    def on_click(self, *args, **kwargs) -> None:
        self.toggle_selection()
        if self.external_command:
            self.external_command()


class SelectableDroneButton(SelectableMixin, DroneButton):
    """
    A DroneButton that supports selection. When clicked, it toggles its selection
    state.
    """
    def __init__(
        self,
        master: ctk.CTkBaseClass,
        drone,
        action = lambda d: None,  # Note: Changed default from print("NONE") to a no-op.
        external_command: Optional[Callable] = None,
        **kwargs
    ):
        super().__init__(
            master=master,
            drone=drone,
            action=action,
            external_command=external_command,
            **kwargs
        )
        # Override the button command with selection handling.
        self.configure(command=self.handle_selection)

    def do_action(self, *args, **kwargs) -> None:
        self.action(self.drone)

    def handle_selection(self) -> None:
        self.toggle_selection()




