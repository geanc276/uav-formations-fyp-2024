import customtkinter as ctk
import logging
from typing import List, Optional, Callable, Union
from frontend.buttons import SelectableButton, SelectableDroneButton, SelectableMixin

# Configure logging
logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)


class Scrollbox(ctk.CTkFrame):
    """
    A scrollable frame that arranges widgets (typically buttons) in a grid layout.
    """

    def __init__(
        self,
        master: ctk.CTkBaseClass,
        width: int = 600,
        height: int = 200,
        max_columns: int = 3,
        min_button_width: int = 150,
        min_button_height: int = 40,
        padding_x: int = 10,
        padding_y: int = 10,
        **kwargs
    ):
        """
        Initialize the Scrollbox.

        :param master: Parent widget.
        :param width: Width of the Scrollbox frame.
        :param height: Height of the Scrollbox frame.
        :param max_columns: Maximum number of columns in the grid layout.
        :param min_button_width: Minimum width for each button.
        :param min_button_height: Minimum height for each button.
        :param padding_x: Horizontal padding between widgets.
        :param padding_y: Vertical padding between widgets.
        :param kwargs: Additional keyword arguments for CTkFrame.
        """
        super().__init__(master, width=width, height=height, **kwargs)
        self.width = width
        self.height = height
        self.max_columns = max_columns
        self.min_button_width = min_button_width
        self.min_button_height = min_button_height
        self.padding_x = padding_x
        self.padding_y = padding_y

        # Prevent the frame from auto-resizing based on its content.
        self.grid_propagate(False)
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # Create a scrollable frame inside this frame.
        self.scrollable_frame = ctk.CTkScrollableFrame(self, width=width, height=height)
        self.scrollable_frame.grid(row=0, column=0, sticky="nsew")
        self.scrollable_frame.grid_columnconfigure(0, weight=1)

        # List to store widget instances (e.g., buttons)
        self.items: List[ctk.CTkButton] = []

    def add_button(self, button: ctk.CTkButton) -> None:
        """
        Adds a button instance to the scrollable frame.

        :param button: An instance of CTkButton or its subclass.
        """
        logger.debug("Adding button: %s", button)
        self.items.append(button)
        self.arrange_buttons()

    def arrange_buttons(self) -> None:
        """
        Arrange widgets in a grid inside the scrollable_frame.
        """
        if not self.items:
            logger.debug("No items to arrange.")
            return

        logger.debug("Arranging %d items in grid.", len(self.items))
        # Clear existing grid placements
        for widget in self.scrollable_frame.grid_slaves():
            widget.grid_forget()

        # Determine grid dimensions
        columns = min(self.max_columns, len(self.items))
        for col in range(columns):
            self.scrollable_frame.grid_columnconfigure(col, weight=1, uniform="item_column")

        # Place each item in the grid
        for index, item in enumerate(self.items):
            row = index // columns
            col = index % columns
            item.grid(
                row=row,
                column=col,
                padx=self.padding_x,
                pady=self.padding_y,
                sticky="nsew"
            )
            self.scrollable_frame.grid_rowconfigure(row, weight=1)

    def remove_button(self, button: ctk.CTkButton) -> None:
        """
        Removes a button instance from the scrollable frame.

        :param button: An instance of CTkButton or its subclass.
        """
        if button in self.items:
            logger.debug("Removing button: %s", button)
            self.items.remove(button)
            button.destroy()
            self.arrange_buttons()
        else:
            logger.warning("Attempted to remove non-existent button: %s", button)

    def clear(self) -> None:
        """
        Clears all buttons from the scrollable frame.
        """
        logger.debug("Clearing all buttons.")
        # Remove a copy of the list so we can safely modify the original.
        for button in self.items.copy():
            self.remove_button(button)


class MultiSelectScrollbox(Scrollbox):
    """
    A scrollable frame that allows multiple selectable buttons.
    """

    def __init__(
        self,
        master: ctk.CTkBaseClass,
        width: int = 600,
        height: int = 200,
        max_columns: int = 3,
        min_button_width: int = 150,
        min_button_height: int = 40,
        padding_x: int = 10,
        padding_y: int = 10,
        **kwargs
    ):
        """
        Initialize the MultiSelectScrollbox.
        """
        super().__init__(
            master,
            width=width,
            height=height,
            max_columns=max_columns,
            min_button_width=min_button_width,
            min_button_height=min_button_height,
            padding_x=padding_x,
            padding_y=padding_y,
            **kwargs
        )
        self.selected_items: List[SelectableButton] = []

    def add_button(self, button: Union[SelectableButton, ctk.CTkButton]) -> None:
        """
        Adds a selectable or regular button to the scrollable frame.
        """
        if isinstance(button, SelectableMixin):
            # Set the external callback so that selection changes are handled.
            button.set_external_command(self.handle_selection, button)
        super().add_button(button)

    def handle_selection(self, button: SelectableButton) -> None:
        """
        Callback to handle selection changes.
        """
        logger.debug("Handling selection for button: %s", button)
        if button.is_selected():
            if button not in self.selected_items:
                self.selected_items.append(button)
                logger.debug("Button selected: %s", button)
        else:
            if button in self.selected_items:
                self.selected_items.remove(button)
                logger.debug("Button deselected: %s", button)

    def get_selected_items(self) -> List[SelectableButton]:
        """
        Returns the list of currently selected buttons.
        """
        logger.debug("Selected items: %s", self.selected_items)
        return self.selected_items

    def clear_selection(self) -> None:
        """
        Clears selection from all selectable buttons.
        """
        logger.debug("Clearing all selections.")
        for button in self.selected_items.copy():
            button.deselect()
        self.selected_items.clear()

    def remove_button(self, button: Union[SelectableButton, ctk.CTkButton]) -> None:
        """
        Removes a button and updates selection if needed.
        """
        if isinstance(button, SelectableButton) and button in self.selected_items:
            self.selected_items.remove(button)
        super().remove_button(button)

    def select_all(self) -> None:
        """
        Selects all selectable buttons.
        """
        for button in self.items:
            if isinstance(button, SelectableMixin) and not button.is_selected():
                button.select()

    def deselect_all(self) -> None:
        """
        Deselects all selectable buttons.
        """
        logger.debug("Deselecting all buttons.")
        for button in self.items:
            if isinstance(button, SelectableMixin) and button.is_selected():
                button.deselect()
        self.selected_items.clear()


class InfoWidget(ctk.CTkFrame):
    """
    A widget displaying a name (on the left) and up to three info lines (on the right)
    with a horizontal separator.
    """

    def __init__(
        self,
        master,
        name: str = "",
        info_lines: Optional[List[str]] = None,
        width: int = 500,
        height: int = 100,
        font_size: int = 16,
        *args, **kwargs
    ):
        """
        Initializes the InfoWidget.

        :param master: Parent widget.
        :param name: Name to display.
        :param info_lines: List of info lines; if fewer than 3, will be padded; if more, will be truncated.
        :param width: Width of the widget.
        :param height: Height of the widget.
        :param font_size: Font size for the text.
        """
        super().__init__(master, width=width, height=height, *args, **kwargs)
        self.grid_propagate(False)
        self.name = name

        # Process info_lines to have exactly 3 items.
        if info_lines is None:
            info_lines = ["Info line 1: ", "Info line 2: ", "Info line 3: "]
        elif len(info_lines) < 3:
            info_lines += [""] * (3 - len(info_lines))
        elif len(info_lines) > 3:
            info_lines = info_lines[:3]
        self.info_lines = info_lines.copy()

        # Configure grid: left column for name, right column for info.
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=3)
        self.grid_rowconfigure(0, weight=1)

        # Left section: name label.
        self.left_frame = ctk.CTkFrame(self)
        self.left_frame.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        self.left_frame.grid_columnconfigure(0, weight=1)
        self.left_frame.grid_rowconfigure(0, weight=1)
        self.name_label = ctk.CTkLabel(
            self.left_frame,
            text=name,
            font=ctk.CTkFont(size=font_size, weight="bold")
        )
        self.name_label.grid(row=0, column=0, sticky="w")

        # Right section: info lines.
        self.right_frame = ctk.CTkFrame(self)
        self.right_frame.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)
        self.right_frame.grid_columnconfigure(0, weight=1)
        self.info_labels: List[ctk.CTkLabel] = []
        for i, info in enumerate(self.info_lines):
            label = ctk.CTkLabel(
                self.right_frame,
                text=info,
                font=ctk.CTkFont(size=font_size - 2),
                anchor="w",
                justify="left"
            )
            label.grid(row=i, column=0, sticky="w", pady=2)
            self.info_labels.append(label)

        # Separator
        self.separator = ctk.CTkFrame(self, height=2, corner_radius=1, fg_color="gray")
        self.separator.grid(row=1, column=0, columnspan=2, sticky="ew", padx=10, pady=(0, 10))

    def set_name(self, name: str):
        """
        Updates the name displayed.
        """
        self.name = name
        self.name_label.configure(text=name)

    def set_info_line(self, line_number: int, text: str):
        """
        Updates one of the three info lines.
        :param line_number: 1, 2, or 3.
        :param text: New text.
        """
        if 1 <= line_number <= 3:
            self.info_lines[line_number - 1] = text
            self.info_labels[line_number - 1].configure(text=text)
        else:
            raise ValueError("line_number must be 1, 2, or 3")

    def set_all_info(self, info_lines: List[str]):
        """
        Updates all info lines.
        """
        # Adjust to exactly 3 lines.
        if len(info_lines) < 3:
            info_lines += [""] * (3 - len(info_lines))
        elif len(info_lines) > 3:
            info_lines = info_lines[:3]
        self.info_lines = info_lines.copy()
        for i in range(3):
            self.info_labels[i].configure(text=self.info_lines[i])

    def set_font_size(self, font_size: int):
        """
        Updates the font size of the widget.
        """
        self.name_label.configure(font=ctk.CTkFont(size=font_size, weight="bold"))
        for label in self.info_labels:
            label.configure(font=ctk.CTkFont(size=font_size - 2))

    def reload_data(self) -> None:
        """
        Reloads the data for this widget using the stored name and info lines.
        """
        logger.debug("Reloading data for InfoWidget: %s", self.name)
        self.set_name(self.name)
        self.set_all_info(self.info_lines)


class InfoWidgetScrollbox(Scrollbox):
    """
    A scrollable frame that manages multiple InfoWidget instances in a grid layout.
    Typically used in a single-column configuration.
    """

    def __init__(
        self,
        master: ctk.CTkBaseClass,
        width: int = 600,
        height: int = 400,
        max_columns: int = 3,
        padding_x: int = 10,
        padding_y: int = 10,
        **kwargs
    ):
        """
        Initialize the InfoWidgetScrollbox.
        """
        super().__init__(
            master,
            width=width,
            height=height,
            max_columns=max_columns,
            min_button_width=0,
            min_button_height=0,
            padding_x=padding_x,
            padding_y=padding_y,
            **kwargs
        )
        self.info_widgets: List[InfoWidget] = []

    def add_info_widget(self, info_widget: InfoWidget) -> None:
        """
        Adds an InfoWidget instance.
        """
        logger.debug("Adding InfoWidget: %s", info_widget)
        self.info_widgets.append(info_widget)
        self.arrange_info_widgets()

    def remove_info_widget(self, info_widget: InfoWidget) -> None:
        """
        Removes an InfoWidget instance.
        """
        if info_widget in self.info_widgets:
            logger.debug("Removing InfoWidget: %s", info_widget)
            self.info_widgets.remove(info_widget)
            info_widget.destroy()
            self.arrange_info_widgets()
        else:
            logger.warning("Attempted to remove non-existent InfoWidget: %s", info_widget)

    def arrange_info_widgets(self) -> None:
        """
        Arrange InfoWidgets in a single-column grid.
        """
        for widget in self.scrollable_frame.grid_slaves():
            widget.grid_forget()

        for index, info_widget in enumerate(self.info_widgets):
            info_widget.grid(
                row=index,
                column=0,
                padx=self.padding_x,
                pady=self.padding_y,
                sticky="ew"
            )
            self.scrollable_frame.grid_rowconfigure(index, weight=1)
        self.scrollable_frame.grid_columnconfigure(0, weight=1)

    def clear_all_info_widgets(self) -> None:
        """
        Removes all InfoWidget instances.
        """
        logger.debug("Clearing all InfoWidgets.")
        for info_widget in self.info_widgets.copy():
            self.remove_info_widget(info_widget)

    def get_all_info_widgets(self) -> List[InfoWidget]:
        """
        Returns a list of all InfoWidget instances.
        """
        return self.info_widgets

    def update_info_widget(
        self,
        index: int,
        name: Optional[str] = None,
        info_lines: Optional[List[str]] = None
    ) -> None:
        """
        Updates the data for an InfoWidget.
        """
        if 0 <= index < len(self.info_widgets):
            info_widget = self.info_widgets[index]
            if name is not None:
                info_widget.set_name(name)
            if info_lines is not None:
                info_widget.set_all_info(info_lines)
        else:
            logger.warning("Index out of range: %d", index)

    def reload_info_widgets(self, info_data: List[dict]) -> None:
        """
        Clears all existing InfoWidgets and creates new ones from the provided data.
        """
        self.clear_all_info_widgets()
        for data in info_data:
            info_widget = InfoWidget(self.scrollable_frame, **data)
            self.add_info_widget(info_widget)

    def reload_widget(self, name: str) -> None:
        """
        Reloads data for the InfoWidget with the given name.
        """
        for widget in self.info_widgets:
            if widget.name == name:
                widget.reload_data()
                break







class LaunchedDroneWidget(ctk.CTkFrame):
    """
    A custom widget representing a launched drone.
    
    Displays the drone's name, status, and battery info along with two buttons:
      - Stop: stops this drone.
      - SSH: opens an SSH terminal for this drone.
    """
    def __init__(self, master, drone, stop_callback, ssh_callback, **kwargs):
        super().__init__(master, **kwargs)
        self.drone = drone
        self.stop_callback = stop_callback
        self.ssh_callback = ssh_callback

        # Style the widget with rounded corners and a light background.
        self.configure(corner_radius=8, fg_color="#F0F0F0")
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)

        # Drone name header.
        self.name_label = ctk.CTkLabel(
            self,
            text=self.drone.name,
            font=ctk.CTkFont(size=18, weight="bold")
        )
        self.name_label.grid(row=0, column=0, columnspan=2, sticky="w", padx=10, pady=(10, 5))

        # Drone info: status and battery.
        status = self.drone.drone_status.to_dict().get('status', 'Unknown')
        battery = self.drone.drone_status.to_dict().get('battery', 'Unknown')
        self.info_label = ctk.CTkLabel(
            self,
            text=f"Status: {status}    Battery: {battery}",
            font=ctk.CTkFont(size=14)
        )
        self.info_label.grid(row=1, column=0, columnspan=2, sticky="w", padx=10, pady=5)

        # Stop button.
        self.stop_button = ctk.CTkButton(
            self,
            text="Stop",
            fg_color="#dc3545",
            hover_color="#c82333",
            command=self.on_stop
        )
        self.stop_button.grid(row=2, column=0, sticky="ew", padx=10, pady=10)

        # SSH button.
        self.ssh_button = ctk.CTkButton(
            self,
            text="SSH",
            fg_color="#1E90FF",
            hover_color="#1C86EE",
            command=self.on_ssh
        )
        self.ssh_button.grid(row=2, column=1, sticky="ew", padx=10, pady=10)

    def on_stop(self):
        """Callback for when the Stop button is pressed."""
        logger.info("Stopping drone: %s", self.drone.name)
        if callable(self.stop_callback):
            self.stop_callback(self.drone)

    def on_ssh(self):
        """Callback for when the SSH button is pressed."""
        logger.info("Opening SSH for drone: %s", self.drone.name)
        if callable(self.ssh_callback):
            self.ssh_callback(self.drone)

    def refresh(self):
        """Refresh the displayed status and battery information."""
        status = self.drone.drone_status.to_dict().get('status', 'Unknown')
        battery = self.drone.drone_status.to_dict().get('battery', 'Unknown')
        self.info_label.configure(text=f"Status: {status}    Battery: {battery}")

