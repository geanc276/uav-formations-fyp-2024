from .scrollbox import Scrollbox
import logging

logger = logging.getLogger(__name__)

class CommandScrollbox(Scrollbox):
    def __init__(self, master, width=600, height=200,
                 max_columns=3, min_button_width=150, min_button_height=40,
                 padding_x=10, padding_y=10, **kwargs):
        super().__init__(master, width=width, height=height, max_columns=max_columns,
                         min_button_width=min_button_width, min_button_height=min_button_height,
                         padding_x=padding_x, padding_y=padding_y, **kwargs)


    def add_command_button(self, command_button):
        """
        Adds a command button to the scrollbox.
        :param command_button: Instance of CommandButton.
        """
        self.add_button(command_button)
    
    def remove_command_button(self, command_name):
        """
        Removes a command button from the scrollbox.
        :param command_name: Name of the command to remove.
        """
        self.remove_button(command_name)

    def clear_commands(self):
        """
        Remove all command buttons from the scrollbox.
        """
        for button in self.buttons:
            button.destroy()
        self.buttons.clear()
        logger.debug("Cleared all command buttons from the scrollbox.")