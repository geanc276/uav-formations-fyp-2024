
# frontend/menu.py

import customtkinter as ctk
import logging

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

color_dict = {
    "red": "#FF0000",
    "green": "#00FF00",
    "blue": "#0000FF",
    "yellow": "#FFFF00",
    # ... truncated for brevity
    "darkseagreen": "#8FBC8F"
}

class PageButton:
    def __init__(self, title, page_name, color=None):
        self.title = title
        self.page_name = page_name
        self.color = color if color else "green"

class Menu(ctk.CTkFrame):
    def __init__(self, master, backend, menu_items, width=200):
        super().__init__(master, width=width, corner_radius=0)
        self.backend = backend
        self.menu_items = menu_items
        self.menu_buttons = []
        self.configure(fg_color="#2B2B2B", bg_color="#2B2B2B")
        self.grid_columnconfigure(0, weight=1)
        self.create_menu()

    def create_menu(self):
        for i, item in enumerate(self.menu_items):
            button_color = item.color if self.is_valid_color(item.color) else "green"
            dark_color = self.adjust_color(button_color, factor=0.5)
            button = ctk.CTkButton(
                self,
                text=item.title,
                command=lambda i=item.page_name: self.backend.menu_action(i),
                fg_color=button_color,
                hover_color=dark_color,
                width=150,
                height=40,
                corner_radius=8,
                font=ctk.CTkFont(size=16, weight="bold"),
                text_color="white"
            )
            button.grid(row=i, column=0, pady=8, padx=10, sticky="ew")
            self.menu_buttons.append(button)

    def update_menu_colors(self):
        for button, item in zip(self.menu_buttons, self.menu_items):
            new_color = item.color if self.is_valid_color(item.color) else "green"
            button.configure(
                fg_color=new_color,
                hover_color=self.adjust_color(new_color, factor=0.9)
            )

    @staticmethod
    def is_valid_color(color_name):
        return True

    @staticmethod
    def adjust_color(color, factor):
        if not color.startswith("#") or len(color) != 7:
            hex_color = color_dict.get(color.lower(), "green")
        else:
            hex_color = color
        return Menu.adjust_color_hex(hex_color, factor)

    @staticmethod
    def adjust_color_hex(color, factor=0.9):
        if not color.startswith("#") or len(color) != 7:
            return color
        r = int(color[1:3], 16)
        g = int(color[3:5], 16)
        b = int(color[5:7], 16)

        r = max(0, min(int(r * factor), 255))
        g = max(0, min(int(g * factor), 255))
        b = max(0, min(int(b * factor), 255))
        
        return f"#{r:02x}{g:02x}{b:02x}"
