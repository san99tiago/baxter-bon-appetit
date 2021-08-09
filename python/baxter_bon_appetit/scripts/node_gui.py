#!/usr/bin/env python

# Own imports
import sys
import os

# General module imports
from PIL import ImageTk, Image
if (sys.version_info[0] >= 3):
    from tkinter import Tk, Canvas, Entry, Text, Button, PhotoImage, Frame
else:
    from Tkinter import Tk, Canvas, Entry, Text, Button, PhotoImage, Frame

# Get path for current folder (to relative import assets)
CURRENT_FOLDER = os.path.abspath(os.path.dirname(__file__))


class NodeGui(Tk):
    """
    ROS Node that creates a window-based Graphical User Interface for Baxter
    Bon Appetit main functionalities in a friendly way.
    """

    def __init__(self):
        # Initialize parent object (Tk) to use their methods as "self"
        Tk.__init__(self)

        # General window config
        self.geometry("1280x720")
        self.configure(bg="#FFFFFF")
        self.title("THE MOST AMAZING FEEDING ROBOT")
        
        # Only add window icon if it's compatible
        try:
            path_to_icon = os.path.join(
                CURRENT_FOLDER, "gui_assets", "robot_icon.ico")
            self.iconbitmap(path_to_icon)
        except:
            print("icon not compatible with current os")

        # Create the necessary components with Tkinter functionalities
        self.create_main_components()

    def test(self):
        print("button clicked")

    def create_main_components(self):
        """
        Method to orchestrate the creation of Tkinter components.
        """
        self.create_background()
        self.create_wait_button()
        self.create_give_food_button()
        self.create_pickup_food_button()
        self.create_go_to_home_button()
        self.create_shutdown_button()
        self.create_stop_button()
        self.create_start_button()

    def create_background(self):
        """
        Method to create the main background for the GUI.
        """
        path_to_image_background = os.path.join(
            CURRENT_FOLDER, "gui_assets", "photo_main_background.png")
        self.button_image_background = ImageTk.PhotoImage(
            Image.open(path_to_image_background))
        self.button_background = Button(
            self,
            image=self.button_image_background,
            background="#AB2A22",
            activebackground="#AB2A22",
            borderwidth=0,
            highlightthickness=0,
            command=None,
            relief="sunken"
        )
        self.button_background.place(
            x=1.1368683772161603e-13,
            y=0.0,
            width=1280.0,
            height=724.0
        )

    def create_wait_button(self):
        """
        Method to create the "wait" button for the GUI.
        """
        path_to_image_wait = os.path.join(
            CURRENT_FOLDER, "gui_assets", "photo_wait.png")
        self.button_image_wait = ImageTk.PhotoImage(
            Image.open(path_to_image_wait))
        self.button_wait = Button(
            self,
            image=self.button_image_wait,
            background="#49494A",
            activebackground="#49494A",
            borderwidth=0,
            highlightthickness=0,
            command=self.test,
            relief="flat"
        )
        self.button_wait.place(
            x=1017.9999999999999,
            y=431.0,
            width=165.0,
            height=169.0
        )

    def create_give_food_button(self):
        """
        Method to create the "give_food" button for the GUI.
        """
        path_to_image_give_food = os.path.join(
            CURRENT_FOLDER, "gui_assets", "photo_give_food.png")
        self.button_image_give_food = ImageTk.PhotoImage(
            Image.open(path_to_image_give_food))
        self.button_give_food = Button(
            self,
            image=self.button_image_give_food,
            background="#49494A",
            activebackground="#49494A",
            borderwidth=0,
            highlightthickness=0,
            command=self.test,
            relief="flat"
        )
        self.button_give_food.place(
            x=707.9999999999999,
            y=430.0,
            width=162.0,
            height=170.0
        )

    def create_pickup_food_button(self):
        """
        Method to create the "pickup_food" button for the GUI.
        """
        path_to_image_pickup_food = os.path.join(
            CURRENT_FOLDER, "gui_assets", "photo_pickup_food.png")
        self.button_image_pickup_food = ImageTk.PhotoImage(
            Image.open(path_to_image_pickup_food))
        self.button_pickup_food = Button(
            self,
            image=self.button_image_pickup_food,
            background="#49494A",
            activebackground="#49494A",
            borderwidth=0,
            highlightthickness=0,
            command=self.test,
            relief="flat"
        )
        self.button_pickup_food.place(
            x=1017.9999999999999,
            y=169.0,
            width=166.0,
            height=168.0
        )

    def create_go_to_home_button(self):
        """
        Method to create the "go_to_home" button for the GUI.
        """
        path_to_image_go_to_home = os.path.join(
            CURRENT_FOLDER, "gui_assets", "photo_go_to_home.png")
        self.button_image_go_to_home = ImageTk.PhotoImage(
            Image.open(path_to_image_go_to_home))
        self.button_go_to_home = Button(
            self,
            image=self.button_image_go_to_home,
            background="#49494A",
            activebackground="#49494A",
            borderwidth=0,
            highlightthickness=0,
            command=self.test,
            relief="flat"
        )
        self.button_go_to_home.place(
            x=707.9999999999999,
            y=169.0,
            width=162.0,
            height=168.0
        )

    def create_shutdown_button(self):
        """
        Method to create the "shutdown" button for the GUI.
        """
        path_to_image_sutdown = os.path.join(
            CURRENT_FOLDER, "gui_assets", "photo_shutdown.png")
        self.button_image_shutdown = ImageTk.PhotoImage(
            Image.open(path_to_image_sutdown))
        self.button_shutdown = Button(
            self,
            image=self.button_image_shutdown,
            background="#49494A",
            activebackground="#49494A",
            borderwidth=0,
            highlightthickness=0,
            command=self.test,
            relief="flat"
        )
        self.button_shutdown.place(
            x=413.9999999999999,
            y=169.0,
            width=161.0,
            height=151.0
        )

    def create_stop_button(self):
        """
        Method to create the "stop" button for the GUI.
        """
        path_to_image_stop = os.path.join(
            CURRENT_FOLDER, "gui_assets", "photo_stop.png")
        self.button_image_stop = ImageTk.PhotoImage(
            Image.open(path_to_image_stop))
        self.button_stop = Button(
            self,
            image=self.button_image_stop,
            background="#49494A",
            activebackground="#49494A",
            borderwidth=0,
            highlightthickness=0,
            command=self.test,
            relief="flat"
        )
        self.button_stop.place(
            x=226.9999999999999,
            y=165.0,
            width=160.0,
            height=158.0
        )

    def create_start_button(self):
        """
        Method to create the "start" button for the GUI.
        """
        path_to_image_start = os.path.join(
            CURRENT_FOLDER, "gui_assets", "photo_start.png")
        self.button_image_start = ImageTk.PhotoImage(
            Image.open(path_to_image_start))
        self.button_start = Button(
            self,
            image=self.button_image_start,
            background="#49494A",
            activebackground="#49494A",
            borderwidth=0,
            highlightthickness=0,
            command=self.test,
            relief="flat"
        )
        self.button_start.place(
            x=47.999999999999886,
            y=164.0,
            width=155.0,
            height=156.0
        )


if __name__ == '__main__':
    gui = NodeGui()
    gui.resizable(False, False)
    gui.mainloop()
