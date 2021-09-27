#!/usr/bin/env python

# Built-in imports
import sys
import os
import threading

# Own imports
import node_pick_up_food

# General module imports
import rospy
import roslaunch
import baxter_interface
from baxter_interface import CHECK_VERSION

from PIL import ImageTk, Image
if (sys.version_info[0] >= 3):
    import tkinter
    from tkinter import Tk, Canvas, Entry, Text, Button, PhotoImage, Frame
    import tkinter.font as tkFont
else:
    from Tkinter import Tk, Canvas, Entry, Text, Button, PhotoImage, Frame
    import tkFont

from std_msgs.msg import (
    String
)

# Get path for current folder (to relative import assets)
CURRENT_FOLDER = os.path.abspath(os.path.dirname(__file__))
CURRENT_UPPER_FOLDER = os.path.abspath(os.path.join(CURRENT_FOLDER, os.pardir))


class NodeGui(Tk):
    """
    ROS Node that creates a window-based Graphical User Interface for Baxter
    Bon Appetit main functionalities in a friendly way.
    """

    def __init__(self, control_algorithm, launch_instance):
        self.control_algorithm = control_algorithm
        self.launch = launch_instance

        # Initialize parent object (Tk) to use their methods as "self"
        Tk.__init__(self)

        # General window config
        self.geometry("1280x720")
        self.configure(bg="#FFFFFF")
        self.title("THE MOST AMAZING FEEDING ROBOT")
        path_to_icon = os.path.join(
            CURRENT_FOLDER, "gui_assets", "robot_icon.ico")
        try:
            self.iconbitmap(path_to_icon)
        except:
            print("Window icon is not supported for this OS")

        # Create the necessary components with Tkinter functionalities
        self.create_main_components()

        # Execute thread to constantly update self.fsm_state for fsm
        self.active_thread = True
        self.t = threading.Thread(target=self.publish_fsm_state)
        self.t.start()

        # Publisher to update global Baxter FSM (for Baxter-Bon-Appetit)
        self.pub_state = rospy.Publisher(
            'user/fsm',
            String,
            queue_size=1
        )
        self.fsm_state = "stop"

    def publish_fsm_state(self):
        """
        Method to publish the desired state in a ROS topic that enables the 
        overall selection of each one of the other nodes.
        :attribute fsm_state: flag that defines the desired state.
            For example, "go_to_home", "get_food", "mpc", "open_loop", "stop".
        """
        while self.active_thread and not rospy.is_shutdown():
            try:
                self.pub_state.publish(self.fsm_state)
            except:
                pass

    def start(self):
        """
        Start button functionalities.
        """
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

        # Start main nodes from the launch file
        self.launch.start()
        rospy.loginfo("started launch")
        self.fsm_state = "start"
        self.update_state_text_info("ROBOT\nREADY")

    def stop(self):
        """
        Stop button functionalities.
        """
        self.fsm_state = "stop"
        self.update_state_text_info("STOPPED")

    def shutdown(self):
        """
        Shutdown button functionalities.
        """
        self.update_state_text_info("SHUT DOWN")

        try:
            # Stop main nodes from the launch file
            self.launch.shutdown()
            rospy.loginfo("stoped launch")

            # Disable robot
            if self._rs.state().enabled:
                print("Disabling robot...")
                self._rs.disable()
            self.fsm_state = "shutdown"
        except:
            rospy.loginfo("Shutdown error")

        self.quit()

    def go_to_home(self):
        """
        Go to home button functionalities
        """
        self.fsm_state = "go_to_home"
        self.update_state_text_info("GOING TO\n   HOME")

    def give_food(self):
        """
        Give food button functionalities
        """
        self.fsm_state = self.control_algorithm
        self.update_state_text_info("GIVING\n FOOD")

    def pickup_food(self):
        """
        Pick up food button functionalities
        """
        self.fsm_state = "pick_up_food"
        self.update_state_text_info("PICKING UP\n    FOOD")

        filename = os.path.join(CURRENT_FOLDER,"recordings", "pick_up_food.csv")
        loops = 1
        main_node_pick_up_food = node_pick_up_food.NodePickUpFood(
            filename,
            loops
        )
        main_node_pick_up_food.map_file()
        self.fsm_state = "go_to_home"

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
        self.create_state_text()

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
            command=self.stop,
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
            command=self.give_food,
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
            command=self.pickup_food,
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
            command=self.go_to_home,
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
            command=self.shutdown,
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
            command=self.stop,
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
            command=self.start,
            relief="flat"
        )
        self.button_start.place(
            x=47.999999999999886,
            y=164.0,
            width=155.0,
            height=156.0
        )

    def create_state_text(self):
        """
        Method to create the "state" text for the GUI (to show current state).
        """
        font_for_states = tkFont.Font(family='Helvetica',size=15, weight='bold')
        self.state_text = Text(
            self,
            bd=0,
            bg="#FFFFFF",
            fg="#FFFFFF",
            highlightthickness=0,
            background="#AB2A22",
            font=font_for_states
        )
        self.state_text.place(
            x=348.0,
            y=643.0,
            width=143.0,
            height=54.0
        )

    def update_state_text_info(self, message):
        """
        Method to update the state located text message to show the current
        state of the FSM.
        :param message: string of the message (state) to show in text.
        """
        self.state_text.delete('1.0', "end")
        self.state_text.insert("insert", message)


if __name__ == '__main__':
    # Name of the launch file and control algorithm (default or by params)
    try:
        launch_file = sys.argv[1]
        control_algorithm = sys.argv[2]
    except:
        launch_file = "mpc_position_control_nodes.launch"
        control_algorithm = "mpc"

    print("Initializing node... ")
    rospy.init_node('gui_baxter_bon_appetit')

    # Launch main nodes to guarantee default Baxter functionalities
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch_path = os.path.join(CURRENT_UPPER_FOLDER, "launch", launch_file)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])

    gui = NodeGui(control_algorithm, launch)
    gui.resizable(False, False)
    gui.mainloop()
    gui.active_thread = False  # To kill fsm-update thread
    sys.exit()
