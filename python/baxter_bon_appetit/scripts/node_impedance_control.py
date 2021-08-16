#!/usr/bin/env python

# Built-in imports
import sys

# Own imports
from baxter_bon_appetit.cfg import (
    JointSpringsBonAppetitConfig,
)

# General module imports
import rospy
import baxter_interface

from dynamic_reconfigure.server import (
    Server
)

from std_msgs.msg import (
    Empty
)

from baxter_core_msgs.msg import (
    JointCommand
)

from std_msgs.msg import (
    String
)


class NodeImpedanceControl:
    """
    ROS Node that enables Baxter Joint Controllers to execute the commands to 
    move Baxter's right limb to a desired position.

    Based on Virtual Joint Springs class for torque example.
    NodeImpedanceControl class contains methods for the joint torque example,
    allowing to move the right limb to a desired location, entering torque
    mode, and attaching virtual springs from software.

    :param sample_time: integer that defines the sample_time in seconds.
    :param reconfig_server: dynamic reconfigure server
    """

    def __init__(self, reconfig_server):
        self._dyn = reconfig_server

        # Control parameters
        self._rate = 1000.0  # Hz
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout

        # Create our limb instance
        limb = "right"
        self._limb = baxter_interface.Limb(limb)

        # Initialize parameters
        self._springs = dict()
        self._damping = dict()
        self.setpoint_angles = dict()

        # Create cuff disable publisher
        cuff_ns = 'robot/limb/' + limb + '/suppress_cuff_interaction'
        self._pub_cuff_disable = rospy.Publisher(
            cuff_ns,
            Empty,
            queue_size=1
        )

        # Initialize command values for first iteration
        command_names = [
            "right_s0",
            "right_s1",
            "right_e0",
            "right_e1",
            "right_w0",
            "right_w1",
            "right_w2"
        ]
        command_values = [0, 0, 0, 0, 0, 0, 0]

        self.setpoint_angles = dict(
            zip(command_names, command_values)
        )

        _fsm_sub = rospy.Subscriber(
            'user/fsm',
            String,
            self.update_fsm_callback,
            queue_size=1
        )
        self.state = "stop"

        _joint_control_values_sub = rospy.Subscriber(
            'user/joint_control_values',
            JointCommand,
            self.joint_control_values_callback,
            queue_size=1
        )

    def update_fsm_callback(self, std_string):
        """
        Recieve the callback function from the current node that publishes the 
        fsm as a "String" std_msgs. This enables the node to keep updating the 
        Finite State Machine values for executing the "mpc_control".
        :param geometry_pose: current fsm message with a standard 
            "String" format from "std_msgs.msg". 
        """
        self.state = std_string.data
        print(self.state)

    def joint_control_values_callback(self, joint_command):
        """
        Callback to get current control_joint_values for Baxter robot.
        """
        command_names = joint_command.names
        command_values = joint_command.command

        self.setpoint_angles = dict(
            zip(command_names, command_values)
        )

    def _update_parameters(self):
        for joint in self._limb.joint_names():
            self._springs[joint] = self._dyn.config[joint[-2:] +
                                                    '_spring_stiffness']
            self._damping[joint] = self._dyn.config[joint[-2:] +
                                                    '_damping_coefficient']

    def _update_forces(self):
        """
        Calculates the current angular difference between the start position
        and the current joint positions applying the joint torque spring forces
        as defined on the dynamic reconfigure server.
        """
        # get latest spring constants
        self._update_parameters()

        # disable cuff interaction
        self._pub_cuff_disable.publish()

        # create our command dict
        cmd = dict()
        # record current angles/velocities
        cur_pos = self._limb.joint_angles()
        cur_vel = self._limb.joint_velocities()
        # calculate current forces
        for joint in self.setpoint_angles.keys():
            # spring portion
            cmd[joint] = self._springs[joint] * (self.setpoint_angles[joint] -
                                                 cur_pos[joint])
            # damping portion
            cmd[joint] -= self._damping[joint] * cur_vel[joint]
        # command new joint torques
        self._limb.set_joint_torques(cmd)

    def move_to_neutral(self):
        """
        Moves the limb to neutral location.
        """
        self._limb.move_to_neutral()

    def execute_control(self):
        """
        Switches to joint torque mode and attached joint springs to current
        joint positions.
        """

        # set control rate
        control_rate = rospy.Rate(self._rate)

        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and disable
        self._limb.set_command_timeout((1.0 / self._rate) * self._missed_cmds)

        # loop at specified rate commanding new joint torques
        while not rospy.is_shutdown():
            # Only execute control when setpoint and state are correct
            correct_setpoint_condition = self.setpoint_angles["right_s0"] != 0
            correct_state_condition = self.state == "mpc" or self.state == "open_loop"

            if (correct_setpoint_condition and correct_state_condition):
                self._update_forces()
            control_rate.sleep()

    def clean_shutdown(self):
        """
        Switches out of joint torque mode to exit cleanly
        """
        self._limb.exit_control_mode()



def main():
    """
    Based on RSDK Joint Torque Example: Joint Springs
    Moves the specified limb to a desired location and enters
    torque control mode, attaching virtual springs (Hooke's Law)
    to each joint maintaining the start position.
    You can adjust the spring constant and damping coefficient
    for each joint using dynamic_reconfigure.
    """

    print("Initializing node... ")
    rospy.init_node("impedance_control")

    dynamic_cfg_srv = Server(
        JointSpringsBonAppetitConfig,
        lambda config,
        level: config
    )

    main_node_impedance_control = NodeImpedanceControl(dynamic_cfg_srv)
    rospy.on_shutdown(main_node_impedance_control.clean_shutdown)
    main_node_impedance_control.execute_control()


if __name__ == "__main__":
    sys.exit(main())
