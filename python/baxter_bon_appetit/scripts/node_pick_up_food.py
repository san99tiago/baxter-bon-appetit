#!/usr/bin/env python

# Built-int imports
import sys

# General module imports
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION


class NodePickUpFood:
    """
    ROS Node that enables the movement of Baxter robot to "pick up food",
    based on a recorded "pick_up_food.csv" file with its needed data.
    :param filename: the file to play
    :param loops: number of times to loop (values < 0 mean 'infinite')
    """

    def __init__(self, filename, loops):
        self.filename = filename
        self.loops = loops

    def try_float(self, x):
        try:
            return float(x)
        except ValueError:
            return None

    def clean_line(self, line, names):
        """
        Cleans a single line of recorded joint positions
        @param line: the line described in a list to process
        @param names: joint name keys
        """
        # convert the line of strings to a float or None
        line = [self.try_float(x) for x in line.rstrip().split(',')]
        # zip the values with the joint names
        combined = zip(names[1:], line[1:])
        # take out any tuples that have a none value
        cleaned = [x for x in combined if x[1] is not None]
        # convert it to a dictionary with only valid commands
        command = dict(cleaned)
        left_command = dict((key, command[key]) for key in command.keys()
                            if key[:-2] == 'left_')
        right_command = dict((key, command[key]) for key in command.keys()
                             if key[:-2] == 'right_')
        return (command, left_command, right_command, line)

    def map_file(self):
        """
        Loops through csv file

        Does not loop indefinitely, but only until the file is read
        and processed. Reads each line, split up in columns and
        formats each line into a controller command in the form of
        name/value pairs. Names come from the column headers
        first column is the time stamp
        """
        left = baxter_interface.Limb('left')
        right = baxter_interface.Limb('right')
        rate = rospy.Rate(1000)

        print("Playing back: %s" % (self.filename,))
        with open(self.filename, 'r') as f:
            lines = f.readlines()
        keys = lines[0].rstrip().split(',')

        l = 0
        # If specified, repeat the file playback 'self.loops' number of times
        while self.loops < 1 or l < self.loops:
            i = 0
            l += 1
            print("Moving to start position...")

            _cmd, lcmd_start, rcmd_start, _raw = self.clean_line(
                lines[1], keys)
            left.move_to_joint_positions(lcmd_start)
            right.move_to_joint_positions(rcmd_start)
            start_time = rospy.get_time()
            for values in lines[1:]:
                i += 1
                loopstr = str(self.loops) if self.loops > 0 else "forever"
                sys.stdout.write("\r Record %d of %d, loop %d of %s" %
                                 (i, len(lines) - 1, l, loopstr))
                sys.stdout.flush()

                cmd, lcmd, rcmd, values = self.clean_line(values, keys)
                # command this set of commands until the next frame
                while (rospy.get_time() - start_time) < values[0]:
                    if rospy.is_shutdown():
                        print("\n Aborting - ROS shutdown")
                        return False
                    if len(lcmd):
                        left.set_joint_positions(lcmd)
                    if len(rcmd):
                        right.set_joint_positions(rcmd)
                    rate.sleep()
        return True


def main():
    """
    Based on RSDK Joint Position Example: File Playback (Rethink Robotics)
    Uses Joint Position Control mode to play back a series of
    recorded joint and gripper positions.
    Run the joint_recorder.py example first to create a recording
    file for use with this example. This example uses position
    control to replay the recorded positions in sequence.
    Note: This version of the playback example simply drives the
    joints towards the next position at each time stamp. Because
    it uses Position Control it will not attempt to adjust the
    movement speed to hit set points "on time".

    Related examples:
        joint_recorder.py; joint_trajectory_file_playback.py.
    """

    print("Initializing node... ")
    rospy.init_node("pick_up_food")

    filename = "pick_up_food.csv"
    loops = 1

    main_node_pick_up_food = NodePickUpFood(filename, loops)
    main_node_pick_up_food.map_file()


if __name__ == '__main__':
    main()
