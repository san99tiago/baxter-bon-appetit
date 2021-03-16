#!/usr/bin/env python

# Built-int imports
import math

# General module imports
import numpy as np

# Own imports
import denavit_hartenberg as dh


class BaxterFPK:
    """
    Calculate Baxter's Forward Pose Kinematics for each limb and with the
    desired degrees of freedom for the total joints.

    :param baxter_distances: list of baxter_distances from BaxterClass.
    :param baxter_transformation_matrices: list of
        baxter_transformation_matrices from BaxterClass.
    :param joint_values: list of joint-values to calculate fpk.
        example: [value_limb_s0, value_limb_s1, value_limb_left_e0,
                value_limb_left_e1, value_limb_left_w0, value_limb_left_w1,
                value_limb_left_w2]
    :param limb: arm to calculate fpk.
        example: "left" or "right".
    :param ndof: number of degrees of freedom for fpk.
        example: 6 or 7.
    """

    def __init__(self, baxter_distances, baxter_transformation_matrices,
                 joint_values, limb, ndof):
        self.baxter_distances = baxter_distances
        self.baxter_transformation_matrices = baxter_transformation_matrices
        self.joint_values = joint_values
        self.limb = limb
        self.ndof = ndof
        self.create_dh_table()

    def create_dh_table(self):
        """
        Create Denavit Hartenberg's input table for fpk based on ndof.
        """

        # Choose between the 6dof or 7dof forward pose kinematics solution
        if (self.ndof == 6):
            self.dh_table_0_6 = np.array(
                [[0, 0, 0, self.joint_values[0]],
                 [-math.pi/2, self.baxter_distances[1], 0, self.joint_values[1]],
                 [0, self.baxter_distances[10], 0,
                     (self.joint_values[3]+math.pi/2)],
                 [math.pi/2, 0, self.baxter_distances[4], self.joint_values[4]],
                 [-math.pi/2, self.baxter_distances[5], 0, self.joint_values[5]],
                 [math.pi/2, 0, 0, self.joint_values[6]]]
            )
        elif (self.ndof == 7):
            self.dh_table_0_7 = np.array(
                [[0, 0, 0, self.joint_values[0]],
                 [-math.pi/2, self.baxter_distances[1],
                     0, (self.joint_values[1]+math.pi/2)],
                 [math.pi/2, 0, self.baxter_distances[2], self.joint_values[2]],
                 [-math.pi/2, self.baxter_distances[3], 0, self.joint_values[3]],
                 [math.pi/2, 0, self.baxter_distances[4], self.joint_values[4]],
                 [-math.pi/2, self.baxter_distances[5], 0, self.joint_values[5]],
                 [math.pi/2, 0, 0, self.joint_values[6]]]
            )
        else:
            print("--- ERROR IN DH NDOF INPUT ARGUMENTS ---")

    def fpk(self):
        """
        Main method to calculate the forward pose kinematics.

        :returns: Transformation Matrix from W0
            (origin of the workspace), to Baxter's Tool (end of the arm).
        """

        # Simplified name for clarity in process (to avoid long initial name)
        TM_list = self.baxter_transformation_matrices
        if self.ndof == 6:
            if self.limb == 'left':
                TM_w0_tool = np.dot(
                    np.dot(
                        np.dot(
                            TM_list[0], TM_list[2]
                        ), dh.denavit_hartenberg(self.dh_table_0_6, False)
                    ), TM_list[4]
                )
            if self.limb == 'right':
                TM_w0_tool = np.dot(
                    np.dot(
                        np.dot(
                            TM_list[1], TM_list[3]
                        ), dh.denavit_hartenberg(self.dh_table_0_6, False)
                    ), TM_list[5]
                )
        if self.ndof == 7:
            if self.limb == 'left':
                TM_w0_tool = np.dot(
                    np.dot(
                        np.dot(
                            TM_list[0], TM_list[2]
                        ), dh.denavit_hartenberg(self.dh_table_0_7, False)
                    ), TM_list[4]
                )
            if self.limb == 'right':
                TM_w0_tool = np.dot(
                    np.dot(
                        np.dot(
                            TM_list[1], TM_list[3]
                        ), dh.denavit_hartenberg(self.dh_table_0_7, False)
                    ), TM_list[5]
                )

        return TM_w0_tool


if __name__ == '__main__':
    pass
