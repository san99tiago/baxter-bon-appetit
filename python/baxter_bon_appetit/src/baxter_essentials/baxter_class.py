#!/usr/bin/env python

# Built-int imports
import math

# Own imports
import IPK
import FPK
import baxter_fpk
import baxter_ipk

# General module imports
import numpy as np


class BaxterClass():
    """
    Baxter class for defining general-purpose distances and methods for Baxter
    robot.
    """

    def __init__(self):
        self.define_baxter_distances()
        self.define_baxter_transformation_matrices()
        self.baxter_distances = [
            self.l0, self.l1, self.l2, self.l3, self.l4, self.l5, self.l6,
            self.L, self.h, self.H, self.lh
        ]
        self.baxter_transformation_matrices = [
            self.TM_W0_BL, self.TM_W0_BR, self.TM_BL_0, self.TM_BR_0,
            self.TM_7_GL, self.TM_7_GR
        ]

    def define_baxter_distances(self):
        """
        Define Baxter distances for each measurement of the arms and the general
        workspace positions/distances.
        """

        # Baxter's arms lengths
        self.l0 = 0.27035
        self.l1 = 0.06900
        self.l2 = 0.36435
        self.l3 = 0.06900
        self.l4 = 0.37429
        self.l5 = 0.01000
        self.l6 = 0.36830

        # Baxter's workspace lengths
        self.L = 0.27800
        self.h = 0.06400
        self.H = 1.10400

        # Extra distance calculated (to avoid repeating this operation)
        self.lh = math.sqrt(self.l2**2 + self.l3**2)

    def define_baxter_transformation_matrices(self):
        """
        Definie a series of constant Transformation Matrices necessary to
        map the workspace before and after Baxter's Denavit-Hartenberg approach.
        """

        # Transformation Matrices from origin {W0} to {BL} and {BR}
        self.TM_W0_BL = np.array([[math.sqrt(2)/2, math.sqrt(2)/2, 0,  self.L],
                                  [-math.sqrt(2)/2, math.sqrt(2) /
                                   2, 0, -self.h],
                                  [0, 0, 1, self.H],
                                  [0, 0, 0, 1]])

        self.TM_W0_BR = np.array([[-math.sqrt(2)/2, math.sqrt(2)/2, 0, -self.L],
                                  [-math.sqrt(2)/2, -math.sqrt(2) /
                                   2, 0, -self.h],
                                  [0, 0, 1, self.H],
                                  [0, 0, 0, 1]])

        # Transformation Matrices from {BL} and {BR} to coordinates {0}
        self.TM_BL_0 = np.array([[1, 0, 0,  0],
                                 [0, 1, 0,  0],
                                 [0, 0, 1, self.l0],
                                 [0, 0, 0,  1]])
        self.TM_BR_0 = self.TM_BL_0

        # Transformation Matrices from {7} to coordinates of tools {GR} and {GL}
        self.TM_7_GL = np.array([[1, 0, 0,  0],
                                 [0, 1, 0,  0],
                                 [0, 0, 1, self.l6],
                                 [0, 0, 0,  1]])
        self.TM_7_GR = self.TM_7_GL

    def fpk(self, joint_values, limb, ndof):
        """
        Forward Pose Kinematics for Baxter Robot.

        :param joint_values: Values for the joint-angles in a list-format
            example: [value_limb_s0, value_limb_s1, value_limb_left_e0,
                value_limb_left_e1, value_limb_left_w0, value_limb_left_w1,
                value_limb_left_w2]
        :param limb: selected Baxter arm.
            example: "left", "right".
        :returns: Transformation Matrix from W0 (origin of the workspace), to
            Baxter's Tool (end of the arm).
        """

        TM_w0_tool = baxter_fpk.BaxterFPK(
            self.baxter_distances,
            self.baxter_transformation_matrices,
            joint_values,
            limb,
            ndof).fpk()
        return TM_w0_tool

    def ipk(self, TM_w0_tool, limb, elbow_disposition):
        """
        Inverse Pose Kinematics for Baxter Robot.

        :param TM_w0_tool: Transformation Matrix from W0
            (origin of the workspace), to Baxter's Tool (end of the arm).
        :param limb: selected Baxter arm.
            example: "left", "right".
        :param elbow_disposition: Elbow disposition for the mathematical
            ipk solution.
            example: "up", "down".
        :returns: Values for the joint-angles in a list-format
            example: [value_limb_s0, value_limb_s1, value_limb_left_e0,
                value_limb_left_e1, value_limb_left_w0, value_limb_left_w1,
                value_limb_left_w2]
        """

        joint_values = baxter_ipk.BaxterIPK(
            self.baxter_distances,
            self.baxter_transformation_matrices,
            TM_w0_tool,
            limb,
            elbow_disposition).ipk()
        return joint_values


if __name__ == '__main__':
    print(BaxterClass.__doc__)

    b1 = BaxterClass()

    # TM_w0_tool = np.array([[0.733, 0.653, 0.190, 0.843],
    #                     [0.384, -0.628, 0.677, -0.162],
    #                     [0.562, -0.423, -0.711, 0.661],
    #                     [0, 0, 0, 1]])
    # THETAS = b1.ipk(TM_w0_tool, "left", "up")

    # # Convert resulting angles to degrees
    # THETAS = np.rad2deg(THETAS)
    # print(THETAS)

    # ---------------TEST 1 (real Baxter values)--------------
    theta1 = math.radians(0)
    theta2 = math.radians(0)
    theta3 = math.radians(0)
    theta4 = math.radians(0)
    theta5 = math.radians(0)
    theta6 = math.radians(0)
    theta7 = math.radians(0)

    joint_values = [theta1, theta2, theta3, theta4, theta5, theta6, theta7]

    TM_w0_tool = b1.fpk(joint_values, "left", 6)

    joint_values_calculated = b1.ipk(TM_w0_tool, "left", "up")

    print("--------- TEST 1 ----------------")
    print("TM_W0_GL:")
    print(TM_w0_tool)
    print("joint_values_original:")
    print(joint_values)
    print("joint_values_calculated:")
    print(joint_values_calculated)

    # ---------------TEST 2 (real Baxter values)--------------
    theta1 = math.radians(10)
    theta2 = math.radians(20)
    theta3 = math.radians(0)
    theta4 = math.radians(40)
    theta5 = math.radians(50)
    theta6 = math.radians(60)
    theta7 = math.radians(70)

    joint_values = [theta1, theta2, theta3, theta4, theta5, theta6, theta7]

    TM_w0_tool = b1.fpk(joint_values, "left", 6)

    joint_values_calculated = b1.ipk(TM_w0_tool, "left", "up")

    print("--------- TEST 2 ----------------")
    print("TM_W0_GL:")
    print(TM_w0_tool)
    print("joint_values_original:")
    print(joint_values)
    print("joint_values_calculated:")
    print(joint_values_calculated)
