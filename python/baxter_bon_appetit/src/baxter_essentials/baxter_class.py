#!/usr/bin/env python

# Built-int imports
import math

# Own imports
import baxter_essentials.baxter_fpk as baxter_fpk
import baxter_essentials.baxter_ipk as baxter_ipk
import baxter_essentials.baxter_jacobian as baxter_jacobian

# General module imports
import numpy as np


class BaxterClass():
    """
    Baxter class for defining general-purpose distances and methods for Baxter
    Robot that are commonly implemented in other scripts/classes.
    """

    def __init__(self):
        self.define_baxter_distances()
        self.define_baxter_transformation_matrices()
        self.baxter_distances = [
            self.l0, self.l1, self.l2, self.l3, self.l4, self.l5, self.l6_left,
            self.l6_right, self.L, self.h, self.H, self.lh
        ]
        self.baxter_transformation_matrices = [
            self.TM_W0_BL, self.TM_W0_BR, self.TM_BL_0, self.TM_BR_0,
            self.TM_7_GL, self.TM_7_GR, self.TM_left_limb_camera
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
        self.l6_left = 0.36830
        self.l6_right = 0.36830 + 0.0417 # Right arm with tool distance included

        # Baxter's workspace lengths
        self.L = 0.27800
        self.h = 0.06400
        self.H = 1.10400

        # Extra distance calculated (to avoid repeating this operation)
        self.lh = math.sqrt(self.l2**2 + self.l3**2)

    def define_baxter_transformation_matrices(self):
        """
        Define a series of constant Transformation Matrices necessary to
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
                                 [0, 0, 1, self.l6_left],
                                 [0, 0, 0,  1]])
        self.TM_7_GR = np.array([[1, 0, 0,  0],
                                 [0, 1, 0,  0],
                                 [0, 0, 1, self.l6_right],
                                 [0, 0, 0,  1]])

        # Tranformation matrix for left limb position for the camera approach
        self.TM_left_limb_camera = np.array(
            [[0.08541563, -0.01183818, -0.99627508, -0.1862726],
             [-0.01792621, -0.99978581, 0.01034299, -0.8667861],
             [-0.99618413, 0.01697598, -0.08560955, 1.3585218],
             [0, 0, 0, 1]]
        )

        self.TM_right_limb_home = np.array(
            [[ 0.01269254, 0.5253967, -0.85076272, -0.88412728],
            [-0.058711, -0.84897177, -0.52516659, -0.47108472],
            [-0.99819433, 0.05661483, 0.02007095, 1.28936195],
            [ 0, 0, 0, 1]]
        )

        self.joint_values_right_limb_home = [
            0.39653403366837947,
            -1.2843254146570626,
            -0.18983012250081996,
            2.5928110267233206,
            -0.11236409271260656,
            -1.3514370741270496,
            0.11351457830352062
        ]

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

    def jacobian(self, joint_values, limb):
        """
        Calculate Baxter's Jacobian from w0 to tool.
            Remark: the expression used was calculated based on the article 
            "Baxter Humanoid Robot Kinematics" from Ohio University.

        :param joint_values: list of joint-values.
            example: [value_limb_s0, value_limb_s1, value_limb_left_e0,
                    value_limb_left_e1, value_limb_left_w0, value_limb_left_w1,
                    value_limb_left_w2]
        :param limb: arm to calculate jacobian.
            example: "left" or "right".
        """

        return baxter_jacobian.BaxterJacobian(
            self.baxter_distances,
            joint_values,
            limb
        ).calculate_jacobian()


if __name__ == '__main__':
    print(BaxterClass.__doc__)

    b1 = BaxterClass()

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

    # ---------------TEST 3 (real Baxter values)--------------
    theta1 = math.radians(10)
    theta2 = math.radians(20)
    theta3 = math.radians(0)
    theta4 = math.radians(40)
    theta5 = math.radians(50)
    theta6 = math.radians(60)
    theta7 = math.radians(70)

    joint_values = [theta1, theta2, theta3, theta4, theta5, theta6, theta7]

    TM_w0_tool = b1.fpk(joint_values, "right", 6)

    joint_values_calculated = b1.ipk(TM_w0_tool, "right", "up")

    print("--------- TEST 3 ----------------")
    print("TM_W0_GL:")
    print(TM_w0_tool)
    print("joint_values_original:")
    print(joint_values)
    print("joint_values_calculated:")
    print(joint_values_calculated)
    print("Jacobian:")
    print(b1.jacobian(joint_values, "right"))
