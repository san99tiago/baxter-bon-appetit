#!/usr/bin/env python

# Built-int imports
import math
import cmath

# General module imports
import numpy as np

# Own imports
import baxter_essentials.denavit_hartenberg as dh


class BaxterIPK:
    """
    Calculate Baxter's Inverse Pose Kinematics for each limb and with the
    desired degrees of freedom for the total joints.

    :param TM_w0_tool: Transformation Matrix from W0
        (origin of the workspace), to Baxter's Tool (end of the arm).
    :param baxter_distances: list of baxter_distances from BaxterClass.
    :param baxter_transformation_matrices: list of
        baxter_transformation_matrices from BaxterClass.
    :param limb: arm to calculate fpk.
        example: "left" or "right".
    :param elbow_disposition: Elbow disposition for the mathematical
        ipk solution.
        example: "up", "down".
    """

    def __init__(self, baxter_distances, baxter_transformation_matrices,
                 TM_w0_tool, limb, elbow_disposition):
        self.TM_w0_tool = TM_w0_tool
        self.baxter_distances = baxter_distances
        self.baxter_transformation_matrices = baxter_transformation_matrices
        self.calibrate_baxter_transformation_matrices()
        self.limb = limb
        self.elbow_disposition = elbow_disposition

    def calibrate_baxter_transformation_matrices(self):
        X_OFFSET = 0
        Y_OFFSET = 0
        Z_OFFSET = - 0.06012

        calibration_matrix = np.array(
            [[1, 0, 0, X_OFFSET],
             [0, 1, 0, Y_OFFSET],
             [0, 0, 1, Z_OFFSET],
             [0, 0, 0, 1]]
        )
        # Add extra tool distance for Baxter's right limb (spoon)
        self.baxter_transformation_matrices[1] = \
            np.dot(self.baxter_transformation_matrices[1], calibration_matrix)

    def ipk(self):
        """
        Main method to calculate the inverse pose kinematics.

        :returns: list of joint-values to calculate fpk.
            example: [value_limb_s0, value_limb_s1, value_limb_left_e0,
                value_limb_left_e1, value_limb_left_w0, value_limb_left_w1,
                value_limb_left_w2]
        """

        # Simplified name for clarity in process (to avoid long initial name)
        TM_list = self.baxter_transformation_matrices

        # Transformation matrix from 0 to 6 (main Baxter-joints)
        if self.limb == 'left':
            TM_0_6 = np.dot(
                np.dot(
                    np.linalg.inv(
                        np.dot(TM_list[0], TM_list[2])
                    ), self.TM_w0_tool
                ), np.linalg.inv((TM_list[4]))
            )

        if self.limb == 'right':
            TM_0_6 = np.dot(
                np.dot(
                    np.linalg.inv(
                        np.dot(TM_list[1], TM_list[3])
                    ), self.TM_w0_tool
                ), np.linalg.inv((TM_list[5]))
            )

        # Find specific limb articulation "s0" (theta1)
        t1 = math.atan2(TM_0_6[1, 3], TM_0_6[0, 3])

        # Find specific limb articulation "s1" (theta2)
        # Remark: If complex, we must approximate to nearest real-value
        E = 2*self.baxter_distances[11] * \
            (self.baxter_distances[1] - TM_0_6[0, 3]/math.cos(t1))
        F = 2*self.baxter_distances[11]*TM_0_6[2, 3]
        G = (TM_0_6[0, 3]/math.cos(t1))**2 + self.baxter_distances[1]**2 + \
            self.baxter_distances[11]**2 - self.baxter_distances[4]**2 + \
            TM_0_6[2, 3]**2 - \
            2*self.baxter_distances[1]*(TM_0_6[0, 3]/math.cos(t1))

        # "a", "b", "c" correspond to the general-cuadratic coefficients
        a = G-E
        b = 2*F
        c = G+E

        # Find two possible mathematical solutions based on general-cuadratic
        # approach for elbow-up and elbow-down
        tt21 = (-b + cmath.sqrt(b**2-4*a*c))/(2*a)
        tt22 = (-b - cmath.sqrt(b**2-4*a*c))/(2*a)

        print(tt21)

        t21 = 2*math.atan(tt21)
        t22 = 2*math.atan(tt22)

        if abs(t21.imag) < 2:
            t21 = t21.real
        else:
            print("Big imaginary part in t21.")
        if abs(t22.imag) < 2:
            t21 = t22.real
        else:
            print("Big imaginary part in t21.")

        print(t21)

        # Find theta4
        t41 = math.atan2(-TM_0_6[2, 3] -
                         self.baxter_distances[11]*math.sin(t21), TM_0_6[0, 3] /
                         math.cos(t1) - self.baxter_distances[1] -
                         self.baxter_distances[11]*math.cos(t21)) - t21
        t42 = math.atan2(-TM_0_6[2, 3] -
                         self.baxter_distances[11]*math.sin(t22), TM_0_6[0, 3] /
                         math.cos(t1) - self.baxter_distances[1] -
                         self.baxter_distances[11]*math.cos(t22)) - t22

        # Find degrees of freedom related to rotation
        s1 = math.sin(t1)
        c1 = math.cos(t1)
        s24 = [math.sin(t21+t41), math.sin(t22+t42)]
        c24 = [math.cos(t21+t41), math.cos(t22+t42)]

        RM_0_3 = {'up': np.array([[-c1*s24[0], -c1*c24[0], -s1],
                                  [-s1*s24[0], -s1*c24[0], c1],
                                  [-c24[0], s24[0], 0]]),
                  'down': np.array([[-c1*s24[1], -c1*c24[1], -s1],
                                    [-s1*s24[1], -s1*c24[1], c1],
                                    [-c24[1], s24[1], 0]])}
        RM_3_6 = {'up': np.dot(RM_0_3['up'].transpose(), TM_0_6[:3, :3]),
                  'down': np.dot(RM_0_3['down'].transpose(), TM_0_6[:3, :3])}

        if self.elbow_disposition == 'up':
            # Find theta5
            t5 = math.atan2(RM_3_6['up'][2, 2], RM_3_6['up'][0, 2])

            # Find theta 7
            t7 = math.atan2(-RM_3_6['up'][1, 1], RM_3_6['up'][1, 0])

            # Find theta 6
            t6 = math.atan2(RM_3_6['up'][1, 0] /
                            math.cos(t7), -RM_3_6['up'][1, 2])

            # Define the vector for dofs
            join_values = [t1, t21, 0, t41, t5, t6, t7]

        if self.elbow_disposition == 'down':
            # Find theta 5
            t5 = math.atan2(RM_3_6['down'][2, 2], RM_3_6['down'][0, 2])

            # Find theta 7
            t7 = math.atan2(-RM_3_6['down'][1, 1], RM_3_6['down'][1, 0])

            # Find theta 6
            t6 = math.atan2(RM_3_6['down'][1, 0] /
                            math.cos(t7), -RM_3_6['down'][1, 2])

            # Definimos el vector de los dof
            join_values = [t1, t22, 0, t42, t5, t6, t7]

        # Retornamos los valores de los dof
        return join_values


if __name__ == '__main__':
    pass
