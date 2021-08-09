#!/usr/bin/env python

# Built-in imports
import math

# General module imports
import numpy as np


class Transformation:
    """
    Transformation matrix operations.

    :param angle_x: rotation of {B} with respect of {A} in "x" axis (radians).
    :param angle_y: rotation of {B} with respect of {A} in "y" axis (radians).
    :param angle_z: rotation of {B} with respect of {A} in "z" axis (radians).
    :param vector_AB: vector from origin of {A} to origin of {B}.
    :returns: transformation matrix size(4, 4)
    """

    def __init__(self, angle_x, angle_y, angle_z, vector_AB):
        # Add extra "1" to vector_AB
        vector_AB.append(1)
        self.vector_AB = np.transpose([vector_AB])

        # Initialize rotation matrices in zero
        self.RX = np.eye(3)
        self.RY = np.eye(3)
        self.RZ = np.eye(3)

        # Change the rotation matrices based on inputs
        self.create_rotation_matrix("x", angle_x)
        self.create_rotation_matrix("y", angle_y)
        self.create_rotation_matrix("z", angle_z)

        # Create homogeneous transformation matrix
        self.create_transformation_matrix()

    def create_rotation_matrix(self, axis, angle):
        """
        Create Roation Matrix from specific angle and axis.
        :param axis: specific axis for rotation.
            Example: "x", "y" or "z".
        :param angle: angle to rotate in specific axis.
            Example: 1.15 or 0.85.
        """

        if axis == "x":
            self.RX = np.array([
                [1, 0, 0],
                [0, math.cos(angle), -math.sin(angle)],
                [0, math.sin(angle), math.cos(angle)]
            ])

        if axis == "y":
            self.RY = np.array([
                [math.cos(angle), 0, math.sin(angle)],
                [0, 1, 0],
                [-math.sin(angle), 0, math.cos(angle)]
            ])

        if axis == "z":
            self.RZ = np.array([
                [math.cos(angle), -math.sin(angle), 0],
                [math.sin(angle), math.cos(angle), 0],
                [0, 0, 1]
            ])

    def create_transformation_matrix(self):
        """
            Get transformation matrix from fixed-angles rotations and a 
            translation.
        """
        # Get rotation matrix with fixed angles approach (Rxyz = Rz.Ry.Rx)
        R_ZYX = np.dot(np.dot(self.RZ, self.RY), self.RX)

        extra_zeros_perspective = np.array([[0, 0, 0]])
        self.TM = np.concatenate((R_ZYX, extra_zeros_perspective))
        self.TM = np.concatenate((self.TM, self.vector_AB), axis=1)

    def get_fixed_angles_from_tm(self, tm):
        """
        Get fixed angles from transformation matrix.
            Remark: these equations are from Craig Robotics book
        :param tm: Tranformation Matrix.
        :returns: array with angles in radians with the following structure.
            np.array([angle_x, angle_y, angle_z]), where "x_angle", "y_angle", 
            and "z_angle" are the specific rotations given the axis.
        """

        y_angle = math.atan2(-tm[2, 0], math.sqrt(tm[0, 0]**2 + tm[1, 0]**2))

        # Compute boundaries cases if necessary (-pi/2 and pi/2)
        if (y_angle == math.pi/2):
            z_angle = 0
            x_angle = math.atan2(tm[0, 1], tm[1, 1])
        elif (y_angle == -math.pi/2):
            z_angle = 0
            x_angle = -math.atan2(tm[0, 1], tm[1, 1])
        else:
            z_angle = math.atan2(
                tm[1, 0]/math.cos(y_angle), tm[0, 0]/math.cos(y_angle))
            x_angle = math.atan2(
                tm[2, 1]/math.cos(y_angle), tm[2, 2]/math.cos(y_angle))

        return([x_angle, y_angle, z_angle])


# TESTS
if __name__ == "__main__":
    test = Transformation(0, 0, math.radians(30), [10, 5, 0])
    print("RX:")
    print(test.RX)
    print("RY:")
    print(test.RY)
    print("RZ:")
    print(test.RZ)
    print("TM:")
    print(test.TM)
    test.get_fixed_angles_from_tm(test.TM)

    test = Transformation(0, 0, math.radians(90), [1, 2, 3])
    print("RX:")
    print(test.RX)
    print("RY:")
    print(test.RY)
    print("RZ:")
    print(test.RZ)
    print("TM:")
    print(test.TM)
    print(test.get_fixed_angles_from_tm(test.TM))

    test = Transformation(math.radians(30), math.radians(
        55), math.radians(90), [1, 2, 3])
    print("RX:")
    print(test.RX)
    print("RY:")
    print(test.RY)
    print("RZ:")
    print(test.RZ)
    print("TM:")
    print(test.TM)
    print(test.get_fixed_angles_from_tm(test.TM))

    TM_right_limb_home = np.array(
        [
            [0.01068722, 0.89724605, -0.44140153, -0.62697903],
            [-0.05035759, -0.44038368, -0.8963963, -0.71040403],
            [-0.99867407, 0.0318079, 0.04047667, 1.35814499],
            [0, 0, 0, 1]
        ]
    )
    print(test.get_fixed_angles_from_tm(TM_right_limb_home))
