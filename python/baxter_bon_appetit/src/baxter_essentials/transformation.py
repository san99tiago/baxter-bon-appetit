#!/usr/bin/env python

# Built-int imports
import math

# General module imports
import numpy as np


class Transformation:
    """
    Get transformation matrix from fixed-angles rotations and a translation.

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
        self.create_transformation_matriz()

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
    
    def create_transformation_matriz(self):
        # Get rotation matrix with fixed angles approach (Rxyz = Rz.Ry.Rx)
        R_ZYX = np.dot(np.dot(self.RZ, self.RY), self.RX)

        extra_zeros_perspective = np.array([[0, 0, 0]])
        self.TM = np.concatenate((R_ZYX, extra_zeros_perspective))
        self.TM = np.concatenate((self.TM, self.vector_AB), axis=1)


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

    test = Transformation(0, 0, math.radians(90), [1, 2, 3])
    print("RX:")
    print(test.RX)
    print("RY:")
    print(test.RY)
    print("RZ:")
    print(test.RZ)
    print("TM:")
    print(test.TM)
