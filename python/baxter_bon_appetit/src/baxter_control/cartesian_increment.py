#!/usr/bin/env python

# Built-int imports
import math

# General module imports
import numpy as np


class CartesianIncrements():
    """
    Class to calculate cartesian position and orientation increments for Baxter 
    Model Predictive Controller structure.
    :param cartesian_goal: column vector of the cartesian goal with the user's 
        mouth with the structure: 
        [x_g, y_g, z_g, x_angle_g, y_angle_g, z_angle_g]' .
    :param cartesian_current: column vector of the cartesian goal with the 
        user's mouth with the structure: 
        [x_c, y_c, z_c, x_angle_c, y_angle_c, z_ancle_c]' .
    """

    def __init__(self, cartesian_goal, cartesian_current):
        self.cartesian_goal = cartesian_goal
        self.cartesian_current = cartesian_current

        self.split_cartesian_vectors_in_position_and_orientation()

    def split_cartesian_vectors_in_position_and_orientation(self):
        self.position_current = self.cartesian_current[0:3]
        self.orientation_current = self.cartesian_current[3:6]
        self.position_goal = self.cartesian_goal[0:3]
        self.orientation_goal = self.cartesian_goal[3:6]

    def calculate_position_increment(self):
        position_step = 0.01

        position_abs_err = np.linalg.norm(
            self.position_goal - self.position_current)

        if (position_abs_err > position_step):
            position_increment = position_step * \
                (self.position_goal - self.position_current) / position_abs_err
        else:
            position_increment = self.position_goal - self.position_current

        return position_increment


if __name__ == "__main__":
    cartesian_current = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6]).reshape(6, 1)
    cartesian_goal = np.array([1.1, 1.2, 1.3, 1.4, 1.5, 1.6]).reshape(6, 1)
    c1 = CartesianIncrements(cartesian_goal, cartesian_current)

    print(c1.calculate_position_increment())
