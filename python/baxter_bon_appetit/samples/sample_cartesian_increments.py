#!/usr/bin/env python
# SAMPLE CARTESIAN INCREMENTS FOR GENERAL PURPOSES TESTS

# Own imports
import baxter_control.cartesian_increment as bcci

# General module imports
import numpy as np
import matplotlib.pyplot as plt


class CartesianIncrementsSimulation:
    """
    CartesianIncrementsSimulation allows the user to generate sample iterations 
    for the cartesian_increment source file. The main goal is to have a solid 
    understanding of how the cartesian vectors evolve on time.
    """

    def __init__(self, current_cartesian_vector, FINAL_CARTESIAN_VECTOR):
        self.current_cartesian_vector = current_cartesian_vector
        self.FINAL_CARTESIAN_VECTOR = FINAL_CARTESIAN_VECTOR

    def execute_simulation(self, error_tolerance=0.0001, max_iterations=10000):
        """
        Execute general simulation based on tolerance and max_iterations 
        criteria to test how the cartesian_increment class works. It also 
        stores the ideal x, y, z, x_angle, y_angle, z_angle vectors throughout 
        the process so that they can be plotted after.
        """
        current_tolerance = 100
        self.current_iteration = 0

        self.x_vector = list()
        self.y_vector = list()
        self.z_vector = list()
        self.x_angle_vector = list()
        self.y_angle_vector = list()
        self.z_angle_vector = list()

        while (current_tolerance > error_tolerance and self.current_iteration < max_iterations):
            cartesian_increment = bcci.CartesianIncrements(
                self.FINAL_CARTESIAN_VECTOR, self.current_cartesian_vector).calculate_cartesian_increment()
            self.current_cartesian_vector = self.current_cartesian_vector + cartesian_increment

            self.x_vector.append(self.current_cartesian_vector[0])
            self.y_vector.append(self.current_cartesian_vector[1])
            self.z_vector.append(self.current_cartesian_vector[2])

            self.x_angle_vector.append(self.current_cartesian_vector[3])
            self.y_angle_vector.append(self.current_cartesian_vector[4])
            self.z_angle_vector.append(self.current_cartesian_vector[5])

            current_tolerance = np.linalg.norm(cartesian_increment)
            self.current_iteration = self.current_iteration + 1

    def create_plots(self):
        """
        Create simple x-y-z position and x-y-z orientatio plots based on the 
        simultations done in the "execute_simulation" method.
        It returns two pop-out matplotlib graphs.
        """
        iterations = np.linspace(
            0, self.current_iteration, self.current_iteration)

        # --->FIRST FIGURE (x-y-z position plots)
        # Define a figure for the creation of the plot
        figure_1 = plt.figure(1)
        axes_1 = figure_1.add_axes([0.1, 0.1, 0.8, 0.8])

        # Generate the plot to the figure_1 and its axes.
        axes_1.plot(iterations, self.x_vector, c='b', linewidth=2)
        axes_1.plot(iterations, self.y_vector, c='g', linewidth=2)
        axes_1.plot(iterations, self.z_vector, c='r', linewidth=2)

        # Customize figure_1 with title, "x"-"y" labels
        axes_1.set_title("X-Y-Z IDEAL POSITION VALUES")
        axes_1.set_xlabel("Iterations [k]")
        axes_1.set_ylabel("Meters [m]")

        # Change the background color of the external part
        figure_1.patch.set_facecolor((0.2, 1, 1))

        # --->SECOND FIGURE (x-y-z angle plots)
        # Define a figure for the creation of the plot
        figure_2 = plt.figure(2)
        axes_2 = figure_2.add_axes([0.13, 0.1, 0.8, 0.8])

        # Generate the plot to the figure_1 and its axes.
        axes_2.plot(iterations, self.x_angle_vector, c='b', linewidth=2)
        axes_2.plot(iterations, self.y_angle_vector, c='g', linewidth=2)
        axes_2.plot(iterations, self.z_angle_vector, c='r', linewidth=2)

        # Customize figure_1 with title, "x"-"y" labels
        axes_2.set_title("X-Y-Z IDEAL ORIENTATION VALUES")
        axes_2.set_xlabel("Iterations [k]")
        axes_2.set_ylabel("Radians [rad]")

        # Change the background color of the external part
        figure_2.patch.set_facecolor((0.2, 1, 1))

        # Display the created figures of this script
        plt.show()


if __name__ == "__main__":
    current_cartesian_vector = np.array(
        [0.7, 0.3, 0.1, -0.01, 0.05, -0.04]).reshape(6, 1)
    FINAL_CARTESIAN_VECTOR = np.array(
        [1.0, 1.1, 0.5, 0, 0, 0]).reshape(6, 1)

    error_tolerance = 0.0001
    max_iterations = 10000

    cis = CartesianIncrementsSimulation(
        current_cartesian_vector, FINAL_CARTESIAN_VECTOR)
    cis.execute_simulation(error_tolerance, max_iterations)
    cis.create_plots()
