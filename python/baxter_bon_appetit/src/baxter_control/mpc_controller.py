#!/usr/bin/env python

# Built-in imports
import math
import time
import sys

# General module imports
import numpy as np
import cvxpy

# Own imports
import baxter_control.cartesian_increment as ci
import baxter_essentials.baxter_class as bc
import baxter_essentials.transformation as transf


class MpcController:
    """
    Main Model Predictive Controller for Baxter right limb.
    :param N: Prediction horizon (must be a positive integer).
    """

    def __init__(self, N, has_max_min_constraints, show_results):
        self.N = N  # Prediction horizon
        self.nu = 7  # Number of inputs (thetas)
        self.nx = 7  # Number of states (delta of thetas)
        self.show_results = show_results
        self.has_max_min_constraints = has_max_min_constraints

        self.define_min_max_thetas()

    def define_min_max_thetas(self):
        """
        Defines the vectors for the maximum and minimum values for the thetas 
        angles of Baxter's arms in radians.
        """
        # Values obtained from article "Baxter Humanoid Robot Kinematics"
        self.min_thetas = np.array(
            [
                math.radians(-141),
                math.radians(-123),
                math.radians(-30),
                math.radians(-3),
                math.radians(-175),
                math.radians(-90),
                math.radians(-175)
            ]
        ).reshape((7, ))

        self.max_thetas = np.array(
            [
                math.radians(51),
                math.radians(60),
                math.radians(30),
                math.radians(150),
                math.radians(175),
                math.radians(120),
                math.radians(175)
            ]
        ).reshape((7, ))

    def execute_mpc(self, cartesian_goal, current_thetas):
        """
        Process Model Predictive Controller based on given parameters for one 
        specific iteration.
        :param cartesian_goal: Cartesian goal for the position and orientation 
            of the right arm's end effector.
            Example: np.array([x, y, z, angle_x, angle_y, angle_z]).reshape(6, 1)
        :param current_thetas: Current state of the robot's degrees of freedom.
            Example: np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]).reshape(7, 1)
        :returns: Dictionary with the structure of 
            {"optimal_thetas": np.array, "optimal_dthetas": np.array}. If there 
            are NO optimal answers, it returns "None".
        """
        # Define X and U vectors (thetas and delta-thetas for the MPC)
        if (sys.version_info[0] >= 3):
            thetas = cvxpy.Variable((self.nx, self.N + 1))
            dthetas = cvxpy.Variable((self.nu, self.N))
        else:
            thetas = cvxpy.Variable(self.nx, self.N + 1)
            dthetas = cvxpy.Variable(self.nu, self.N)

        # Initialize cost function and constraints for the MPC problem
        cost = 0.0
        constr = []

        for t in range(self.N):
            # Calculate variables for the construction of the cost function

            cartesian_increment = self.calculate_cartesian_increment(
                cartesian_goal[:, t].reshape(6, 1), current_thetas)
            jacobian = self.calculate_jacobian(current_thetas)

            # Create cost function of:
            # |[dx, dy, dz, dx_angle, dy_angle, dz_angle]' - Jee * DeltaThetas|^2
            print("cartesian_increment.shape:")
            print(cartesian_increment.shape)
            print("dthetas[:,t]:")
            print(dthetas[:, t])
            print("jacobian * dthetas[:, t]).shape:")
            #print((jacobian * dtheas[:, t]).shape)
            inner_error = cartesian_increment - jacobian * dthetas[:, t]
            cost += cvxpy.quad_form(inner_error, np.eye(6))

            # Constaint for: Thetas[k+1] = Thetas[k] + DeltaThetas[k]
            constr += [thetas[:, t + 1] ==
                       thetas[:, t] + dthetas[:, t]]

            if (self.show_results):
                print("cost[{}] = {}".format(t, cost))

            if (self.has_max_min_constraints):
                # Constraint for min values of thetas
                constr += [thetas[:, t] >= self.min_thetas]

                # Constraint for max values of thetas
                constr += [thetas[:, t] <= self.max_thetas]

        # Constraint for initial state: Theta[k] = ThetaCurrent
        constr += [thetas[:, 0] == current_thetas[:, 0]]

        # Define the Convex Optimization Problem with cost function and constraints
        prob = cvxpy.Problem(cvxpy.Minimize(cost), constr)

        # Solve the Conver Optimization Problem and calculate solve-time
        start = time.time()
        prob.solve(verbose=False)
        elapsed_time = time.time() - start

        if (self.show_results):
            print("calc time:{0} [sec]".format(elapsed_time))

        if prob.status == cvxpy.OPTIMAL:
            self.optimal_thetas = thetas.value
            self.optimal_dthetas = dthetas.value
            return {"optimal_thetas": self.optimal_thetas, "optimal_dthetas": self.optimal_dthetas}
        else:
            return None

    def calculate_cartesian_increment(self, cartesian_goal, current_thetas):
        """
        Calculate cartesian increment for the construction of the cost function.
        :param cartesian_goal: Cartesian goal for the position and orientation 
            of the right arm's end effector.
            Example: np.array([x, y, z, angle_x, angle_y, angle_z]).reshape(6, 1)
        :param current_thetas: Current state of the robot's degrees of freedom.
            Example: np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]).reshape(7, 1)
        :returns: Vector of cartesian increment with shape (6, 1).
        """
        # Current tranformation matrix for baxter's right limb
        current_tm = bc.BaxterClass().fpk(current_thetas, "right", 7)

        # Get [x, y, z] from current transformation matrix
        current_position = current_tm[0:3, 3].reshape(3, 1)

        # Get [x_angle, y_angle, z_angle] from current transformation matrix
        t = transf.Transformation(0, 0, 0, [0, 0, 0])  # Fake init (see below)
        fixed_angles_from_tm = t.get_fixed_angles_from_tm(current_tm)
        current_orientation_angles = np.array(
            fixed_angles_from_tm).reshape(3, 1)

        # Concatenate [x, y, z]' and [x_angle, y_angle, z_angle]'
        cartesian_current = np.concatenate(
            [current_position, current_orientation_angles], axis=0)

        # Find cartesian increment for the first part of the error_matrix
        c1 = ci.CartesianIncrements(cartesian_goal, cartesian_current)
        cartesian_increment = c1.calculate_cartesian_increment()

        if (self.show_results):
            print("current_tm:")
            print(current_tm)
            print("current_position:")
            print(current_position)
            print("current_orientation_angles:")
            print(current_orientation_angles)
            print("--------------------")
            print("cartesian_goal:")
            print(cartesian_goal)
            print("cartesian_current:")
            print(cartesian_current)
            print("cartesian_increment:")
            print(cartesian_increment)

        return cartesian_increment.reshape((6, ))

    def calculate_jacobian(self, current_thetas):
        """
        Calculate right arm jacobian based on Baxter Class for the construction 
        of the cost function.
        :param current_thetas: Current state of the robot's degrees of freedom.
            Example: np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]).reshape(7, 1)
        :returns: Matrix of Baxter's jacobian of shape (6, 7)
        """
        b1 = bc.BaxterClass()
        jacobian = b1.jacobian(current_thetas, "right")

        if (self.show_results):
            print("jacobian:")
            print(jacobian)

        return np.matrix(jacobian).reshape(6, 7)


if __name__ == "__main__":
    MPC = MpcController(1, True, True)
    print()

    cartesian_goal = np.array([-0.5, -0.8, 1.0, 0, 0, 0]).reshape(6, 1)
    current_thetas = np.array(
        [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]).reshape(7, 1)
    mpc_results = MPC.execute_mpc(cartesian_goal, current_thetas)

    print('mpc_results["optimal_thetas"]:')
    print(mpc_results["optimal_thetas"])
    print("desired_thetas")
    print('mpc_results["optimal_dthetas"]:')
    print(mpc_results["optimal_dthetas"])

    new_thetas = current_thetas + mpc_results["optimal_dthetas"]
    print("new_thetas:")
    print(new_thetas)

    # CURRENT CARTESIAN POSITION CALCULATIONS...
    print("...fpk[current_thetas]...")
    tm_current = bc.BaxterClass().fpk(current_thetas, "right", 7)
    current_position = tm_current[0:3, 3]
    current_orientation = transf.Transformation(
        0, 0, 0, [0, 0, 0]).get_fixed_angles_from_tm(tm_current)

    # NEW CARTESIAN POSITION CALCULATIONS...
    print("...fpk[new_thetas]...")
    tm_new = bc.BaxterClass().fpk(new_thetas, "right", 7)

    new_position = tm_new[0:3, 3]
    new_orientation = transf.Transformation(
        0, 0, 0, [0, 0, 0]).get_fixed_angles_from_tm(tm_new)

    print("----CURRENT CARTESIAN----")
    print(np.concatenate(
        [current_position, current_orientation], axis=0).reshape(6, 1))
    print("----NEW CARTESIAN----")
    print(np.concatenate(
        [new_position, new_orientation], axis=0).reshape(6, 1))
    print("----CARTESIAN GOAL----")
    print(cartesian_goal)
