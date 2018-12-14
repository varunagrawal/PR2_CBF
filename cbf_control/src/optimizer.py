import numpy as np
import math
import cvxopt
from get_fk import GetFK, get_transformation_matrix


class Optimizer:
    def __init__(self, joint_limits, gamma=100):
        # do setup stuff
        self.joint_limits = joint_limits
        self.gamma = gamma

    def jacobian(self, q):
        """ Calculate manipulator Jacobian.
            Takes numpy array of joint angles, in radians.

            We make a function call to the robot to provide the manipulator Jacobian.
        """
        raise NotImplementedError()

    def fk(self, func, q):
        """
        Forward kinematics
        """
        rospy.loginfo("Querying for FK")
        resp = func(q)
        T = get_transformation_matrix(resp)
        return T

    def diff(a, b):
        # return np.dot(np.linalg.inv(a), b)
        return a - b
        
    def ik(self, q, g, e=1e-9):
        """ Inverse kinematics.
            q: joint angles
            g: goal state

            Optional: e: error norm threshold
        """
        configuration_trajectory = []

        while True:
            err = self.diff(g, self.fk(q))
            J = self.jacobian(q)
            JJ_inv = np.linalg.pinv(np.matmul(J.T, J))

            u = JJ_inv * err

            # optimize u as per the control barrier function
            u = self.optimize(q, u)

            # q = q + np.matmul(np.matmul(JJ_inv, J.T), err)
            q = q + u
            
            # Record the update as part of the trajectory
            configuration_trajectory.append(q)

            if np.linalg.norm(err) <= e:
                break

        # return result in interval [-pi,pi)
        return configuration_trajectory

    def optimize(self, c, x):
        """
        Please refer to `https://cvxopt.org/userguide/coneprog.html#quadratic-programming` for notational convention.

        c: robot configuration
        x: controller
        """
        x = x.astype(np.float)
        c = c.astype(np.float)

        P = 2 * np.eye(x.shape[0], dtype=np.float)
        q = -2 * c

        qm = (self.joint_limits[:, 1] + self.joint_limits[:, 0]) / 2

        d = (self.joint_limits[:, 1] - self.joint_limits[:, 0]).astype(np.float)
        delta = np.square(d) / 4

        h = self.gamma * (delta - np.power((x - qm), 2))

        G = np.diag((2*(x - qm) / np.power((delta - np.power(x-qm, 2)), 2)))

        P = cvxopt.matrix(P, P.shape)

        q = q.reshape(q.shape[0], 1)
        q = cvxopt.matrix(q, q.shape)

        G = cvxopt.matrix(G, G.shape)

        h = h.reshape(h.shape[0], 1)
        h = cvxopt.matrix(h, h.shape)

        # Run the solver
        c_opt = cvxopt.solvers.qp(P, q, G, h)

        return c_opt
