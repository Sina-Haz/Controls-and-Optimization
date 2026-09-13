import numpy as np
from numpy.linalg import inv
from numpy.linalg import cholesky
from math import sin, cos
from scipy.interpolate import interp1d
from scipy.integrate import ode
from scipy.integrate import solve_ivp

from trajectories import x_d, u_d


class Quadrotor(object):
    """
    Constructor. Compute function S(t) using S(t) = L(t) L(t)^t, by integrating backwards
    from S(tf) = Qf. We will then use S(t) to compute the optimal controller efforts in
    the compute_feedback() function
    """

    def __init__(self, Q, R, Qf, tf):
        self.m = 1
        self.a = 0.25
        self.I = 0.0625
        self.Q = Q
        self.R = R

        """ 
    We are integrating backwards from Qf
    """

        # Get L(tf) L(tf).T = S(tf) by decomposing S(tf) using Cholesky decomposition
        Lf = cholesky(Qf) # returns lower by default, no need to transpose
        # We need to reshape L0 from a square matrix into a row vector to pass into solve_ivp()
        lf = np.reshape(Lf, (36))
        # L must be integrated backwards, solve_ivp handles for us if we pass it inverted tspan from tf-> 0
        tspan = [tf, 0]
        def _ldot(t, l):
          return np.reshape(self.Ldot(t, l.reshape(6, 6)), 36)
        sol = solve_ivp(_ldot, tspan, lf, dense_output=True)
        self.l_spline = sol.sol

    # Linearized Dynamics of _f(x, u) in quad_sim to get xdot \approx A(t)x_e + B(t)x_e
    def A(self, t):
        x, u = x_d(t), u_d(t)
        theta, u1, u2 = x[2], u[0], u[1]

        dq_ddotdq = np.array(
            [
                [0, 0, (-np.cos(theta) / self.m) * (u1 + u2)],
                [0, 0, (-np.sin(theta) / self.m) * (u1 + u2)],
                [0, 0, 0],
            ]
        )
        A = np.block([[np.zeros((3, 3)), np.eye(3)], [dq_ddotdq, np.zeros((3, 3))]])
        return A

    def B(self, t):
        x, u = x_d(t), u_d(t)
        theta = x[2]
        M = np.array(
            [
                [-np.sin(theta) / self.m, -np.sin(theta) / self.m],
                [np.cos(theta) / self.m, np.cos(theta) / self.m],
                [self.a / self.I, -self.a / self.I],
            ]
        )
        B = np.block([[np.zeros((3, 2))], [M]])
        return B

    def Ldot(self, t, L):
        Q = self.Q
        R = self.R
        A = self.A(t)
        B = self.B(t)

        dLdt = np.zeros((6, 6))
        dLdt -= 0.5 * Q @ inv(L).transpose()
        dLdt -= A.transpose() @ L
        dLdt += 0.5 * L @ L.transpose() @ B @ inv(R) @ B.transpose() @ L

        return dLdt


    def compute_feedback(self, t, x):
        # compute current trajectory error x_e:
        x_e = x - x_d(t)
        # Retrieve L(t)
        L = np.reshape(self.l_spline(t), (6, 6))
        u_fb = np.zeros((2,))
        # given L, can compute S and we have linearized control dynamics
        # Thus we compute u* optimal control law given by TV-LQR:
        u_fb = -inv(self.R) @ self.B(t).transpose() @ L @ L.transpose() @ x_e

        # Add u_fb to u_d(t), the feedforward term.  u = u_fb + u_d
        u = u_d(t) + u_fb
        return u

