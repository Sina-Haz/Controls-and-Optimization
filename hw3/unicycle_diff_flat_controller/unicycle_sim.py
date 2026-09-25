from math import *
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import importlib
from random import uniform

# Reload the module to use the latest code
import unicycle_spline
import unicycle_input
import plot_unicycle_trajectory

importlib.reload(unicycle_spline)
importlib.reload(unicycle_input)
importlib.reload(plot_unicycle_trajectory)
from unicycle_spline import unicycle_spline
from unicycle_input import unicycle_input
from plot_unicycle_trajectory import plot_unicycle_trajectory


# Two randomly placed obstacles.  ``*_x`` is the horizontal (flat-output y)
# coordinate and ``*_y`` is the vertical (flat-output z) coordinate.
class Obstacles:
    def __init__(self):
        self.obstacle_1_x = uniform(3, 4)
        self.obstacle_1_y = uniform(2, 4)
        self.obstacle_1_radius = uniform(2.5, 3)
        self.obstacle_2_x = 8
        self.obstacle_2_y = uniform(-3, -1)
        self.obstacle_2_radius = uniform(1.5, np.nextafter(2, 1.5))


def simulate_unicycle():
    # SIMULATE_UNICYCLE simulates a trajectory for the unicycle system
    t0 = 0.0
    tf = 10.0

    # Create the obstacles
    obs = Obstacles()

    # get unicycle desired path
    y_spline, z_spline = unicycle_spline(t0, tf, obs)
    # verify that \dot y > 0 for the given spline.
    t_chk = np.linspace(t0, tf, 1000)
    ydot_min = np.min(y_spline(t_chk, 1))
    if ydot_min <= 0:
        raise ValueError("ERROR: \\dot y is not positive over the entire trajectory!")

    # Autonomous ODE
    def f(t, x):
        u = unicycle_input(t, y_spline, z_spline)
        xdot = np.array([np.cos(x[2]) * u[1], np.sin(x[2]) * u[1], u[0]])
        return xdot

    # Initial position
    x0 = np.array([0, 0, np.arctan2(z_spline(t0, 1), y_spline(t0, 1))])

    # Integrate
    sol = solve_ivp(f, (0, tf), x0, t_eval=np.linspace(t0, tf, 100), max_step=5e-3)

    return plot_unicycle_trajectory(sol.t, sol.y.T, y_spline, z_spline, obs)


if __name__ == "__main__":
    simulate_unicycle()
