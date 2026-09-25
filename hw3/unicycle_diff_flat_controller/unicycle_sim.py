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


# Three randomly placed circular obstacles, alternating above/below the
# straight line from (0,0) to (10,0). Each one straddles that line, so the
# path has to slalom: under the first, over the second, under the third.
# Each obstacle is (y, z, radius) in flat-output coordinates.
class Obstacles:
    def __init__(self):
        self.obstacles = [
            (uniform(2.0, 2.4), uniform(0.3, 0.6), uniform(0.6, 0.9)),
            (uniform(4.8, 5.2), uniform(-0.6, -0.3), uniform(0.6, 0.9)),
            (uniform(7.6, 8.0), uniform(0.3, 0.6), uniform(0.6, 0.9)),
        ]


def simulate_unicycle(n_frame=10):
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
    sol = solve_ivp(f, (0, tf), x0, t_eval=np.linspace(t0, tf, 200), max_step=5e-3)

    return plot_unicycle_trajectory(sol.t, sol.y.T, y_spline, z_spline, obs, n_frame)


if __name__ == "__main__":
    simulate_unicycle()
