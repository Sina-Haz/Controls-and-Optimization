import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt


def unicycle_input(t, y_spline, z_spline):
    # UNICYCLE_INPUT returns input to the unicycle
    #   @param t - current time
    #   @param y_spline - spline object for desired y trajectory
    #   @param z_spline - spline object for desired z trajectory
    #
    #   @output u - input u(t) to the unicycle system
    ydot, zdot, yddot, zddot = (
        y_spline(t, 1),
        z_spline(t, 1),
        y_spline(t, 2),
        z_spline(t, 2),
    )
    theta = np.arctan(zdot / ydot)
    u2 = np.sqrt(ydot**2 + zdot**2)
    u1 = (zddot * ydot - zdot * yddot) / (u2**2)
    u = np.array([u1, u2])
    return u
