import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from random import uniform


def avoid_obs(t, y, z, obs_pt, obs_r, i, margin=0.25):
    # We know point i collides w/ obstacle, thus we move it above or below obstacle
    obs_y, obs_z = obs_pt
    if z[i] < obs_z:
        z_detour = obs_z - obs_r - margin
    else:
        z_detour = obs_z + obs_r + margin

    z[i] = z_detour


def compute_waypoints(t, y, z, obs, n_waypts=100, margin=.75):
    ts, ys, zs = (
        np.linspace(t[0], t[-1], n_waypts),
        np.linspace(y[0], y[-1], n_waypts),
        np.linspace(z[0], z[-1], n_waypts),
    )
    ts, ys, zs = list(ts), list(ys), list(zs)

    obstacles = [
        (np.array([obs.obstacle_1_x, obs.obstacle_1_y]), obs.obstacle_1_radius),
        (np.array([obs.obstacle_2_x, obs.obstacle_2_y]), obs.obstacle_2_radius),
    ]

    # i=0 and i=n_waypts-1 are the fixed start/goal -- never touch them. They're
    # provably clear of both obstacles given the obstacle parameter ranges, and
    # avoid_obs would break there anyway (t[i-1] wraps, t[i+1] is out of range).
    for i in range(1, n_waypts - 1):
        pt = np.array([ys[i], zs[i]])
        for obs_pt, obs_r in obstacles:
            if np.linalg.norm(pt - obs_pt) < obs_r + margin:
                avoid_obs(ts, ys, zs, obs_pt, obs_r, i, margin)

    return ts, ys, zs


def unicycle_spline(t0, tf, obs):
    # UNICYCLE_SPLINE returns a spline object representing a path from
    # (y(t0),z(t0)) = (0,0) to (y(t0),z(t0)) = (10,0) that avoids the two
    # circular obstacles in obs, such that d\dt y(t) > 0
    #   @param t0 - initial time
    #   @param tf - final time
    #
    #   @output y_spline - spline object for desired y trajectory
    #   @output z_spline - spline object for desired z trajectory
    y0 = 0
    z0 = 0

    yf = 10
    zf = 0

    # TODO: design the spline here
    t = np.array([t0, tf])
    y = np.array([y0, yf])
    z = np.array([z0, zf])

    ts, ys, zs = compute_waypoints(t, y, z, obs)

    y_spline = CubicSpline(ts, ys)
    z_spline = CubicSpline(ts, zs)

    return y_spline, z_spline
