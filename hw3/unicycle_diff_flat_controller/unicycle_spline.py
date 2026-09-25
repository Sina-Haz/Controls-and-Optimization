import numpy as np
from scipy.interpolate import CubicSpline


def compute_waypoints(y0, z0, yf, zf, obs, margins):
    # One waypoint per obstacle, placed directly above/below its center at a
    # clearance of radius + margin. Obstacles above the centerline (z > 0) are
    # passed underneath and vice versa, so alternating obstacles force a slalom.
    ys, zs = [y0], [z0]
    for (oy, oz, r), m in zip(obs.obstacles, margins):
        ys.append(oy)
        zs.append(oz - (r + m) if oz > 0 else oz + (r + m))
    ys.append(yf)
    zs.append(zf)
    return np.array(ys), np.array(zs)


def min_clearance(y_spline, z_spline, t0, tf, obs):
    # Clearance (distance to obstacle boundary) of the densely sampled path
    # for each obstacle. Negative means the path cuts through it.
    t = np.linspace(t0, tf, 2000)
    y, z = y_spline(t), z_spline(t)
    return np.array([np.min(np.hypot(y - oy, z - oz)) - r for oy, oz, r in obs.obstacles])


def unicycle_spline(t0, tf, obs, min_margin=0.3):
    # UNICYCLE_SPLINE returns a spline object representing a path from
    # (y(t0),z(t0)) = (0,0) to (y(tf),z(tf)) = (10,0) that weaves around the
    # circular obstacles in obs, such that d\dt y(t) > 0
    #   @param t0 - initial time
    #   @param tf - final time
    #
    #   @output y_spline - spline object for desired y trajectory
    #   @output z_spline - spline object for desired z trajectory
    y0, z0 = 0, 0
    yf, zf = 10, 0

    margins = np.full(len(obs.obstacles), min_margin)
    for _ in range(50):
        ys, zs = compute_waypoints(y0, z0, yf, zf, obs, margins)

        # y is linear in t (constant forward speed), so \dot y > 0 trivially.
        # Waypoint times follow from their y coordinate.
        ts = t0 + (tf - t0) * (ys - y0) / (yf - y0)
        y_spline = CubicSpline(ts, ys)
        # Clamped ends: \dot z = 0 at start and goal, i.e. heading is along +y.
        z_spline = CubicSpline(ts, zs, bc_type="clamped")

        # The spline only touches the clearance circle at the waypoints; between
        # them it can clip an obstacle's edge. Push out whichever ones it clips.
        clearance = min_clearance(y_spline, z_spline, t0, tf, obs)
        if np.all(clearance >= min_margin):
            return y_spline, z_spline
        margins[clearance < min_margin] += 0.05

    raise RuntimeError("Could not find a collision-free spline through the obstacles")
