import numpy as np
from pydrake.autodiffutils import AutoDiffXd

def cos(theta):
  return AutoDiffXd.cos(theta)
def sin(theta):
  return AutoDiffXd.sin(theta)

def landing_constraint(vars):
  '''
  Impose a constraint such that if the ball is released at final state xf, 
  it will land a distance d from the base of the robot 
  '''
  l = 1
  g = 9.81
  constraint_eval = np.zeros((3,), dtype=AutoDiffXd)
  q = vars[:2]
  qdot = vars[2:4]
  t_land = vars[-1]
  pos = np.array([
      -l*sin(q[0]) - l*sin(q[0] + q[1]),
      -l*cos(q[0]) - l*cos(q[0] + q[1])
    ])
  vel = np.array([
      -l*qdot[1]*cos(q[0] + q[1]) + qdot[0]*(-l*cos(q[0]) - l*cos(q[0] + q[1])), 
      l*qdot[1]*sin(q[0] + q[1]) + qdot[0]*(l*sin(q[0]) + l*sin(q[0] + q[1]))
    ])

  # TODO: Express the landing constraint as a function of q, qdot, and t_land
  # We constrain z_ball(t_land) (will set to 0)
  constraint_eval[0] = pos[1] + vel[1] * t_land - 0.5 * g * t_land **2
  # We constrain x_ball(t_land) (will set to d)
  constraint_eval[1] = pos[0] + vel[0] * t_land
  # We constraint z_ball(t_f), i.e. final position of end effector before launch (will set to 2)
  constraint_eval[2] = pos[1]

  return constraint_eval

def AddFinalLandingPositionConstraint(prog, xf, d, t_land):
  # First we get the constraint functions from landing_constraint
  vars = np.hstack((xf, t_land))
  lb = [0, d, 1.99]
  ub = [0,np.inf, 2]
  prog.AddConstraint(landing_constraint, lb, ub, vars)
  prog.AddBoundingBoxConstraint(0, np.inf, t_land)


