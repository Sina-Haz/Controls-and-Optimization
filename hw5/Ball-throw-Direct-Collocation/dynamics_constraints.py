import numpy as np

import pydrake.math
from pydrake.autodiffutils import AutoDiffXd
from functools import partial



def EvaluateDynamics(planar_arm, context, x, u):
  # Computes the dynamics xdot = f(x,u)

  planar_arm.SetPositionsAndVelocities(context, x)
  n_v = planar_arm.num_velocities()

  M = planar_arm.CalcMassMatrixViaInverseDynamics(context)
  B = planar_arm.MakeActuationMatrix()
  g = planar_arm.CalcGravityGeneralizedForces(context)
  C = planar_arm.CalcBiasTerm(context)

  M_inv = np.zeros((n_v,n_v)) 
  if(x.dtype == AutoDiffXd):
    M_inv = pydrake.math.inv(M)
  else:
    M_inv = np.linalg.inv(M)
  v_dot = M_inv @ (B @ u + g - C)
  return np.hstack((x[-n_v:], v_dot))

def CollocationConstraintEvaluator(planar_arm, context, dt, x_i, u_i, x_ip1, u_ip1):
  n_x = planar_arm.num_positions() + planar_arm.num_velocities()
  h_i = np.zeros(n_x,)
  # Add dynamics constraint s.t. spline midpt deriv = dynamics evaluated at spline midpt w/ midpt of input spline
  # First we need expressions which represent spline midpt val and deriv
  f_i, f_ip1 = EvaluateDynamics(planar_arm, context, x_i, u_i), EvaluateDynamics(planar_arm, context, x_ip1, u_ip1)
  spline_midpt_val = 0.5 * (x_ip1 + x_i) + (dt / 8) * (f_i - f_ip1)
  spline_midpt_deriv = (3 / (2 * dt)) * (x_ip1 - x_i) - 0.25 * ( f_i + f_ip1)
  h_i = spline_midpt_deriv - EvaluateDynamics(planar_arm, context, spline_midpt_val, 0.5 * (u_i + u_ip1))
  return h_i

def AddCollocationConstraints(prog, planar_arm, context, N, x, u, timesteps):
  n_u = planar_arm.num_actuators()
  n_x = planar_arm.num_positions() + planar_arm.num_velocities()

  def CollocationConstraintHelper(vars, i):
    x_i = vars[:n_x]
    u_i = vars[n_x:n_x + n_u]
    x_ip1 = vars[n_x + n_u: 2*n_x + n_u]
    u_ip1 = vars[-n_u:]
    return CollocationConstraintEvaluator(planar_arm, context, timesteps[i+1] - timesteps[i], x_i, u_i, x_ip1, u_ip1) 

  for i in range(N - 1):
    vars = np.hstack((x[i], u[i], x[i+1], u[i+1]))
    helper = partial(CollocationConstraintHelper, i=i)
    prog.AddConstraint(helper, np.zeros(n_x), np.zeros(n_x), vars)



      
    # TODO: Within this loop add the dynamics constraints for segment i (aka collocation constraints)
    #       to prog
    # Hint: use prog.AddConstraint(CollocationConstraintHelper, lb, ub, vars)
    # where vars = hstack(x[i], u[i], ...)