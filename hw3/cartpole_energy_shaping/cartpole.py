
import numpy as np
from math import sin, cos, pi

from pydrake.systems.controllers import LinearQuadraticRegulator



class Cartpole(object):
  '''

  '''
  def __init__(self, kp=1, kd=5):
    # k_p and k_d gains for swing up controller
    
    # TODO: choose appropriate k_p and k_d gains
    self.kp = kp
    self.kd = kd
    self.g = 9.81

    # mass of the pole and mass of the part
    self.mc = 1
    self.mp = 1
    self.L = 1

    # Computes the lqr gains for the final stabilizing controller
    A = np.array([[0,                  0,                                1, 0],
                  [0,                  0,                                0, 1],
                  [0, self.g * (self.mc / self.mp),                      1, 0],
                  [0, self.g * (self.mc + self.mp) / (self.L * self.mc), 0, 0]])
    B = np.array([0, 0, 1/self.mc, 1/(self.L * self.mc)]).T
    Q = np.eye((4))
    Q[3, 3] = 10
    R = np.array([1])

    self.K, self.s = LinearQuadraticRegulator(A, B, Q, R) 
    self.x_des = np.array([0, pi, 0, 0])


  def compute_efforts(self, t, x):
    q = x[:2]
    qdot = x[-2:]

    e_tilde = 0.5 * qdot[1] ** 2 - self.g * cos(q[1]) - self.g
    # Crude heuristic that tells us approx distance of (theta, dot theta) from (pi, 0) which is our equilibrium
    # Would be better to use x^T S x as a measure of cost-to-go for determining LQR but then you would need to have a rough
    # idea of how to scale Q and R s.t. S is on a reasonable scale s.t we could say cost-to-go < 1 (or you would want a good constant to compare)
    angular_distance  = x[3]**2 + (x[1]-pi)**2

    if np.abs(e_tilde) < 1 and angular_distance < 1:
      return self.compute_lqr_input(x)
    else:
      return self.compute_energy_shaping_input(t, x)


  def compute_energy_shaping_input(self, t, x):
    '''
    Computes the energy shaping inputs to stabilize the cartpole
    '''
    g = self.g
    q = x[:2]
    qdot = x[-2:]
    e_tilde = 0.5 * qdot[1] ** 2 - g * cos(q[1]) - g
    q1_ddot_des = x[3] * cos(q[1]) * e_tilde
    
    
    
    # We add this pd term to q1 ddot b/c it also makes it s.t. our acceleration moves in the direction of
    # The cart's CoM at q1 = 0. Note that this is fine b/c for V(E), q1 ddot distributes linearly
    # V(E) = (energy shaping term) + (cross term w/ PD), as get closer to q1 = 0 cross term -> 0
    q1_ddot_pd_term = - self.kd * qdot[0] - self.kp * q[0]

    # Add pd terms to q1_ddot_des
    q1_ddot_des = q1_ddot_des + q1_ddot_pd_term

    # Given desired accel q1 ddot we can compute u as function of this to achieve it
    u = q1_ddot_des * (1 + sin(q[1])**2) - g * sin(q[1]) * cos(q[1]) - sin(q[1]) * qdot[1]**2 

    return u

  def compute_lqr_input(self, x):
    '''
    Stabilizes the cartpole at the final location using lqr
    '''
    return -self.K @ (x - self.x_des)


