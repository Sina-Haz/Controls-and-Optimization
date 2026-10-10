import numpy as np
import math

def Ab_i1(i, n, d, dt_i, w_i, w_ip1):
  '''
  Ab_i1(i, n, d, dt_i, w_i, w_ip1) computes the linear equality constraint
  constants that require the ith polynomial to meet waypoints w_i and w_{i+1}
  at it's endpoints.
  Parameters:
     i - index of the polynomial.
     n - total number of polynomials.
     d - the number of terms in each polynomial.
     dt_i - Delta t_i, duration of the ith polynomial.
     w_i - waypoint at the start of the ith polynomial.
     w_ip1 - w_{i+1}, waypoint at the end of the ith polynomial.
  Outputs
     A_i1 - A matrix from linear equality constraint A_i1 v = b_i1
     b_i1 - b vector from linear equality constraint A_i1 v = b_i1
  '''
  # v layout: sigma_{i,j} in R^2 stored at v[2*(i*d + j) : 2*(i*d + j) + 2]
  # Rows: [start_y, start_z, end_y, end_z]
  A_i1 = np.zeros((4, 2*d*n))
  b_i1 = np.zeros((4, 1))

  w_i, w_ip1 = np.reshape(w_i, (-1, 1)), np.reshape(w_ip1, (-1, 1)) # make col vectors
  b_i1 = np.vstack((w_i, w_ip1))

  y_start_idx = 2*i*d
  powers = [dt_i**j for j in range(d)]

  A_i1[0, y_start_idx]=1
  A_i1[1, y_start_idx+1]=1
  A_i1[2, y_start_idx: y_start_idx + 2*d : 2] = powers # cols 0, 2, 4, ... (i.e. y vars)
  A_i1[3, y_start_idx+1: y_start_idx + 2*d : 2] = powers # cols 1, 3, 5, ... (z-vars)

  return A_i1, b_i1


def Ab_i2(i, n, d, dt_i):
  '''
  Same as above but now we are coding the equality constraints for
  spline continuity (i.e. we have that the kth derivative of gamma i = kth derivative of gamma i+1 where they meet)
  '''
  # v layout: sigma_{i,j} in R^2 stored at v[2*(i*d + j) : 2*(i*d + j) + 2]
  # Rows: 4 constraints on derivs * 2 vars = 8 total constraints as rows
  A = np.zeros((8, n*d, 2)) # flatten the 2 later
  b = np.zeros((8, 1)) # will stay 0 btw

  base_idx = i*d
  next_idx = (i+1)*d

  for ax in range(2):
    for k in range(1,5):
      row = 4*ax + (k-1) # idxs y rows then z rows
      A[row, base_idx + k : base_idx + d, ax] = [(math.factorial(j) / math.factorial(j-k)) * dt_i **(j-k) for j in range(k, d)]
      A[row, next_idx + k, ax] = -math.factorial(k)
  return A.reshape(8, 2*n*d), b
