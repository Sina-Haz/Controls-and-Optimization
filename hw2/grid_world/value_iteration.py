import numpy as np
from math import *
from grid_world import *

import matplotlib.pyplot as plt

def plot_value_function_and_optimal_policy(world, V, u_opt):
  plt.clf()
  v_plot = plt.imshow(V, interpolation='nearest')
  colorbar = plt.colorbar()
  colorbar.set_label("Value function")
  plt.xlabel("Column")
  plt.ylabel("Row")
  arrow_length = 0.25
  for row in range(world.rows):
    for col in range(world.cols):
      if u_opt[row, col] == 0: #N
        plt.arrow(col, row, 0, -arrow_length, head_width=0.1)
      elif u_opt[row, col] == 1: #E
        plt.arrow(col, row, arrow_length, 0, head_width=0.1)
      elif u_opt[row, col] == 2: #S
        plt.arrow(col, row, 0, arrow_length, head_width=0.1)
      elif u_opt[row, col] == 3: #W
        plt.arrow(col, row, -arrow_length, 0, head_width=0.1)
      else:
        raise ValueError("Invalid action")
  plt.savefig('value_function.png', dpi=240)
  plt.show()

def value_iteration(world, threshold, gamma, plotting=True):
  V = np.zeros((world.rows, world.cols))
  u_opt = np.zeros((world.rows, world.cols))

  fig = plt.figure("Gridworld")
  # Iterate until V_err = max(V_new - V_old) < threshold
  err, n_iters = float('inf'), 0
  while err > threshold:
    V_nxt = V.copy()
    # Update / Bootstrap over each possible state
    for s in range(world.rows * world.cols):
      r, c = world.map_state_to_row_col(s)
      # sum takes in generator expression -> sum for each possible next state
      # min takes in generator expression -> take max sum for each possible action
      V_nxt[r, c] = min( 
        sum(p*(cost + gamma * V[world.map_state_to_row_col(s_nxt)]) for p, s_nxt, cost in world.P[s][a])
        for a in [0, 1, 2, 3]
      )
    # Now we can check convergence, update value fn
    err = np.max(np.abs(V - V_nxt))
    V = V_nxt
    n_iters +=1

  print(f'Number of iterations to convergence: {n_iters}')

  # Once we have V^* for all states we can easily compute u_opt
  for s in range(world.rows * world.cols):
    r, c = world.map_state_to_row_col(s)
    acts_to_cst_to_go = np.array([
      sum(p*(cost + gamma * V[world.map_state_to_row_col(s_nxt)]) for p, s_nxt, cost in world.P[s][a])
        for a in [0, 1, 2, 3]
    ])
    u_opt[r, c] = np.argmin(acts_to_cst_to_go)

  if plotting:
    plot_value_function_and_optimal_policy(world, V, u_opt)

  return V, u_opt

if __name__=="__main__":
  world = GridWorld() 
  threshold = 0.0001
  gamma = 0.9
  # value_iteration(world, threshold, gamma, False)
  value_iteration(world, threshold, gamma, True)
