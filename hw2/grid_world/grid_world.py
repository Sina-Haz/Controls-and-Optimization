import numpy as np


class GridWorld():


  ''' 
  Gridworld dynamics very loosely borrowed from Sutton and Barto:
  Reinforcement Learning

  The dynamics have been simplified to be deterministic
  '''
  def __init__(self, slip_prob = 0.1):
    # actions legend 
    # 0: N 
    # 1: E 
    # 2: S 
    # 3: W 
    self.rows = 10
    self.cols = 10

    base_cost = 1
    large_cost = 5
    wall_cost = 2

    slow_states = [1 * self.rows + 2, 2 * self.rows + 2, 
                   3 * self.rows + 2, 4 * self.rows + 2,
                   5 * self.rows + 4, 5 * self.rows + 5, 
                   5 * self.rows + 6, 5 * self.rows + 7, 
                   5 * self.rows + 8]
    goal_state = 2 * self.rows + 4

    # helper fn for P, assumes s is valid state
    def cost(s, valid_a):
      if s == goal_state: return 0
      if valid_a == False: return wall_cost
      if s in slow_states: return large_cost
      return base_cost
      
                   
    P = np.zeros((self.rows * self.cols,4), dtype=(list))

    # TODO: Fill out the transition matrix P
    # P is a n x m matrix whose elements are a list of 2 tuples, the contents of 
    # each tuple is (probability, next_state, cost)
    #
    # The entry for the goal state has already been completed for you. You may 
    # find the convenience functions map_row_col_to_state() and 
    # map_state_to_row_col() helpful but are not required to use them.

    for s in range(self.rows * self.cols):
      if(s == goal_state):
        # Taking any action at the goal state will stay at the goal state
        P[s][0] = [(1.0, s, 0),
                   (0.0, 0, 0)]
        P[s][1] = [(1.0, s, 0),
                   (0.0, 0, 0)]
        P[s][2] = [(1.0, s, 0),
                   (0.0, 0, 0)]
        P[s][3] = [(1.0, s, 0),
                   (0.0, 0, 0)]
      else:
        for a in range(4):
          s_nxt, valid = self.compute_next_state(s, a)
          if valid:
            P[s][a] = [(1 - slip_prob, s_nxt, cost(s_nxt, valid)), (slip_prob, s, cost(s,valid))]
          elif valid == False:
            P[s][a] = [(1, s, cost(s, valid)), (0, 0, 0)]
    
    self.P = P

  def map_row_col_to_state(self, row, col):
    return row * self.cols + col

  def map_state_to_row_col(self, state):
    return state // self.cols, np.mod(state, self.cols)

  # Helper to help us see what s' is given action completes (no slip), assumes a in {0, 1, 2, 3}
  # If next action is valid return (s', True) if would go oob return (s, False)
  def compute_next_state(self, s, a):
    r,c = self.map_state_to_row_col(s)
    if a == 0: # go up
      r-=1
    if a == 1: # go right
      c+=1
    if a==2: # down
      r+=1
    if a==3: # left
      c-=1

    if r < 0 or r >= self.rows or c < 0 or c >= self.cols:
      return (s, False) # oob
    s_nxt = self.map_row_col_to_state(r, c)
    return (s_nxt, True)
      

  def eval_action(self, state, action):
    row, col = self.map_state_to_row_col(state)
    if action < 0 or action > 3:
      raise ValueError('Not a valid action')
    if row < 0 or row >= self.rows:
      raise ValueError('Row out of bounds')
    if col < 0 or col >= self.cols:
      raise ValueError('Col out of bounds')
    return self.P[state, action]
