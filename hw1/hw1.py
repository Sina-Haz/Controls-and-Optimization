import numpy as np

# 1c) check eigenvals of undelayed v. delayed system

A = np.array([[0.5, 0], [0, 1]])
B = np.array([[0], [1]])
K = np.array([-1, -1.25])

BK = np.outer(B, K)

# undelayed dynamics: A+BK
undelayed = A + BK
delayed = np.block([[A, BK], [np.eye(2), np.zeros((2, 2))]])

rho_undelayed = max(np.abs(np.linalg.eigvals(undelayed)))
rho_delayed = max(np.abs(np.linalg.eigvals(delayed)))

print("Max complex modulus of eigenvalues of undelayed & delayed system, problem 1: ")
print(f"{rho_undelayed=}")
print(f"{rho_delayed=}")

# For problem 5, write Jf(x), evaluate at equilibrium points and get Eigenvalues
def Jf(x):
    return np.array([[0, 1], [-1 + (x[0]+1)**2, -1]])

eqs = [[-1, 2], [-1 - np.sqrt(3), 2], [-1 + np.sqrt(3), 2]]
print("Jacobian of f(x) evaluated at equilibrium points and its eigenvalues: ")
Jfe1 = Jf(eqs[0])
print(f"For x* = {[-1, 2]}, we have jacobian:\n {Jfe1}\n this has eigenvalues: {np.linalg.eigvals(Jfe1)}")

Jfe2 = Jf(eqs[1])
print(f"For x* = {eqs[1]}, we have jacobian:\n {Jfe2}\n this has eigenvalues: {np.linalg.eigvals(Jfe2)}")


Jfe3 = Jf(eqs[2])
print(f"For x* = {eqs[2]}, we have jacobian:\n {Jfe3}\n this has eigenvalues: {np.linalg.eigvals(Jfe3)}")




