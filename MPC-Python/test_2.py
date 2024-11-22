import numpy as np
from scipy.optimize import minimize, NonlinearConstraint

# Define the obstacle position and minimum distance
obs_pos = np.array([2.0, 2.0])  # Obstacle position
d_min = 1.0  # Minimum distance to obstacle

# Quadratic obstacle constraint: (x - obs_pos)^2 >= d_min^2
def obstacle_constraint(x):
    return (x[0] - obs_pos[0])**2 + (x[1] - obs_pos[1])**2 - d_min**2

# Nonlinear constraint for obstacle avoidance
nonlinear_constraint = NonlinearConstraint(obstacle_constraint, 0, np.inf)

# Define a simple cost function: minimize distance to a goal
def cost_function(x):
    goal = np.array([5.0, 5.0])  # Goal position
    return np.linalg.norm(x - goal)

# Initial guess for the optimization
x0 = np.array([0.0, 0.0])

# Perform optimization
result = minimize(cost_function, x0, constraints=[nonlinear_constraint], method='SLSQP')

# Display the result
print("Optimal Position:", result.x)
print("Minimum Distance to Goal:", result.fun)
print("Success:", result.success)
print("Message:", result.message)
