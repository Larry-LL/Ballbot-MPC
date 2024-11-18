import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

# Parameters
N = 20  # Number of waypoints
x_start, y_start = 0.0, 0.0  # Start point
x_goal, y_goal = 5.0, 5.0    # Goal point
obstacles = [
    {"center": (2.0, 2.0), "radius": 0.5},  # Circular obstacle 1
    {"center": (3.0, 4.0), "radius": 0.7},  # Circular obstacle 2
]

# Flattened initial guess (straight-line path)
z0 = np.linspace(0, 5, N * 2)

# Objective function
def objective(z):
    z = z.reshape(N, 2)  # Reshape into (N, 2) array
    cost = 0
    for i in range(N - 1):
        cost += np.sum((z[i + 1] - z[i]) ** 2)  # Smoothness cost (squared distance)
    return cost

# Constraints
constraints = []

# Start and goal constraints
constraints.append({'type': 'eq', 'fun': lambda z: z[0] - x_start})  # x_start
constraints.append({'type': 'eq', 'fun': lambda z: z[1] - y_start})  # y_start
constraints.append({'type': 'eq', 'fun': lambda z: z[-2] - x_goal})  # x_goal
constraints.append({'type': 'eq', 'fun': lambda z: z[-1] - y_goal})  # y_goal

# Obstacle avoidance constraints
for obs in obstacles:
    obs_center = np.array(obs["center"])
    obs_radius = obs["radius"]
    for i in range(N):
        constraints.append({
            'type': 'ineq',  # Inequality constraint: >= 0
            'fun': lambda z, i=i, obs_center=obs_center, obs_radius=obs_radius: 
                np.linalg.norm(z[2*i:2*i+2] - obs_center) - obs_radius
        })

# Solve the optimization problem
result = minimize(objective, z0, constraints=constraints, method='SLSQP')

# Extract the solution
if result.success:
    z_opt = result.x.reshape(N, 2)
    x_path, y_path = z_opt[:, 0], z_opt[:, 1]
else:
    print("Optimization failed:", result.message)
    z_opt = z0.reshape(N, 2)
    x_path, y_path = z_opt[:, 0], z_opt[:, 1]

# Plot the results
plt.figure(figsize=(8, 8))
plt.plot(x_path, y_path, 'b-o', label="Planned Path")
plt.scatter([x_start, x_goal], [y_start, y_goal], color="red", label="Start/Goal")

# Plot obstacles
for obs in obstacles:
    circle = plt.Circle(obs["center"], obs["radius"], color="gray", alpha=0.5)
    plt.gca().add_artist(circle)

plt.legend()
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Optimization-Based Path Planning (SciPy)")
plt.axis("equal")
plt.grid(True)
plt.show()
