import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

def optimize_and_plot_trajectory(x_start, y_start, x_goal, y_goal, obstacles, N=30):
    """
    Optimizes and plots a trajectory using scipy.optimize.
    
    Parameters:
    - x_start, y_start: Start point coordinates
    - x_goal, y_goal: Goal point coordinates
    - obstacles: List of obstacles, each defined as {"center": (x, y), "radius": r}
    - N: Number of waypoints
    """
    # Initial guess: Straight line from start to goal
    x_guess = np.linspace(x_start, x_goal, N)
    y_guess = np.linspace(y_start, y_goal, N)
    initial_guess = np.hstack((x_guess, y_guess))  # Flatten to a 1D array

    def cost_function(trajectory):
        """Minimizes smoothness of trajectory."""
        x = trajectory[:N]
        y = trajectory[N:]
        return np.sum((np.diff(x)**2 + np.diff(y)**2))  # Sum of squared differences

    def constraints():
        """Generates equality and inequality constraints."""
        cons = []
        # Start and goal point constraints
        cons.append({'type': 'eq', 'fun': lambda traj: traj[0] - x_start})       # x_start
        cons.append({'type': 'eq', 'fun': lambda traj: traj[N - 1] - x_goal})   # x_goal
        cons.append({'type': 'eq', 'fun': lambda traj: traj[N] - y_start})      # y_start
        cons.append({'type': 'eq', 'fun': lambda traj: traj[2 * N - 1] - y_goal})  # y_goal

        # Obstacle avoidance constraints
        for obs in obstacles:
            center = np.array(obs["center"])
            radius = obs["radius"]
            for i in range(N):
                cons.append({
                    'type': 'ineq',  # Inequality: distance >= radius
                    'fun': lambda traj, i=i, center=center, radius=radius: 
                           np.linalg.norm([traj[i] - center[0], traj[N + i] - center[1]]) - radius
                })
        return cons

    # Solve the optimization problem
    result = minimize(
        cost_function,
        initial_guess,
        constraints=constraints(),
        method='SLSQP',
        options={'disp': True}
    )

    if not result.success:
        raise ValueError(f"Optimization failed: {result.message}")

    # Reshape the optimized trajectory into Nx2
    optimized_trajectory = np.vstack((result.x[:N], result.x[N:])).T

    # Plot the trajectory
    plt.figure(figsize=(8, 8))
    plt.plot(optimized_trajectory[:, 0], optimized_trajectory[:, 1], label="Optimized Trajectory", marker='o')
    plt.scatter(x_start, y_start, color='green', label="Start Point")
    plt.scatter(x_goal, y_goal, color='red', label="Goal Point")

    # Plot obstacles
    for obs in obstacles:
        center = obs["center"]
        radius = obs["radius"]
        circle = plt.Circle(center, radius, color='gray', alpha=0.5, label="Obstacle")
        plt.gca().add_artist(circle)

    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Trajectory Optimization with Obstacle Avoidance")
    plt.grid()
    plt.legend()
    plt.axis("equal")
    plt.show()

# Example Usage
x_start, y_start = 0, 0
x_goal, y_goal = 10, 10
obstacles = [
    {"center": (4, 4), "radius": 1},
    {"center": (7, 8), "radius": 1.5},
]
optimize_and_plot_trajectory(x_start, y_start, x_goal, y_goal, obstacles)
