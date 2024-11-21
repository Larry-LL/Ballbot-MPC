import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Define the obstacle's motion
def obstacle_position(t):
    # Example: Linear motion along x-axis
    x = 2 + 0.5 * t  # Starting at (2, 2), moving right
    y = 2
    return np.array([x, y])

# Define the robot's trajectory
time_steps = np.linspace(0, 10, 100)  # 10 seconds, 100 steps
robot_trajectory = np.array([[t, np.sin(t)] for t in time_steps])  # Example trajectory

# Initialize the plot
fig, ax = plt.subplots()
ax.set_xlim(0, 10)
ax.set_ylim(-2, 5)
ax.set_title("Trajectory Planning with Moving Obstacle")

# Draw static elements
robot_line, = ax.plot([], [], 'b-', label="Robot Trajectory")  # Robot trajectory
robot_point, = ax.plot([], [], 'bo', label="Robot Position")   # Robot current position
obstacle, = ax.plot([], [], 'ro', label="Moving Obstacle")     # Obstacle

# Animation update function
def update(frame):
    t = time_steps[frame]
    
    # Update obstacle position
    obs_pos = obstacle_position(t)
    obstacle.set_data([obs_pos[0]], [obs_pos[1]])  # Use lists for single points
    
    # Update robot position
    robot_pos = robot_trajectory[frame]
    robot_point.set_data([robot_pos[0]], [robot_pos[1]])  # Use lists for single points
    
    # Update trajectory
    robot_line.set_data(robot_trajectory[:frame+1, 0], robot_trajectory[:frame+1, 1])
    
    return robot_line, robot_point, obstacle

# Create the animation
ani = FuncAnimation(
    fig, update, frames=len(time_steps), interval=100, blit=True
)

# Add legend
ax.legend()

# Show the animation
plt.show()
