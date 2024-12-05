import numpy as np
import cvxpy as cp
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from ballbot_mpc import BallbotMPC
from ballbot_mpc_DynamicObs import BallbotMPC_DO
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle

Q = np.diag([100, 100, 100, 100,1000, 1000, 1000, 100])
R = np.diag([5,5])
Qf = np.diag([8, 50, 8, 10,8, 50, 8, 10])
nx = 8
nu = 2
u_min = -4.9
u_max = 4.9
tolerance =0.02


# Q = np.diag([100, 100, 100, 100,1000, 1000, 1000, 100])
# R = np.diag([5,5])
# Qf = np.diag([8, 50, 8, 10,8, 50, 8, 10])
# nx = 8
# nu = 2
# u_min = -4.9
# u_max = 4.9
# tolerance =0.01

case = 1   # 1 = static obstacle || 2=circular traj following || 3 = known dynamic obs avoidance

if case == 1:
    ballbot_mpc = BallbotMPC(Q, R, Qf, nx, nu, u_min, u_max)
    x_current = np.zeros((8,1))
    u_current = np.zeros((nu))
    x_positions = []
    y_positions = []
    thetay_positions = []
    thetax_positions = []
    iteration = 0
    ahead_ref_idx = 7
    x_goal = np.array([0, 5, 0, 0, 0, 5, 0, 0])
    start = np.array([0,0])
    goal = np.array([x_goal[1],x_goal[5]])
    tolerance = 0.6
    obstacles = [
        {"center": (3, 3), "radius": 1},

    ]
    closest_idx = 0
    trajectory = ballbot_mpc.trajectory_optimization_static_obs(start, goal, obstacles)
    # print(trajcotory)

    x_ref = np.array([0, trajectory[0, 0], 0, 0, 0, trajectory[0, 1], 0, 0])
 

    while np.linalg.norm(x_current.flatten() - x_goal) > tolerance:
        current_position = np.array([x_current[1], x_current[5]]).flatten()

        distances = np.linalg.norm(trajectory - current_position, axis=1)

        # Find the index of the closest waypoint
        current_error = np.linalg.norm(x_current.flatten() - x_ref)
        
        if closest_idx + ahead_ref_idx < len(trajectory) and current_error <= tolerance:
            x_ref = np.array([0, trajectory[closest_idx+ahead_ref_idx, 0], 0, 0, 0, trajectory[closest_idx+ahead_ref_idx, 1], 0, 0])
            closest_idx += ahead_ref_idx
        else:
            x_ref = x_goal
        x_current, u_current = ballbot_mpc.compute_mpc(x_current, u_current, x_goal , x_ref)
        iteration +=1
        x_positions.append(x_current[1, 0])  # x position (second state)
        y_positions.append(x_current[5, 0])  
        thetay_positions.append(x_current[0,0])
        thetax_positions.append(x_current[4,0]) 
        print(iteration)

    print("Total iteration time equals",iteration)




    #visualization
    plt.figure(figsize=(8, 8))
    plt.plot(x_positions, y_positions, 'bo-', label='Trajectory')  
    plt.scatter(x_goal[1], x_goal[5], color='red', marker='o', label='Goal Position (1,1)')  

    # Add obstacles
    for obstacle in obstacles:
        center = obstacle["center"]
        radius = obstacle["radius"]
        circle = plt.Circle(center, radius, color='gray', alpha=0.5, label='Obstacle')
        plt.gca().add_artist(circle)

    # Add labels and title
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position(m)")
    plt.title("Trajectory of X and Y Positions in MPC")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

    # Visualization of Theta X and Theta Y positions
    plt.figure(figsize=(8, 8))
    plt.plot(thetax_positions, thetay_positions, 'go-', label='Tilt Trajectory')  
    plt.scatter(0, 0, color='red', marker='o', label='Goal Orientation (0,0)')  

    # Add obstacles for the second plot (optional, if relevant)
    # for obstacle in obstacles:
    #     center = obstacle["center"]
    #     radius = obstacle["radius"]
    #     circle = plt.Circle(center, radius, color='gray', alpha=0.5, label='Obstacle')
    #     plt.gca().add_artist(circle)

    # Add labels and title for the second plot
    plt.xlabel("Theta X Angle(rad)")
    plt.ylabel("Theta Y Angle(rad)")
    plt.title("Trajectory of Theta X and Theta Y Positions in MPC")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()



if case == 2: 
    ballbot_mpc = BallbotMPC(Q, R, Qf, nx, nu, u_min, u_max)
    x_current = np.zeros((8,1))
    u_current = np.zeros((nu))
    x_positions = []
    y_positions = []
    thetay_positions = []
    thetax_positions = []
    iteration = 0
    ahead_ref_idx = 3
    start = np.array([0,0])
    target_center = [1,1]
    target_radius = 1
    num_waypoints = 30 
    x_start = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    # x_goal = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    idx = 0
    trajectory = ballbot_mpc.traj_follow_circle(target_center, target_radius, num_waypoints)
    x_ref = np.array([0, trajectory[0, 0], 0, 0, 0, trajectory[0, 1], 0, 0])
    while idx < num_waypoints-ahead_ref_idx:
        current_position = np.array([x_current[1], x_current[5]]).flatten()
        x_goal = np.array([0, trajectory[idx,0], 0, 0, 0, trajectory[idx,1], 0, 0])
        
        if idx + ahead_ref_idx < len(trajectory) and np.linalg.norm(x_current.flatten() - x_ref) <= tolerance :
            x_ref = np.array([0, trajectory[idx+ahead_ref_idx, 0], 0, 0, 0, trajectory[idx+ahead_ref_idx, 1], 0, 0])
            idx+=ahead_ref_idx

        else:
            x_ref = np.array([0, trajectory[idx, 0], 0, 0, 0, trajectory[idx, 1], 0, 0])

        print(idx)
        x_current, u_current = ballbot_mpc.compute_mpc(x_current, u_current, x_goal , x_ref)
        iteration +=1
        x_positions.append(x_current[1, 0])  # x position (second state)
        y_positions.append(x_current[5, 0])  
        thetay_positions.append(x_current[0,0])
        thetax_positions.append(x_current[4,0]) 
        # print(iteration)
        # if iteration >50 and np.linalg.norm(x_current.flatten() - stop_coordinates) < tolerance:
        #     break
    plt.figure(figsize=(8, 8))
    plt.plot(x_positions, y_positions, 'b-', label='Real Robot Trajectory') 
    plt.plot(trajectory[:,0],trajectory[:,1],'-', label = 'Planned Trajectory') 
    plt.scatter(x_goal[1], x_goal[5], color='red', marker='o', label='Goal Position (1,1)')  
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.title("Trajectory of X and Y Positions in MPC")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()
    plt.figure(figsize=(8, 8))
    plt.plot(thetax_positions, thetay_positions, 'go-', label='Tilt Trajectory')  
    plt.scatter(0, 0, color='red', marker='o', label='Goal Orientation (0,0)')  
    plt.xlabel("Theta X Position")
    plt.ylabel("Theta Y Position")
    plt.title("Trajectory of Theta X and Theta Y Positions in MPC")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

    # Print max values of tilt positions
    print("Max Theta X:", max(thetax_positions))
    print("Max Theta Y:", max(thetay_positions))
    print("Total iteration time equals",iteration)

if case == 3: 
    ballbot_mpc = BallbotMPC_DO(Q, R, Qf, nx, nu, u_min, u_max)
    x_current = np.zeros((8,1))
    u_current = np.zeros((nu))
    x_positions = []
    y_positions = []
    thetay_positions = []
    thetax_positions = []
    iteration = 0
    ahead_ref_idx = 5
    x_goal = np.array([0, 5, 0, 0, 0, 5, 0, 0])
    start = np.array([0,0])
    goal = np.array([x_goal[1],x_goal[5]])
    obs1_radius = 0.5
    obs2_radius = 0.7
    t_tot = 5
    num_steps = 50
    tolerance = 0.1
    closest_idx = 0

    trajectory, obs1_x, obs1_y, obs2_x, obs2_y, time_steps = ballbot_mpc.dynamic_obs_traj(start, goal, obs1_radius, obs2_radius, t_tot, num_steps)

    x_ref = np.array([0, trajectory[0, 0], 0, 0, 0, trajectory[0, 1], 0, 0])


    while np.linalg.norm(x_current.flatten() - x_goal) > tolerance:
        current_position = np.array([x_current[1], x_current[5]]).flatten()

        distances = np.linalg.norm(trajectory - current_position, axis=1)

        # Find the index of the closest waypoint
        current_error = np.linalg.norm(x_current.flatten() - x_ref)
        
        if closest_idx + ahead_ref_idx < len(trajectory) and current_error <= tolerance:
            x_ref = np.array([0, trajectory[closest_idx+ahead_ref_idx, 0], 0, 0, 0, trajectory[closest_idx+ahead_ref_idx, 1], 0, 0])
            closest_idx += ahead_ref_idx
        else:
            x_ref = x_goal
        x_current, u_current = ballbot_mpc.compute_mpc(x_current, u_current, x_goal , x_ref)
        iteration +=1
        x_positions.append(x_current[1, 0])  # x position (second state)
        y_positions.append(x_current[5, 0])  
        thetay_positions.append(x_current[0,0])
        thetax_positions.append(x_current[4,0]) 
        print(iteration)
        # print(iteration)




    # Plot and animate
    fig, ax = plt.subplots()
    ax.set_xlim(-1, 6)
    ax.set_ylim(-1, 6)
    ax.set_title("Planned Trajectory with Animated Moving Obstacle")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")

    # Static elements
    ax.plot(trajectory[:, 0], trajectory[:, 1], 'b-', label="Planned Trajectory")  # Planned trajectory
    robot_point, = ax.plot([], [], 'bo', label="Robot Position")                  # Robot position
    obstacle_center, = ax.plot([], [], 'ro', label="Moving Obstacle")
    obstacle_radius = Circle((0, 0), obs1_radius, color='r', alpha=0.3)  # Transparent circle
    ax.add_patch(obstacle_radius)  # Add circle to the plot

    obstacle_center2, = ax.plot([], [], 'ro', label="Moving Obstacle2")
    obstacle_radius2 = Circle((0, 0), obs2_radius, color='r', alpha=0.3)  # Transparent circle
    ax.add_patch(obstacle_radius2)  # Add circle to the plot

    # Add new elements to the static plot
    ax.plot(trajectory[:, 0], trajectory[:, 1], 'bo-', label='Trajectory')  # Trajectory line with points
    ax.scatter(x_goal[0], x_goal[1], color='red', marker='o', label='Goal Position')  # Goal position

    # Add animated element for x_positions and y_positions
    animated_positions, = ax.plot([], [], 'g*', label='Dynamic Points')  # Green stars for dynamic positions

    # Animation update function
    def update(frame):
        # Update robot position
        robot_point.set_data([trajectory[frame, 0]], [trajectory[frame, 1]])  # Wrap in lists

        # Update obstacle positions
        obstacle_center.set_data([obs1_x[frame]], [obs1_y[frame]])  # Wrap in lists
        obstacle_radius.center = (obs1_x[frame], obs1_y[frame])     # Update circle center
        
        obstacle_center2.set_data([obs2_x[frame]], [obs2_y[frame]])  # Wrap in lists
        obstacle_radius2.center = (obs2_x[frame], obs2_y[frame])     # Update circle center

        # Update x_positions and y_positions
        animated_positions.set_data(x_positions[:frame], y_positions[:frame])  # Update trajectory of points
        
        return robot_point, obstacle_center, obstacle_radius, obstacle_center2, obstacle_radius2, animated_positions

    # Create the animation
    ani = FuncAnimation(fig, update, frames=num_steps, interval=100, blit=True)

    # Add legend and show plot
    ax.legend()
    plt.show()