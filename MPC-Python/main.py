import numpy as np
import cvxpy as cp
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from ballbot_mpc import BallbotMPC

Q = np.diag([100, 100, 100, 100,1000, 1000, 1000, 100])
R = np.diag([5,5])
Qf = np.diag([8, 50, 8, 10,8, 50, 8, 10])
nx = 8
nu = 2
u_min = -4.9
u_max = 4.9
tolerance =0.02

ballbot_mpc = BallbotMPC(Q, R, Qf, nx, nu, u_min, u_max)

# Q = np.diag([100, 100, 100, 100,1000, 1000, 1000, 100])
# R = np.diag([5,5])
# Qf = np.diag([8, 50, 8, 10,8, 50, 8, 10])
# nx = 8
# nu = 2
# u_min = -4.9
# u_max = 4.9
# tolerance =0.01


if False:
    x_current = np.zeros((8,1))
    u_current = np.zeros((nu))
    x_positions = []
    y_positions = []
    thetay_positions = []
    thetax_positions = []
    iteration = 0
    ahead_ref_idx = 7
    x_goal = np.array([0, 5, 0, 0, 0, 6, 0, 0])
    start = np.array([0,0])
    goal = np.array([x_goal[1],x_goal[5]])
    obstacles = [
        {"center": (4, 4), "radius": 1},
        {"center": (7, 8), "radius": 1.5},
    ]

    trajectory = ballbot_mpc.trajectory_optimization_static_obs(start, goal, obstacles)
    # print(trajcotory)


    while np.linalg.norm(x_current.flatten() - x_goal) > tolerance:
        current_position = np.array([x_current[1], x_current[5]]).flatten()

        distances = np.linalg.norm(trajectory - current_position, axis=1)

        # Find the index of the closest waypoint
        closest_idx = np.argmin(distances)
        
        if closest_idx + ahead_ref_idx < len(trajectory):
            x_ref = np.array([0, trajectory[closest_idx+ahead_ref_idx, 0], 0, 0, 0, trajectory[closest_idx+ahead_ref_idx, 1], 0, 0])
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
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
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
    plt.xlabel("Theta X Position")
    plt.ylabel("Theta Y Position")
    plt.title("Trajectory of Theta X and Theta Y Positions in MPC")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()



if True: 
    x_current = np.zeros((8,1))
    u_current = np.zeros((nu))
    x_positions = []
    y_positions = []
    thetay_positions = []
    thetax_positions = []
    iteration = 0
    ahead_ref_idx = 2
    start = np.array([0,0])
    target_center = [1,1]
    target_radius = 1
    num_waypoints = 30 
    x_start = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    x_goal = np.array([0, 1, 0, 0, 0, 0, 2, 0])
    idx = 0
    trajectory = ballbot_mpc.traj_follow_circle(target_center, target_radius, num_waypoints)
    x_ref = np.array([0, trajectory[0, 0], 0, 0, 0, trajectory[0, 1], 0, 0])
    while idx < num_waypoints-ahead_ref_idx:
        current_position = np.array([x_current[1], x_current[5]]).flatten()
        
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
    plt.plot(x_positions, y_positions, 'bo-', label='Real Robot Trajectory') 
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


