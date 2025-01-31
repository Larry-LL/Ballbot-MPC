import numpy as np
import cvxpy as cp
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from scipy.optimize import minimize



class BallbotMPC:
    def __init__(self, Q, R, Qf, nx, nu, u_min, u_max):
        self.Q = Q   # State cost matrix
        self.R = R   # Control input cost matrix
        self.Qf = Qf  # Terminal state cost matrix
        self.nx = nx  # Number of state variables
        self.nu = nu  # Number of control inputs
        self.umin = u_min  # Minimum input constraint
        self.umax = u_max  # Maximum input constraint
        self.N = 30  # Prediction horizon
        self.T = 0.1  # Time step


    def trajectory_optimization_static_obs(self,start, goal, obstacles, num_waypoints=70):
        x_start = start[0]
        y_start = start[1]
        x_goal = goal[0]
        y_goal = goal[1]

        x_initial_guess = np.linspace(x_start,x_goal,num_waypoints)
        y_initial_guess = np.linspace(y_start,y_goal,num_waypoints)
        decision_var = np.hstack((x_initial_guess,y_initial_guess))
        
        def traj_cost(trajectory): 
            x = trajectory[:num_waypoints]
            y = trajectory[num_waypoints:] 
            return np.sum((np.diff(x)**2 + np.diff(y)**2))
        
        def traj_constraints():
            cons = []
            cons.append({'type': 'eq', 'fun': lambda traj: traj[0] - x_start})       # x_start
            cons.append({'type': 'eq', 'fun': lambda traj: traj[num_waypoints - 1] - x_goal})   # x_goal
            cons.append({'type': 'eq', 'fun': lambda traj: traj[num_waypoints] - y_start})      # y_start
            cons.append({'type': 'eq', 'fun': lambda traj: traj[2 * num_waypoints - 1] - y_goal})
            for obs in obstacles:
                center = np.array(obs["center"])
                radius = obs["radius"]
                for i in range(num_waypoints):
                    cons.append({
                    'type': 'ineq',  # Inequality: distance >= radius
                    'fun': lambda traj, i=i, center=center, radius=radius: 
                           np.linalg.norm([traj[i] - center[0], traj[num_waypoints + i] - center[1]]) - radius
                })
            return cons
        result = minimize(
        traj_cost,
        decision_var,
        constraints=traj_constraints(),
        method='SLSQP',
        options={'disp': True, 'maxiter': 500}
        # options={'disp': True}
    )
        if not result.success:
            raise ValueError(f"Optimization failed: {result.message}")
        # Reshape the optimized trajectory into Nx2
        optimized_trajectory = np.vstack((result.x[:num_waypoints], result.x[num_waypoints:])).T
        # plt.figure(figsize=(8, 8))
        # plt.plot(optimized_trajectory[:, 0], optimized_trajectory[:, 1], 'b-o', label="Optimized Trajectory")
        # plt.scatter(x_start, y_start, color='green', label="Start Point")
        # plt.scatter(x_goal, y_goal, color='red', label="Goal Point")
        
        # for obs in obstacles:
        #     center = obs["center"]
        #     radius = obs["radius"]
        #     circle = plt.Circle(center, radius, color='gray', alpha=0.5, label="Obstacle")
        #     plt.gca().add_artist(circle)

        # plt.xlabel("X")
        # plt.ylabel("Y")
        # plt.title("Trajectory Optimization with Obstacles")
        # plt.grid(True)
        # plt.legend()
        # plt.axis("equal")
        # plt.show()


        return optimized_trajectory

    def traj_follow_circle(self,target_center, target_radius, num_waypoints):
        
        theta_points = np.linspace(-np.pi/2, 3/2 * np.pi, num_waypoints)
        waypoints = np.zeros((num_waypoints, 2))  # Each row: [x, y]
        
        # Calculate waypoints
        for i in range(num_waypoints):
            waypoints[i, 0] = target_center[0] + target_radius * np.cos(theta_points[i])  # x-coordinate
            waypoints[i, 1] = target_center[1] + target_radius * np.sin(theta_points[i])  # y-coordinate
        
        return waypoints



    def calcaulte_continous_dynamics(self,x_current,u_current):  # linearize at referenced state x_current & u_current
        u_current = u_current.reshape(-1, 1)

        theta_x = x_current[0,0]
        theta_x_dot = x_current[2,0]
        T_YZ = u_current[0,0]

        theta_y = x_current[4,0]
        theta_y_dot = x_current[6,0]
        T_XZ = u_current[1,0]
        epsilon = 1e-8
        
        #calculate the continous dynamics with various linearized points
        A_cont_yz = np.array([    
            [0, 0, 1.0, 0],
            [0, 0, 0, 1.0],
            [
                float((2.537e35 * np.cos(theta_x) + 5.494e34 * T_YZ * np.sin(theta_x) + 6.468e34 * np.cos(theta_x)**3 
                + 3.664e33 * theta_x_dot**2 - 6.71e33 * theta_x_dot**2 * np.cos(theta_x)**2 
                + 9.276e33 * T_YZ * np.cos(theta_x)**2 * np.sin(theta_x)) / 
                (6.187e32 * np.cos(theta_x)**4 - 7.329e33 * np.cos(theta_x)**2 + 2.17e34+epsilon)), 
                0, 
                (4.4e-3 * theta_x_dot * np.sin(2.0 * theta_x)) / (2.2e-3 * np.cos(2.0 * theta_x) - 0.024+epsilon), 
                0
            ],
            [
                float((2.655e34 * theta_x_dot**2 * np.cos(theta_x) - 1.29e36 * np.cos(theta_x)**2 - 105.0 * np.cos(theta_x)**4 
                - 1.015e35 * T_YZ * np.sin(2.0 * theta_x) + 6.769e33 * theta_x_dot**2 * np.cos(theta_x)**3 
                + 7.076e35) / 
                (1.237e33 * np.cos(theta_x)**4 - 1.466e34 * np.cos(theta_x)**2 + 4.341e34+epsilon)), 
                0, 
                float((1.188e15 * theta_x_dot * np.sin(theta_x)) / (1.086e14 * np.sin(theta_x)**2 + 5.348e14+epsilon)), 
                0
            ]
        ])

        A_cont_xz = np.array([    #calculate the continous dynamics with various linearized points
            [0, 0, 1.0, 0],
            [0, 0, 0, 1.0],
            [
                (2.537e35 * np.cos(theta_y) + 5.494e34 * T_XZ * np.sin(theta_y) + 6.468e34 * np.cos(theta_y)**3 
                + 3.664e33 * theta_y_dot**2 - 6.71e33 * theta_y_dot**2 * np.cos(theta_y)**2 
                + 9.276e33 * T_XZ * np.cos(theta_y)**2 * np.sin(theta_y)) / 
                (6.187e32 * np.cos(theta_y)**4 - 7.329e33 * np.cos(theta_y)**2 + 2.17e34+epsilon), 
                0, 
                (4.4e-3 * theta_y_dot * np.sin(2.0 * theta_y)) / (2.2e-3 * np.cos(2.0 * theta_y) - 0.024+epsilon), 
                0
            ],
            [
                (2.655e34 * theta_y_dot**2 * np.cos(theta_y) - 1.29e36 * np.cos(theta_y)**2 - 105.0 * np.cos(theta_y)**4 
                - 1.015e35 * T_XZ * np.sin(2.0 * theta_y) + 6.769e33 * theta_y_dot**2 * np.cos(theta_y)**3 
                + 7.076e35) / 
                (1.237e33 * np.cos(theta_y)**4 - 1.466e34 * np.cos(theta_y)**2 + 4.341e34), 
                0, 
                (1.188e15 * theta_y_dot * np.sin(theta_y)) / (1.086e14 * np.sin(theta_y)**2 + 5.348e14+epsilon), 
                0
            ]
        ])
    # Compute each element of the matrix
    
        A_cont_upper = np.hstack((A_cont_yz, np.zeros((4, 4))))
        A_cont_lower = np.hstack((np.zeros((4, 4)), A_cont_xz))
        A_cont = np.vstack((A_cont_upper,A_cont_lower))

        B_cont_yz = np.array([
            [0],
            [0],
            [(0.0667*np.cos(theta_x))/(0.00445*np.cos(theta_x)**2 - 0.0264)],
            [-0.365/(0.00445*np.cos(theta_x)**2 - 0.0264)]
        ])
        B_cont_xz = np.array([
            [0],
            [0],
            [(0.0667*np.cos(theta_y))/(0.00445*np.cos(theta_y)**2 - 0.0264)],
            [-0.365/(0.00445*np.cos(theta_y)**2 - 0.0264)]
        ])


        B_cont_left = np.vstack((B_cont_yz,np.zeros((4,1))))
        B_cont_right = np.vstack((np.zeros((4,1)), B_cont_xz))
        B_cont = np.hstack((B_cont_left,B_cont_right))

        return A_cont, B_cont
    def continuous_dynamics(self, x, u, A_cont, B_cont):
        # continous dynamics first derivative
        xdot = A_cont @ x + B_cont @ u
        return xdot

    def discretize_system(self, A_cont, B_cont, T):
        # discretize time, euler integration 
        Ad = np.eye(self.nx) + A_cont * T  
        Bd = B_cont * T
        return Ad, Bd

    def compute_mpc(self, x_current, u_current, x_goal,x_ref): #our main mpc computational function 
        # Define decision variables
        x = cp.Variable((self.N, self.nx))   
        u = cp.Variable((self.N - 1, self.nu)) 
        
        constraints = []
        
        # Initial state constraint
        constraints.append(x[0, :] == x_current.flatten())

        # Input saturation constraints
        for i in range(self.N - 1):
            constraints.append(u[i, :] >= self.umin)
            constraints.append(u[i, :] <= self.umax)
        
        # Discretized dynamics
        A_cont, B_cont = self.calcaulte_continous_dynamics(x_current,u_current)  # 1st loop x_current & u_current will be zeros as setup in subroutine
        Ad, Bd = self.discretize_system(A_cont, B_cont, self.T)     #A_cont and B_cont 


        # Dynamics constraints
        for i in range(self.N - 1):
            constraints.append(x[i + 1, :] == Ad @ x[i, :] + Bd @ u[i, :])

        # Cost function
        cost = 0
        for i in range(self.N - 1):
            cost += cp.quad_form((x_ref - x[i, :]), self.Q) + cp.quad_form(u[i, :], self.R)

        cost += cp.quad_form((x_goal - x[self.N - 1, :]), self.Qf)  # Terminal cost

        # Solve the optimization problem
        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve()
  
        u_current = u[0, :].value
        
        if u_current is not None:
            u_current = u_current.reshape(-1, 1)
            x_current = Ad @ x_current + Bd @ u_current
        else:
            u_current = np.zeros((self.nu, 1))

        return x_current, u_current

# Q = np.diag([100, 100, 100, 100,1000, 1000, 1000, 100])
# R = np.diag([5,5])
# Qf = np.diag([8, 50, 8, 10,8, 50, 8, 10])
# x_goal = np.array([0, 5, 0, 0, 0, 6, 0, 0])
# nx = 8
# nu = 2
# u_min = -4.9
# u_max = 4.9
# tolerance =0.01

# ballbot_mpc = BallbotMPC(Q, R, Qf, nx, nu, u_min, u_max)

# x_current = np.zeros((8,1))
# u_current = np.zeros((nu))
# x_positions = []
# y_positions = []
# thetay_positions = []
# thetax_positions = []
# iteration = 0
# ahead_ref_idx = 7

# start = np.array([0,0])
# goal = np.array([x_goal[1],x_goal[5]])
# obstacles = [
#     {"center": (4, 4), "radius": 1},
#     {"center": (7, 8), "radius": 1.5},
# ]

# trajectory = ballbot_mpc.trajectory_optimization_static_obs(start, goal, obstacles)
# # print(trajcotory)


# while np.linalg.norm(x_current.flatten() - x_goal) > tolerance:
#     current_position = np.array([x_current[1], x_current[5]]).flatten()

#     distances = np.linalg.norm(trajectory - current_position, axis=1)

#     # Find the index of the closest waypoint
#     closest_idx = np.argmin(distances)
    
#     if closest_idx + ahead_ref_idx < len(trajectory):
#         x_ref = np.array([0, trajectory[closest_idx+ahead_ref_idx, 0], 0, 0, 0, trajectory[closest_idx+ahead_ref_idx, 1], 0, 0])
#     else:
#         x_ref = x_goal

#     x_current, u_current = ballbot_mpc.compute_mpc(x_current, u_current, x_goal , x_ref)
#     iteration +=1
#     x_positions.append(x_current[1, 0])  # x position (second state)
#     y_positions.append(x_current[5, 0])  
#     thetay_positions.append(x_current[0,0])
#     thetax_positions.append(x_current[4,0]) 
#     print(iteration)

# print("Total iteration time equals",iteration)




# #visualization
# plt.figure(figsize=(8, 8))
# plt.plot(x_positions, y_positions, 'bo-', label='Trajectory')  
# plt.scatter(x_goal[1], x_goal[5], color='red', marker='o', label='Goal Position (1,1)')  

# # Add obstacles
# for obstacle in obstacles:
#     center = obstacle["center"]
#     radius = obstacle["radius"]
#     circle = plt.Circle(center, radius, color='gray', alpha=0.5, label='Obstacle')
#     plt.gca().add_artist(circle)

# # Add labels and title
# plt.xlabel("X Position")
# plt.ylabel("Y Position")
# plt.title("Trajectory of X and Y Positions in MPC")
# plt.legend()
# plt.grid(True)
# plt.axis('equal')
# plt.show()

# # Visualization of Theta X and Theta Y positions
# plt.figure(figsize=(8, 8))
# plt.plot(thetax_positions, thetay_positions, 'go-', label='Tilt Trajectory')  
# plt.scatter(0, 0, color='red', marker='o', label='Goal Orientation (0,0)')  

# # Add obstacles for the second plot (optional, if relevant)
# # for obstacle in obstacles:
# #     center = obstacle["center"]
# #     radius = obstacle["radius"]
# #     circle = plt.Circle(center, radius, color='gray', alpha=0.5, label='Obstacle')
# #     plt.gca().add_artist(circle)

# # Add labels and title for the second plot
# plt.xlabel("Theta X Position")
# plt.ylabel("Theta Y Position")
# plt.title("Trajectory of Theta X and Theta Y Positions in MPC")
# plt.legend()
# plt.grid(True)
# plt.axis('equal')
# plt.show()

# # Print max values of tilt positions
# print("Max Theta X:", max(thetax_positions))
# print("Max Theta Y:", max(thetay_positions))
