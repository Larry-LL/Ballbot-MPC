import numpy as np
import cvxpy as cp
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle



class BallbotMPC_DO:
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

    def dynamic_obs_traj(self,t_tot, num_steps=50 ):
        t = np.linspace(0,t_tot,num_steps)
        #obs1 setup 
        amplitude = 1
        frequency = 0.5

        obs1_x = amplitude * np.sin(2 * np.pi * frequency * t) + 1  # Moves back and forth around x=1
        obs1_y = 0.5 * np.ones(num_steps)  # Constant y-coordinate
        obs1_traj = np.vstack((obs1_x,obs1_y))

        #obs2 setup
        obs2_x = amplitude * np.sin(2 * np.pi * frequency * t) + 3 # Moves back and forth around x=1
        obs2_y = 0.5 * np.ones(num_steps)  # Constant y-coordinate 
        obs2_traj = np.vstack((obs2_x,obs2_y))

        return obs1_traj, obs2_traj



    def calcaulte_continous_dynamics(self,x_current,u_current):  # linearize at referenced state x_current & u_current
        u_current = u_current.reshape(-1, 1)

        theta_x = x_current[0,0]
        theta_x_dot = x_current[1,0]
        T_YZ = u_current[0,0]

        theta_y = x_current[4,0]
        theta_y_dot = x_current[5,0]
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
    # return A_cont, B_cont 
    
    def continuous_dynamics(self, x, u, A_cont, B_cont):
        # continous dynamics first derivative
        xdot = A_cont @ x + B_cont @ u
        return xdot

    def discretize_system(self, A_cont, B_cont, T):
        # discretize time, euler integration 
        Ad = np.eye(self.nx) + A_cont * T  
        Bd = B_cont * T
        return Ad, Bd
    #return A_d, B_d 
    
    def compute_mpc(self, x_current, u_current, x_goal, x_ref, obs_traj):
        # Unpack obstacle trajectories
        obs1_traj, obs2_traj = obs_traj
        obs1_x, obs1_y = obs1_traj
        obs2_x, obs2_y = obs2_traj

        # Initialize variables
        u = np.zeros((self.N - 1, self.nu))  # Initial guess for control inputs
        x = np.zeros((self.N, self.nx))     # States along the trajectory
        x[0, :] = x_current.flatten()       # Initial state

        # Define bounds for control inputs
        bounds = [(self.umin[i], self.umax[i]) for i in range(self.nu)] * (self.N - 1)

        # Cost function
        def cost_function(u_flat):
            u = u_flat.reshape((self.N - 1, self.nu))
            cost = 0
            for i in range(self.N - 1):
                # State update
                A_cont, B_cont = self.calcaulte_continous_dynamics(x[i, :].reshape(-1, 1), u[i, :].reshape(-1, 1))
                Ad, Bd = self.discretize_system(A_cont, B_cont, self.T)
                x[i + 1, :] = Ad @ x[i, :] + Bd @ u[i, :]

                # State and control costs
                cost += (x[i, :] - x_ref).T @ self.Q @ (x[i, :] - x_ref) + u[i, :].T @ self.R @ u[i, :]

                # Obstacle avoidance penalties
                obs1_dist = np.sqrt((x[i, 0] - obs1_x[i])**2 + (x[i, 1] - obs1_y[i])**2)
                obs2_dist = np.sqrt((x[i, 0] - obs2_x[i])**2 + (x[i, 1] - obs2_y[i])**2)
                cost += 100 / max(obs1_dist, 0.1) + 100 / max(obs2_dist, 0.1)  # Avoid divide by zero

            # Terminal cost
            cost += (x[-1, :] - x_goal).T @ self.Qf @ (x[-1, :] - x_goal)
            return cost

        # Constraints
        def dynamics_constraints(u_flat):
            u = u_flat.reshape((self.N - 1, self.nu))
            x_dyn = np.zeros((self.N, self.nx))
            x_dyn[0, :] = x_current.flatten()
            constraints = []
            for i in range(self.N - 1):
                A_cont, B_cont = self.calcaulte_continous_dynamics(x_dyn[i, :].reshape(-1, 1), u[i, :].reshape(-1, 1))
                Ad, Bd = self.discretize_system(A_cont, B_cont, self.T)
                x_dyn[i + 1, :] = Ad @ x_dyn[i, :] + Bd @ u[i, :]
                constraints.extend((x_dyn[i + 1, :] - x[i + 1, :]).flatten())
            return np.array(constraints)

        # Solve optimization
        result = minimize(
            cost_function,
            u.flatten(),
            method='SLSQP',
            bounds=bounds,
            constraints={'type': 'eq', 'fun': dynamics_constraints},
            options={'maxiter': 500, 'disp': True}
        )

        # Extract control inputs
        u_optimized = result.x.reshape((self.N - 1, self.nu))
        u_current = u_optimized[0, :].reshape(-1, 1)

        # Update current state
        A_cont, B_cont = self.calcaulte_continous_dynamics(x_current, u_current)
        Ad, Bd = self.discretize_system(A_cont, B_cont, self.T)
        x_current = Ad @ x_current + Bd @ u_current

        return x_current, u_current



# Q = np.diag([100, 100, 100, 100,1000, 1000, 1000, 100])
# R = np.diag([5,5])
# Qf = np.diag([8, 50, 8, 10,8, 50, 8, 10])
# x_goal = np.array([0, 5, 0, 0, 0, 5, 0, 0])
# nx = 8
# nu = 2
# u_min = -4.9
# u_max = 4.9
# tolerance =0.01

# ballbot_mpc = BallbotMPC_DO(Q, R, Qf, nx, nu, u_min, u_max)

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
# obs1_radius = 0.5
# obs2_radius = 0.7
# t_tot = 5
# num_steps = 50

# trajectory, obs1_x, obs1_y, obs2_x, obs2_y, time_steps = ballbot_mpc.dynamic_obs_traj(start, goal, obs1_radius, obs2_radius, t_tot, num_steps)

# # Plot and animate
# fig, ax = plt.subplots()
# ax.set_xlim(-1, 6)
# ax.set_ylim(-1, 6)
# ax.set_title("Planned Trajectory with Animated Moving Obstacle")
# ax.set_xlabel("X")
# ax.set_ylabel("Y")

# # Static elements
# ax.plot(trajectory[:, 0], trajectory[:, 1], 'b-', label="Planned Trajectory")  # Planned trajectory
# robot_point, = ax.plot([], [], 'bo', label="Robot Position")                  # Robot position
# obstacle_center, = ax.plot([], [], 'ro', label="Moving Obstacle")
# obstacle_radius = Circle((0, 0), obs1_radius, color='r', alpha=0.3)  # Transparent circle
# ax.add_patch(obstacle_radius)  # Add circle to the plot

# obstacle_center2, = ax.plot([], [], 'ro', label="Moving Obstacle2")
# obstacle_radius2 = Circle((0, 0), obs2_radius, color='r', alpha=0.3)  # Transparent circle
# ax.add_patch(obstacle_radius2)  # A



# # Animation update function
# def update(frame):
#     # Update robot position
#     robot_point.set_data([trajectory[frame, 0]], [trajectory[frame, 1]])  # Wrap in lists

#     # Update obstacle position
#     obstacle_center.set_data([obs1_x[frame]], [obs1_y[frame]])  # Wrap in lists
#     obstacle_radius.center = (obs1_x[frame], obs1_y[frame])     # Update circle center
    
#     obstacle_center2.set_data([obs2_x[frame]], [obs2_y[frame]])  # Wrap in lists
#     obstacle_radius2.center = (obs2_x[frame], obs2_y[frame])     # Update circle center
    
#     return robot_point, obstacle_center, obstacle_radius,obstacle_center2, obstacle_radius2

# # Create the animation
# ani = FuncAnimation(fig, update, frames=num_steps, interval=100, blit=True)

# # Add legend and show plot
# ax.legend()
# plt.show()
