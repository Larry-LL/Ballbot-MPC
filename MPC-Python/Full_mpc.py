import numpy as np
# import cvxpy as cp
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle

class BallbotMPC_DO:
    def __init__(self, Q, R, Qf, nx, nu, u_min, u_max):
        self.Q = Q   # State cost 
        self.R = R   # Control input cost matrix
        self.Qf = Qf  # Terminal state cost matrix
        self.nx = nx  # Number of state variables
        self.nu = nu  # Number of control inputs
        self.umin = u_min  # Minimum input constraint
        self.umax = u_max  # Maximum input constraint
        self.N = 15  # Prediction horizon
        self.T = 0.1  # Time step
        self.tilt_angle_limit = 0.349  #theta x and y tilt angle limitation 

    # def dynamic_obs_traj(self,t_tot, num_steps):
    #     t = np.linspace(0,t_tot,num_steps)
    #     #obs1 setup 
    #     amplitude = 1
    #     frequency = 0.5

    #     obs1_x = amplitude * np.sin(2 * np.pi * frequency * t) + 1  # Moves back and forth around x=1
    #     obs1_y = 1.5 * np.ones(num_steps)  # Constant y-coordinate
    #     obs1_traj = np.vstack((obs1_x,obs1_y))

    #     #obs2 setup
    #     obs2_x = amplitude * np.sin(2 * np.pi * frequency * t) + 3 # Moves back and forth around x=1
    #     obs2_y = 1.5 * np.ones(num_steps)  # Constant y-coordinate 
    #     obs2_traj = np.vstack((obs2_x,obs2_y))
        

    #     return obs1_traj, obs2_traj
    def dynamic_obs_traj(self,t_tot):
        num_steps = int(t_tot / self.T) + 1
        t = np.linspace(0,t_tot,num_steps)
        #obs1 setup 
        amplitude = 1
        frequency = 0.5

        obs1_x = amplitude * np.sin(2 * np.pi * frequency * t) + 1  # Moves back and forth around x=1
        obs1_y = 1.5 * np.ones(num_steps)  # Constant y-coordinate
        obs1_traj = np.vstack((obs1_x,obs1_y))

        #obs2 setup
        obs2_x = amplitude * np.sin(2 * np.pi * frequency * t) + 3 # Moves back and forth around x=1
        obs2_y = 1.5 * np.ones(num_steps)  # Constant y-coordinate 
        obs2_traj = np.vstack((obs2_x,obs2_y))
        

        return obs1_traj, obs2_traj


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
    # return A_cont, B_cont 

    def discretize_system(self, A_cont, B_cont, T):
        # discretize time, euler integration 
        Ad = np.eye(self.nx) + A_cont * T  
        Bd = B_cont * T
        return Ad, Bd
    #return A_d, B_d 
    
    def compute_mpc(self, x_current, u_current, x_ref, obs_radius,obs1_traj, obs2_traj):

        # Prediction horizon and dimensions
        N = self.N
        nx = self.nx
        nu = self.nu
        
        # In compute_mpc, check if self.u_prev exists
        if hasattr(self, 'u_prev'):
            u0 = np.vstack((self.u_prev[1:], self.u_prev[-1]))
        else:
            u0 = np.zeros((N - 1, nu))


# Initialize states by interpolating between current state and goal state
        x0 = np.linspace(x_current.flatten(), x_goal, N)

        # Flatten control and state variables for optimization
        z0 = np.hstack((u0.flatten(), x0.flatten()))

        # Bounds for control inputs and states
        u_bounds = [(self.umin, self.umax)] * ((N - 1) * nu)
        x_bounds = [(None, None)] * (N * nx)  # No specific state bounds
        bounds = u_bounds + x_bounds

        # Cost function
        def cost_function(z):
            # Extract control inputs and states from z
            u = z[: (N - 1) * nu].reshape(N - 1, nu)
            x = z[(N - 1) * nu :].reshape(N, nx)
        

            safe_distance = 0.5
            obstacle_weight = 20
            cost = 0
            for i in range(N - 1):
                obs1_x, obs1_y = obs1_traj[:, i]
                distance_to_obs = ((x[i, 1] - obs1_x)**2 + (x[i, 5] - obs1_y)**2)
                cost += (x[i] - x_goal).T @ self.Q @ (x[i] - x_goal) + u[i].T @ self.R @ u[i]
                if distance_to_obs < safe_distance**2:
                    cost += obstacle_weight / distance_to_obs
                            #termnial cost
            cost += (x[N - 1] - x_goal).T @ self.Qf @ (x[N - 1] - x_goal)
            return cost


        # Dynamic constraint
        def dynamic_constraint(z):
            u = z[: (N - 1) * nu].reshape(N - 1, nu)
            x = z[(N - 1) * nu :].reshape(N, nx)
    
            constraints = []
            for i in range(N - 1):
                x_i = x[i].reshape(-1, 1)  # (8, 1)
                u_i = u[i].reshape(-1, 1)
                A_cont, B_cont = self.calcaulte_continous_dynamics(x_i, u_i)
                Ad, Bd = self.discretize_system(A_cont, B_cont, self.T)
                x_next_predicted = (Ad @ x_i + Bd @ u_i).flatten()
                constraints.append(x[i + 1] - x_next_predicted)  # Discretized dynamics
            return np.concatenate(constraints)

        # Initial state constraint
        def initial_constraint(z):
            x = z[(N - 1) * nu :].reshape(N, nx)
            return (x[0] - x_current).flatten()
        
        # obstacle avoidance constraint
        def obstacle_avoidance_constraint(z,obs1_traj, obs2_traj):
            x = z[(N - 1) * nu :].reshape(N, nx)
            constraints = []
            for i in range(N):
                # Obstacle 1 avoidance
                obs1_x, obs1_y = obs1_traj[:, i]
                # print("x",x[i, 1].shape)
                constraints.append(
                    (x[i, 1] - obs1_x)**2 + (x[i, 5] - obs1_y)**2 - (obs_radius+0.1)**2)
                # Obstacle 2 avoidance
                # obs2_x, obs2_y = obs2_traj[:, i]
                # constraints.append(
                #     (x[i, 1] - obs2_x)**2 + (x[i, 5] - obs2_y)**2 - (obs_radius+0.05)**2
                # )
            return np.array(constraints)
        def input_sat_cons(z):
            u = z[: (N - 1) * nu].reshape(N - 1, nu)
            constraints = []
            for i in range(N-1):
                for j in range(nu):
                    constraints.append(self.umax - u[i,j])
                    constraints.append(u[i,j] - self.umin)
            return np.array(constraints)
        # state constraints like the tilt angle 

        def tilt_angle_constraint(z):
            x = z[(N - 1) * nu :].reshape(N, nx)
            tilt_angle_indices = [0, 4]
            constraints = []
            for i in range(N):  # Iterate over the horizon
                for idx in tilt_angle_indices:  # Apply constraint to both columns
                    constraints.append(self.tilt_angle_limit - x[i, idx] )
                    constraints.append( x[i, idx] + self.tilt_angle_limit)
            return np.array(constraints)

        # Combine constraints

        constraints = [
            {"type": "eq", "fun": dynamic_constraint},
            {"type": "eq", "fun": initial_constraint},
            # {"type": "ineq", "fun": obstacle_avoidance_constraint, "args": (obs1_traj, obs2_traj)},
            {"type": "ineq", "fun": input_sat_cons},
            {"type": "ineq", "fun": tilt_angle_constraint},
        ]

        # Solve optimization problem
        result = minimize(cost_function, z0, bounds=bounds, constraints=constraints,options={'maxiter': 1000, 'disp': True})
        if not result.success:
            print("Optimization failed:", result.message)

        # Extract optimal control inputs and states
        u_opt = result.x[: (N - 1) * nu].reshape(N - 1, nu)
        x_opt = result.x[(N - 1) * nu :].reshape(N, nx)

        x_current = x_opt[1,:]
        u_current = u_opt[0,:]

        return x_current, u_current



Q = np.diag([10, 50, 5, 5, 10, 50, 5, 5])
R = np.diag([5,5])
Qf = np.diag([8, 5, 8, 10, 8, 5, 8, 10])
# x_goal = np.array([0, 1.5, 0, 0, 0, 2, 0, 0])
x_goal = np.array([0, 1.2, 0, 0, 0, 2, 0, 0])
x_current = np.zeros((8, 1)).flatten()
nx = 8
nu = 2
u_min = -5
u_max = 5
tolerance =0.01

u_current = np.zeros((nu))

ballbot_mpc = BallbotMPC_DO(Q, R, Qf, nx, nu, u_min, u_max)

iteration = 0 
x_pos = [0]
y_pos = [0]

t_tot = 10

obs1_traj, obs2_traj = ballbot_mpc.dynamic_obs_traj(t_tot)

obs_radius = 0.2

error = 5

robot_trajectory = [(0, 0)]
u1_list = [0]
u2_list = [0]
thetay_positions = [0]
thetax_positions = [0]
obs_look_ahead = 0

x_current_ref = np.array([0, x_current[1], 0, 0, 0, x_current[5], 0, 0])

# x_ref_seg = np.linspace(x_current.flatten(), x_goal, 20)

while error >= tolerance:
# while iteration < 19:
    obs1_traj_slice = obs1_traj[:,iteration+obs_look_ahead:iteration + ballbot_mpc.N+obs_look_ahead]
    obs2_traj_slice = obs2_traj[:,iteration+obs_look_ahead:iteration+ ballbot_mpc.N+obs_look_ahead]
    # x_ref = np.linspace(x_ref_seg[iteration], x_ref_seg[iteration+1],ballbot_mpc.N)

    x_current, u_current = ballbot_mpc.compute_mpc(x_current, u_current, x_goal,obs_radius,obs1_traj_slice, obs2_traj_slice)
    x_pos.append(x_current[1])
    y_pos.append(x_current[5])
    u1_list.append(u_current[0])
    u2_list.append(u_current[1])
    thetay_positions.append(x_current[0])
    thetax_positions.append(x_current[4]) 

    robot_trajectory.append((x_current[1], x_current[5]))

    x_current = x_current.flatten()
    x_pos_error = x_current[1] - x_goal[1]
    y_pos_error = x_current[5] - x_goal[5]
    error = np.linalg.norm([x_pos_error, y_pos_error])

    iteration += 1
    print(iteration)

print("theta_y_pos",thetay_positions)
print("theta_x_pos",thetax_positions)
print("x_pos",x_pos)
print("y_pos",y_pos)
print("u1_list",u1_list)
print("u2_list",u2_list)



plt.figure(figsize=(8, 8))
plt.xlim(-1, 2.5)  # Set x-axis limits (min, max)
plt.ylim(-1, 2.5)  # Set y-axis limits (min, max)

# Plot trajectory
plt.plot(x_pos, y_pos, 'bo-', label='Trajectory')
# Mark the point at [1.2, 2] with a red dot
plt.plot(1.2, 2, 'ro', label='Point [1.2, 2]')

# Add labels and title
plt.xlabel("x")
plt.ylabel("y")
plt.title("Ballbot Trajectory")

# Add legend
plt.legend()
# Display the plot
plt.show()




plt.figure(figsize=(10, 10))
plt.plot(thetay_positions, thetax_positions, 'bo-', label='Trajectory')  
plt.xlabel("theta_x")
plt.ylabel("theta_y")
plt.title("Ballbot Tilt Angle")
plt.show()


fig, ax = plt.subplots()
ax.set_xlim(-1, 3)
ax.set_ylim(-1, 3)
ax.set_aspect('equal')
ax.set_title("Dynamic Obstacles and Robot Trajectory")

# Add dynamic obstacles
obs1_circle = plt.Circle((0, 0), obs_radius, color='blue', alpha=0.5, label="Obstacle 1")
obs2_circle = plt.Circle((0, 0), obs_radius, color='red', alpha=0.5, label="Obstacle 2")
ax.add_artist(obs1_circle)
ax.add_artist(obs2_circle)

# Add robot trajectory
robot_line, = ax.plot([], [], 'g-', label="Robot Trajectory")
goal_x, goal_y = 1.2, 2
ax.plot(goal_x, goal_y, 'ro', label="Goal Point") 

# Update function for animation
def update(frame):
    # Update dynamic obstacles
    obs1_circle.center = (obs1_traj[0, frame], obs1_traj[1, frame])
    obs2_circle.center = (obs2_traj[0, frame], obs2_traj[1, frame])

    # Update robot trajectory
    if frame < len(robot_trajectory):
        robot_line.set_data(
            [pos[0] for pos in robot_trajectory[:frame + 1]],
            [pos[1] for pos in robot_trajectory[:frame + 1]]
        )
    return obs1_circle, obs2_circle, robot_line

# Synchronize animation with time step
logical_time_step = 0.1  # MPC and obstacle time step
real_time_interval_ms = logical_time_step * 4000  # Convert seconds to milliseconds

# Create animation
ani = FuncAnimation(fig, update, frames=len(x_pos), interval=real_time_interval_ms, blit=True)

# Display the animation
plt.legend()
plt.show()
