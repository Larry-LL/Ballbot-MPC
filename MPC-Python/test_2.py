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
        self.N = 10  # Prediction horizon
        self.T = 0.1  # Time step

    def dynamic_obs_traj(self,t_tot, num_steps = 50 ):
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
    
    def compute_mpc(self, x_current, u_current, x_goal, t_tot,num_steps,obs_radius=0.2):

        # Prediction horizon and dimensions
        N = self.N
        nx = self.nx
        nu = self.nu
        
        obs1_traj, obs2_traj = self.dynamic_obs_traj(t_tot, num_steps)

        # Initial guess for control inputs and states
        # u0 = np.vstack((u_current, np.zeros((N - 2, nu))))
        

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

            cost = 0
            for i in range(N - 1):
                cost += (x[i] - x_goal).T @ self.Q @ (x[i] - x_goal) + u[i].T @ self.R @ u[i]
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

        def obstacle_avoidance_constraint(z):
            x = z[(N - 1) * nu :].reshape(N, nx)
            constraints = []
            for i in range(N):
                # Obstacle 1 avoidance
                obs1_x, obs1_y = obs1_traj[:, i]
                constraints.append(
                    (x[i, 0] - obs1_x)**2 + (x[i, 1] - obs1_y)**2 - obs_radius**2
                )
                # Obstacle 2 avoidance
                obs2_x, obs2_y = obs2_traj[:, i]
                constraints.append(
                    (x[i, 0] - obs2_x)**2 + (x[i, 1] - obs2_y)**2 - obs_radius**2
                )
            return np.array(constraints)

        # Combine constraints
        constraints = [
            {"type": "eq", "fun": dynamic_constraint},
            {"type": "eq", "fun": initial_constraint},
            {"type": "ineq", "fun": obstacle_avoidance_constraint},
        ]

        # Solve optimization problem
        result = minimize(cost_function, z0, bounds=bounds, constraints=constraints)

        # Extract optimal control inputs and states
        u_opt = result.x[: (N - 1) * nu].reshape(N - 1, nu)
        x_opt = result.x[(N - 1) * nu :].reshape(N, nx)
        print(x_opt.shape)
        print(u_opt.shape)

        x_current = x_opt[1,:]
        u_current = u_opt[0,:]

        return x_current, u_current

        

Q = np.diag([100, 100, 100, 100,1000, 1000, 1000, 100])
R = np.diag([5,5])
Qf = np.diag([8, 1000, 8, 10, 8, 1000, 8, 10])
x_goal = np.array([0, 3, 0, 0, 0, 2, 0, 0])
x_current = np.zeros((8, 1))
nx = 8
nu = 2
u_min = -4.9
u_max = 4.9
tolerance =0.2
t_tot = 5
u_current = np.zeros((nu))

ballbot_mpc = BallbotMPC_DO(Q, R, Qf, nx, nu, u_min, u_max)


t_tot = 5
num_steps = 100
iteration = 0 
x_pos = []
y_pos = []
print("processing")

error = 5

while error >= tolerance:
# while iteration <= 30:
    x_current, u_current = ballbot_mpc.compute_mpc(x_current, u_current, x_goal, t_tot,num_steps)
    x_pos.append(x_current[1])
    y_pos.append(x_current[5])
    x_pos_error = x_current[1] - x_goal[1]
    y_pos_error = x_current[5] - x_goal[5]
    error = np.linalg.norm([x_pos_error, y_pos_error])

    iteration += 1
    print(iteration)
    print(error)


plt.figure(figsize=(8, 6))
plt.plot(x_pos, y_pos, marker='o', label="Trajectory")

# Labels and title
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.title("Trajectory of Ballbot")
plt.legend()
plt.grid()

# Show the plot
plt.show()
