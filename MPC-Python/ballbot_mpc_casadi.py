import numpy as np
import cvxpy as cp
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import casadi as ca


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

    def calcaulte_continous_dynamics(self, x_current, u_current):
        # Reshape u_current to be a column vector (CasADi reshape)
        u_current = ca.reshape(u_current, u_current.size1(), 1)

        # Extract state variables
        theta_x = x_current[0, 0]
        theta_x_dot = x_current[1, 0]
        T_YZ = u_current[0, 0]

        theta_y = x_current[4, 0]
        theta_y_dot = x_current[5, 0]
        T_XZ = u_current[1, 0]
        epsilon = 1e-8  # Small number to avoid division by zero

        # Calculate the continuous dynamics for theta_x
        A_cont_yz = ca.vertcat(
            ca.horzcat(0, 0, 1.0, 0),
            ca.horzcat(0, 0, 0, 1.0),
            ca.horzcat(
                (2.537e35 * ca.cos(theta_x) + 5.494e34 * T_YZ * ca.sin(theta_x) + 6.468e34 * ca.cos(theta_x)**3 +
                3.664e33 * theta_x_dot**2 - 6.71e33 * theta_x_dot**2 * ca.cos(theta_x)**2 +
                9.276e33 * T_YZ * ca.cos(theta_x)**2 * ca.sin(theta_x)) /
                (6.187e32 * ca.cos(theta_x)**4 - 7.329e33 * ca.cos(theta_x)**2 + 2.17e34 + epsilon),
                0,
                (4.4e-3 * theta_x_dot * ca.sin(2.0 * theta_x)) /
                (2.2e-3 * ca.cos(2.0 * theta_x) - 0.024 + epsilon),
                0
            ),
            ca.horzcat(
                (2.655e34 * theta_x_dot**2 * ca.cos(theta_x) - 1.29e36 * ca.cos(theta_x)**2 - 105.0 * ca.cos(theta_x)**4 -
                1.015e35 * T_YZ * ca.sin(2.0 * theta_x) + 6.769e33 * theta_x_dot**2 * ca.cos(theta_x)**3 +
                7.076e35) /
                (1.237e33 * ca.cos(theta_x)**4 - 1.466e34 * ca.cos(theta_x)**2 + 4.341e34 + epsilon),
                0,
                (1.188e15 * theta_x_dot * ca.sin(theta_x)) /
                (1.086e14 * ca.sin(theta_x)**2 + 5.348e14 + epsilon),
                0
            )
        )

        # Calculate the continuous dynamics for theta_y
        A_cont_xz = ca.vertcat(
            ca.horzcat(0, 0, 1.0, 0),
            ca.horzcat(0, 0, 0, 1.0),
            ca.horzcat(
                (2.537e35 * ca.cos(theta_y) + 5.494e34 * T_XZ * ca.sin(theta_y) + 6.468e34 * ca.cos(theta_y)**3 +
                3.664e33 * theta_y_dot**2 - 6.71e33 * theta_y_dot**2 * ca.cos(theta_y)**2 +
                9.276e33 * T_XZ * ca.cos(theta_y)**2 * ca.sin(theta_y)) /
                (6.187e32 * ca.cos(theta_y)**4 - 7.329e33 * ca.cos(theta_y)**2 + 2.17e34 + epsilon),
                0,
                (4.4e-3 * theta_y_dot * ca.sin(2.0 * theta_y)) /
                (2.2e-3 * ca.cos(2.0 * theta_y) - 0.024 + epsilon),
                0
            ),
            ca.horzcat(
                (2.655e34 * theta_y_dot**2 * ca.cos(theta_y) - 1.29e36 * ca.cos(theta_y)**2 - 105.0 * ca.cos(theta_y)**4 -
                1.015e35 * T_XZ * ca.sin(2.0 * theta_y) + 6.769e33 * theta_y_dot**2 * ca.cos(theta_y)**3 +
                7.076e35) /
                (1.237e33 * ca.cos(theta_y)**4 - 1.466e34 * ca.cos(theta_y)**2 + 4.341e34 + epsilon),
                0,
                (1.188e15 * theta_y_dot * ca.sin(theta_y)) /
                (1.086e14 * ca.sin(theta_y)**2 + 5.348e14 + epsilon),
                0
            )
        )

        # Assemble A_cont
        A_cont_upper = ca.horzcat(A_cont_yz, ca.MX.zeros((4, 4)))
        A_cont_lower = ca.horzcat(ca.MX.zeros((4, 4)), A_cont_xz)
        A_cont = ca.vertcat(A_cont_upper, A_cont_lower)

        # Calculate B_cont
        B_cont_yz = ca.vertcat(
            0,
            0,
            (0.0667 * ca.cos(theta_x)) / (0.00445 * ca.cos(theta_x)**2 - 0.0264),
            -0.365 / (0.00445 * ca.cos(theta_x)**2 - 0.0264)
        )
        B_cont_xz = ca.vertcat(
            0,
            0,
            (0.0667 * ca.cos(theta_y)) / (0.00445 * ca.cos(theta_y)**2 - 0.0264),
            -0.365 / (0.00445 * ca.cos(theta_y)**2 - 0.0264)
        )

        B_cont_left = ca.vertcat(B_cont_yz, ca.MX.zeros((4, 1)))
        B_cont_right = ca.vertcat(ca.MX.zeros((4, 1)), B_cont_xz)
        B_cont = ca.horzcat(B_cont_left, B_cont_right)

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

    def compute_mpc(self, x_current, u_current, x_desired): #our main mpc computational function 
        # Define decision variables
        x = ca.MX.sym("x", self.nx, self.N)  # State variables (n_x x N)
        u = ca.MX.sym("u", self.nu, self.N - 1)  # Co

        constraints = []
        lbg = []  # Lower bounds
        ubg = []  # Upper bounds
        # Initial state equality constraint
        constraints.append(x[:, 0] - x_current)
        lbg.extend([0] * self.nx)
        ubg.extend([0] * self.nx)

        
        # Input saturation constraints
        for i in range(self.N - 1):
            constraints.append(u[:, i] - self.umin)  # u[i, :] >= umin -> g(u) = u[i, :] - umin >= 0
            lbg.extend([0] * self.nu)  # Lower bound: u[i, :] - umin >= 0
            ubg.extend([float("inf")] * self.nu)
            #start here
            constraints.append(self.umax - u[:, i])  # u[i, :] <= umax -> g(u) = umax - u[i, :] >= 0
            lbg.extend([0] * self.nu)  # Lower bound: u[i, :] - umin >= 0
            ubg.extend([float("inf")] * self.nu)

        # Discretized dynamics
        u_current = ca.DM(u_current)
        A_cont, B_cont = self.calcaulte_continous_dynamics(x_current,u_current)  # 1st loop x_current & u_current will be zeros as setup in subroutine
        Ad, Bd = self.discretize_system(A_cont, B_cont, self.T)     #A_cont and B_cont 


        # Dynamics constraints
        for i in range(self.N - 1):
            constraints.append(x[:, i + 1] - (Ad @ x[:, i] + Bd @ u[:, i]))
            # Equality bounds for this constraint
            lbg.extend([0] * self.nx)  # Lower bound = 0 for all state variables
            ubg.extend([0] * self.nx)

        # add obstacle constraints
        obstacle_center = np.array([0.2,0.5])
        obstacle_radius = 0.1
    
    # Choose a convex region to the left of the obstacle
        dist_to_obstacle = (x[1, i] - obstacle_center[0])**2 + (x[5, i] - obstacle_center[1])**2
        constraints.append(dist_to_obstacle)
        lbg.append(obstacle_radius**2)  # Minimum distance squared
        ubg.append(float("inf"))    

        # Cost function
        
        cost = 0

        # add cost
        for i in range(self.N - 1):
            # State tracking cost
            cost += ca.mtimes([(x[:, i] - x_desired).T, self.Q, (x[:, i] - x_desired)])
            # Control effort cost
            cost += ca.mtimes([u[:, i].T, self.R, u[:, i]])

        # Terminal cost
        cost += ca.mtimes([(x[:, self.N - 1] - x_desired).T, self.Qf, (x[:, self.N - 1] - x_desired)])


                # Solve the optimization problem
        nlp = {
            "x": ca.vertcat(ca.reshape(x, -1, 1), ca.reshape(u, -1, 1)),  # Optimization variables
            "f": cost,  # Objective function
            "g": ca.vertcat(*constraints)  # Constraints
        }

        # Solver options
        opts = {
            "ipopt.print_level": 0,
            "print_time": 0,
            "ipopt.tol": 1e-4,
        }

        # Create the solver
        solver = ca.nlpsol("solver", "ipopt", nlp, opts)

        # Solve the optimization problem
        solution = solver(lbg=lbg, ubg=ubg)

        # Extract optimal control and state trajectories
        opt_x = solution["x"].full()  # Convert to numeric (NumPy array)
        x_traj = opt_x[: self.nx * self.N].reshape(self.nx, self.N)  # Reshape to (nx x N)
        u_traj = opt_x[self.nx * self.N :].reshape(self.nu, self.N - 1)  # Reshape to (nu x N-1)

        # Extract the first control input
        u_current = u_traj[:, 0]

        # Propagate the state forward using the first control input
        x_current = Ad @ x_current + Bd @ u_current
        return x_current, u_current

# Example instantiation and usage
Q = np.diag([100, 100, 100, 100,1000, 1000, 1000, 100])
R = np.diag([5,5])
Qf = np.diag([8, 50, 8, 10,8, 50, 8, 10])
x_desired = np.array([0, 1, 0, 0, 0, 1, 0, 0])
nx = 8
nu = 2
u_min = -4.9
u_max = 4.9
tolerance =0.01

ballbot_mpc = BallbotMPC(Q, R, Qf, nx, nu, u_min, u_max)

x_current = np.zeros((8,1))
u_current = np.zeros((nu))
x_positions = []
y_positions = []
thetay_positions = []
thetax_positions = []

iteration = 0
while True:
    # Solve MPC problem
    x_next, u_next = ballbot_mpc.compute_mpc(x_current, u_current, x_desired)

    # Convert x_next and x_desired to numeric (if necessary)
    x_next_numeric = ca.DM(x_next) if isinstance(x_next, ca.MX) else x_next
    x_desired_numeric = ca.DM(x_desired) if isinstance(x_desired, ca.MX) else x_desired

    # Compute the Frobenius norm as a numeric value
    norm_value = float(ca.norm_fro(x_next_numeric - x_desired_numeric))

    # Check if the norm is within tolerance
    if norm_value <= tolerance:
        break

    # Update state and control input
    x_current = x_next_numeric
    u_current = u_next



