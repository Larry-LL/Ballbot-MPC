import numpy as np
import cvxpy as cp
from scipy.integrate import solve_ivp

class BallbotMPC:
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

    def calcaulte_continous_dynamics(self,x_current,u_current):  # linearize at referenced state x_current & u_current

        A_cont = np.zeros((self.nx,self.nx))
        B_cont = np.zeros((self.nx,self,nu))

        return A_cont, B_cont

    def continuous_dynamics(self, x, u, A_cont, B_cont):
        # Define the continuous-time dynamics for the ballbot here
        xdot = A_cont @ x + B_cont @ u
        return xdot

    def discretize_system(self, A_cont, B_cont, T):
        # Discretize the system (example: using first-order approximation)
        Ad = np.eye(self.nx) + A_cont * T  
        Bd = B_cont * T
        return Ad, Bd

    def compute_mpc(self, x_current, u_current, x_desired):
        # Define decision variables
        x = cp.Variable((self.N, self.nx))   
        u = cp.Variable((self.N - 1, self.nu)) 
        
        constraints = []
        
        # Initial state constraint
        constraints.append(x[0, :] == x_current)
        
        # Input saturation constraints
        for i in range(self.N - 1):
            constraints.append(u[i, :] >= self.umin)
            constraints.append(u[i, :] <= self.umax)
        
        # Discretized dynamics
        A_cont, B_cont = self.calcaulte_continous_dynamics(self,x_current,u_current)  # Placeholder for actual dynamics
        Ad, Bd = self.discretize_system(A_cont, B_cont, self.T)

        # Dynamics constraints
        for i in range(self.N - 1):
            constraints.append(x[i + 1, :] == Ad @ x[i, :] + Bd @ u[i, :])

        # Cost function
        cost = 0
        for i in range(self.N - 1):
            cost += cp.quad_form((x_desired-x[i, :]), self.Q) + cp.quad_form(u[i, :], self.R)
        cost += cp.quad_form((x_desired - x[self.N - 1, :]), self.Qf)  # Terminal cost

        # Solve the optimization problem
        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve()

        u_mpc = u[0, :].value

        # Update x_current using the continuous dynamics and integration
        if u_mpc is not None:
            A_cont, B_cont = self.calcaulte_continous_dynamics(x_current, u_mpc)
            sol = solve_ivp(
                lambda t, x: self.continuous_dynamics(x, u_mpc, A_cont, B_cont),
                [0, self.T],
                x_current,
                method='RK45'
            )
            x_current = sol.y[:, -1]  # Take the final state after time T

        return u_mpc, x_current

# Example instantiation and usage
Q = np.eye(8)
R = np.eye(2)
Qf = np.eye(8)
nx = 8
nu = 2
u_min = 0
u_max = 4.9

ballbot_mpc = BallbotMPC(Q, R, Qf, nx, nu, u_min, u_max)

# Initial state
x_current = np.zeros(nx)      # Initial state, assuming starting from zero
u_current = np.zeros(nu)      # Initial control input, also starting from zero
x_desired = np.array([1, 0, 0, 0, 0, 0, 0, 0])  # Example target state
# Run the MPC loop
for _ in range(20):  # Run for 20 steps as an example
    u_mpc, x_current = ballbot_mpc.compute_mpc(x_current, u_current, x_desired)
    print("Control input:", u_mpc)
    print("Updated state:", x_current)

    # Update u_current for the next iteration
    u_current = u_mpc
