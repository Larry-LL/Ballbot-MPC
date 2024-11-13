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

        theta_x = x_current[0]
        theta_x_dot = x_current[1]
        T_YZ = u_current[0]

        theta_y = x_current[4]
        theta_y_dot = x_current[5]
        T_XZ = u_current[1]


        # Compute each element of the matrix YZ_A
        A_cont = np.array([
            [0, 0, 1.0, 0],
            [0, 0, 0, 1.0],
            [(4.2e26 * np.cos(theta_x) + 7.7e25 * T_YZ * np.sin(theta_x) + 6.9e25 * np.cos(theta_x)**3 
            + 4.9e24 * theta_x_dot**2 - 9.1e24 * theta_x_dot**2 * np.cos(theta_x)**2 
            + 9.6e24 * T_YZ * np.cos(theta_x)**2 * np.sin(theta_x)) / (7.8e11 * np.cos(theta_x)**2 - 6.3e12)**2, 
            0, 
            (4.0e-3 * theta_x_dot * np.sin(2.0 * theta_x)) / (2.0e-3 * np.cos(2.0 * theta_x) - 0.03), 
            0],
            [(0.5 * (5.0e25 * theta_x_dot**2 * np.cos(theta_x) - 1.8e27 * np.cos(theta_x)**2 
                    - 1.3e26 * T_YZ * np.sin(2.0 * theta_x) + 8.2e24 * theta_x_dot**2 * np.cos(theta_x)**3 
                    + 9.4e26)) / (7.8e11 * np.cos(theta_x)**2 - 6.3e12)**2, 
            0, 
            (1.1e13 * theta_x_dot * np.sin(theta_x)) / (7.8e11 * np.sin(theta_x)**2 + 5.5e12), 
            0]
        ])

        A_cont_upper = np.hstack((A_cont, np.zeros((4, 4))))
        A_cont_lower = np.hstack((np.zeros((4, 4)), A_cont))

        A_cont = np.vstack((A_cont_upper,A_cont_lower))

        B_cont = np.array([  
            [0],
            [0],
            [(0.063*np.cos(theta_x))/(4.0e-3*np.cos(theta_x)**2 - 0.032)],
            [-0.43/(4.0e-3*np.cos(theta_x)**2 - 0.032)]
        ])

        B_cont_left = np.vstack((B_cont,np.zeros((4,1))))
        B_cont_right = np.vstack((np.zeros((4,1)), B_cont))

        B_cont = np.hstack((B_cont_left,B_cont_right))

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
        A_cont, B_cont = self.calcaulte_continous_dynamics(x_current,u_current)  # 1st loop x_current & u_current will be zeros as setup in subroutine
        Ad, Bd = self.discretize_system(A_cont, B_cont, self.T)     #A_cont and B_cont 

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

        u_current = u[0, :].value

        # Update x_current using the continuous dynamics and integration
        # if u_current is not None:
        #     A_cont, B_cont = self.calcaulte_continous_dynamics(x_current, u_current)
        #     sol = solve_ivp(
        #         lambda t, x: self.continuous_dynamics(x, u_current, A_cont, B_cont),
        #         [0, self.T],
        #         x_current,
        #         method='RK45'
        #     )
        #     x_current = sol.y[:, -1]  # Take the final state after time T

        if u_current is not None:
            x_current = Ad @ x_current + Bd @ u_current

        return x_current, u_current

# Example instantiation and usage
Q = np.diag([8000, 0.001, 2000, 0.001,8000, 0.001, 2000, 0.001])
R = np.diag([40,40])
Qf = np.diag([8000, 0.001, 2000, 0.001,8000, 0.001, 2000, 0.001])
x_desired = [0,10,0,2,0,10,0,2]
nx = 8
nu = 2
u_min = 0
u_max = 4.9

ballbot_mpc = BallbotMPC(Q, R, Qf, nx, nu, u_min, u_max)

# # Initial state
# x_current = [0,0]      # Initial state, assuming starting from zero
# u_current = 0      # Initial control input, also starting from zero
# x_desired = np.array([1, 0, 0, 0, 0, 0, 0, 0])  # Example target state
# # Run the MPC loop
# for _ in range(20):  # Run for 20 steps as an example
#     u_mpc, x_current = ballbot_mpc.compute_mpc(x_current, u_current, x_desired)
#     print("Control input:", u_mpc)
#     print("Updated state:", x_current)

#     # Update u_current for the next iteration
#     u_current = u_mpc

x_current = np.zeros(8)
u_current = np.zeros(2)
A_cont, B_cont = ballbot_mpc.calcaulte_continous_dynamics(x_current, u_current)
print(A_cont)
print(B_cont)
for _ in range(20):  # Run for 20 steps as an example
    x_current, u_current = ballbot_mpc.compute_mpc(x_current, u_current, x_desired)
    print("Control input:", u_current)
    print("Updated state:", x_current)
    
