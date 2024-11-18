import numpy as np
import cvxpy as cp
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt



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
            cost += cp.quad_form((x_desired-x[i, :]), self.Q) + cp.quad_form(u[i, :], self.R)
        cost += cp.quad_form((x_desired - x[self.N - 1, :]), self.Qf)  # Terminal cost

        # Solve the optimization problem
        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve()
  

        u_current = u[0, :].value
        u_current = u_current.reshape(-1, 1)

        if u_current is not None:
            x_current = Ad @ x_current + Bd @ u_current
        else:
            u_current = np.zeros((self.nu, 1))

        return x_current, u_current

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
while np.linalg.norm(x_current.flatten() - x_desired) > tolerance:
    x_current, u_current = ballbot_mpc.compute_mpc(x_current, u_current, x_desired)
    iteration +=1
    x_positions.append(x_current[1, 0])  # x position (second state)
    y_positions.append(x_current[5, 0])  
    thetay_positions.append(x_current[0,0])
    thetax_positions.append(x_current[4,0]) 
    print(thetax_positions)
    print(iteration)
print("x_current:", x_current.flatten())
print("Total iteration time equals",iteration)

##visualization
plt.figure(figsize=(8, 8))
plt.plot(x_positions, y_positions, 'bo-', label='Trajectory')  
plt.scatter(x_desired[1], x_desired[5], color='red', marker='o', label='Goal Position (1,1)')  

# Add labels and title
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

# Add labels and title for the second plot
plt.xlabel("Theta X Position")
plt.ylabel("Theta Y Position")
plt.title("Trajectory of Theta X and Theta Y Positions in MPC")
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()
print(max(thetax_positions))
print(max(thetay_positions))