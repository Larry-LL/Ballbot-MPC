import numpy as np
from scipy.optimize import minimize

# Define parameters
N = 10  # Prediction horizon
dt = 0.1  # Time step
state_dim = 2  # State dimension
control_dim = 1  # Control dimension

# System dynamics: x[k+1] = A*x[k] + B*u[k]
A = np.array([[1, dt], [0, 1]])
B = np.array([[0], [dt]])

# Initial state
x0 = np.array([0, 0])

# Desired state
xd = np.array([1, 0])

# Control input bounds
u_min, u_max = -1, 1

# Quadratic cost weights
Q = np.eye(state_dim)  # State cost
R = 0.01 * np.eye(control_dim)  # Control input cost

# Flattened optimization variables: [u[0], u[1], ..., u[N-1], x[0], x[1], ..., x[N]]
def unpack_variables(z):
    u = z[:N*control_dim].reshape(N, control_dim)
    x = z[N*control_dim:].reshape(N+1, state_dim)
    return u, x

def cost_function(z):
    u, x = unpack_variables(z)
    cost = 0
    for k in range(N):
        cost += np.dot((x[k] - xd), Q @ (x[k] - xd)) + np.dot(u[k], R @ u[k])
    return cost

def dynamics_constraints(z):
    u, x = unpack_variables(z)
    constraints = []
    for k in range(N):
        constraints.append(x[k+1] - (A @ x[k] + B @ u[k]))
    return np.concatenate(constraints)

# Optimization bounds (for u and x)
u_bounds = [(u_min, u_max)] * (N * control_dim)
x_bounds = [(None, None)] * ((N+1) * state_dim)

# Initial guess
z0 = np.zeros((N * control_dim + (N+1) * state_dim,))

# Equality constraints (dynamics + initial state)
eq_constraints = [{'type': 'eq', 'fun': lambda z: dynamics_constraints(z)},
                  {'type': 'eq', 'fun': lambda z: unpack_variables(z)[1][0] - x0}]

# Optimization
result = minimize(cost_function, z0, bounds=u_bounds + x_bounds, constraints=eq_constraints)

# Extract results
u_opt, x_opt = unpack_variables(result.x)

print("Optimal Control Inputs:", u_opt)
print("Optimal State Trajectory:", x_opt)
