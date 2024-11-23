from sympy import symbols, integrate, sqrt

# Define symbols
x, y, z, a, b, m = symbols('x y z a b m')

# Volume of the solid
z_upper = b - (x**2 + y**2) / a
y_upper = sqrt(a*b - x**2)
x_upper = sqrt(a*b)

# Define density

V = integrate(integrate(integrate(1, (z, 0, z_upper)), (y, 0, y_upper)), (x, 0, x_upper))
rho = m / V
print("here:",V)
# Define integrand for Ixx
integrand_Ixx = rho * (y**2 + z**2)

# Compute Ixx
Ixx = integrate(integrate(integrate(integrand_Ixx, (z, 0, z_upper)), (y, 0, y_upper)), (x, 0, x_upper))
print("done")
print("Ixx:", Ixx.simplify())
