import sympy as sp

# https://www.youtube.com/watch?v=5qJY-ZaKSic&t=320s&ab_channel=BriannoColler

# state = sp.Matrix(sp.symarray('state', 4))

m_c, m_p, L, g = sp.symbols("m_c, m_p, L, g")
parameters = sp.Matrix([m_c, m_p, L, g])

x, theta, x_dot, theta_dot, x_ddot, theta_ddot = sp.symbols("x, theta, x_dot, theta_dot, x_ddot, theta_ddot")
state = sp.Matrix([x, theta, x_dot, theta_dot])

F = sp.symbols("F")
inputs = sp.Matrix([F])

sin = sp.sin
cos = sp.cos

# -m_p * g * sin(theta) == m_p*x_ddot*cos(theta) - m_p * L * theta_ddot
# F + m_p*L*theta_ddot*cos(theta) - m_p * L * theta_dot * theta_dot * sin(theta) == (m_c + m_p) * x_ddot

expr = sp.Matrix([
    m_p * g * sin(theta) + m_p*x_ddot*cos(theta) - m_p * L * theta_ddot,
    F + m_p*L*theta_ddot*cos(theta) - m_p * L * theta_dot * theta_dot * sin(theta) - (m_c + m_p) * x_ddot
])

res = sp.solve(expr, (x_ddot, theta_ddot))

print(res)

state_dot = sp.Matrix([x_dot, theta_dot, res[x_ddot], res[theta_ddot]])


get_model_lambda = sp.lambdify((state, parameters, inputs), state_dot, 'sympy')
print(sp.printing.cxxcode(state_dot, assign_to="res", standard="C++11"))

from sympy.utilities.codegen import codegen

[(c_name, c_code), (h_name, c_header)] = \
    codegen(('state_dot', state_dot), "C99", "test", header=False, empty=False)

print(c_code)

