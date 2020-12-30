import sympy as sp

# https://www.youtube.com/watch?v=5qJY-ZaKSic&t=320s&ab_channel=BriannoColler

parameters = sp.Matrix(sp.symbols("m_c, m_p, L, g"))
# parameters = sp.MatrixSymbol('parameters', 4, 1)
m_c, m_p, L, g = parameters

x, theta, x_dot, theta_dot, x_ddot, theta_ddot = sp.symbols("x, theta, x_dot, theta_dot, x_ddot, theta_ddot")
state_sim = sp.Matrix([x, theta, x_dot, theta_dot])
state_ctrl = sp.Matrix([theta, x_dot, theta_dot])
state_ctrl_2 = sp.Matrix([theta, theta_dot])

F = sp.symbols("F")
inputs = sp.Matrix([F])

t_sym = sp.symbols("t_sym")

sin = sp.sin
cos = sp.cos

# -m_p * g * sin(theta) == m_p*x_ddot*cos(theta) - m_p * L * theta_ddot
# F + m_p*L*theta_ddot*cos(theta) - m_p * L * theta_dot * theta_dot * sin(theta) == (m_c + m_p) * x_ddot

expr = sp.Matrix([
    m_p * g * sin(theta) + m_p*x_ddot*cos(theta) - m_p * L * theta_ddot,
    F + m_p*L*theta_ddot*cos(theta) - m_p * L * theta_dot * theta_dot * sin(theta) - (m_c + m_p) * x_ddot
])

res = sp.solve(expr, (x_ddot, theta_ddot))

state_sim_dot = sp.Array([x_dot, theta_dot, res[x_ddot], res[theta_ddot]])
state_ctrl_dot = sp.Array([theta_dot, res[x_ddot], res[theta_ddot]])
state_ctrl_2_dot = sp.Array([theta_dot, res[theta_ddot]])

model_sim_lnp = sp.lambdify((state_sim, t_sym, parameters, inputs), state_sim_dot, 'numpy')
model_ctrl_lnp = sp.lambdify((state_ctrl, t_sym, parameters, inputs), state_ctrl_dot, 'numpy')
model_ctrl_2_lnp = sp.lambdify((state_ctrl_2, t_sym, parameters, inputs), state_ctrl_2_dot, 'numpy')

print(state_sim_dot.__repr__())
print(state_ctrl_dot.__repr__())
print(state_ctrl_2_dot.__repr__())

stat_point_ctrl_2 = sp.solve(sp.Matrix(state_ctrl_2_dot))

# %%

import numpy as np

d_model_d_state_sim = sp.lambdify((state_sim, t_sym, parameters, inputs), sp.Matrix(state_sim_dot).jacobian(state_sim), 'sympy')
d_model_d_state_ctrl = sp.lambdify((state_ctrl, t_sym, parameters, inputs), sp.Matrix(state_ctrl_dot).jacobian(state_ctrl), 'sympy')
d_model_d_state_ctrl_2 = sp.lambdify((state_ctrl_2, t_sym, parameters, inputs), sp.Matrix(state_ctrl_2_dot).jacobian(state_ctrl_2), 'sympy')

d_model_d_inputs_sim = sp.lambdify((state_sim, t_sym, parameters, inputs), sp.Matrix(state_sim_dot).jacobian(inputs), 'sympy')
d_model_d_inputs_ctrl = sp.lambdify((state_ctrl, t_sym, parameters, inputs), sp.Matrix(state_ctrl_dot).jacobian(inputs), 'sympy')
d_model_d_inputs_ctrl_2 = sp.lambdify((state_ctrl_2, t_sym, parameters, inputs), sp.Matrix(state_ctrl_2_dot).jacobian(inputs), 'sympy')

y0 = sp.Array([
    0.0,
    0.0,
    0.0,
    0.0
])

u0 = sp.Array([
    F,
])

params = parameters

# params = sp.Array([
#     0.3,
#     1.0,
#     1.0,
#     9.81,
# ])

t_s = sp.Array([0.0])

A_sim = d_model_d_state_sim(y0, t_s, params, u0)
B_sim = d_model_d_inputs_sim(y0, t_s, params, u0)

A_ctrl = d_model_d_state_ctrl(y0[1:], t_s, params, u0)
B_ctrl = d_model_d_inputs_ctrl(y0[1:], t_s, params, u0)


A_ctrl_2 = d_model_d_state_ctrl_2([0, theta], t_s, params, sp.Array([stat_point_ctrl_2[0][F]]))
B_ctrl_2 = d_model_d_inputs_ctrl_2([0, theta], t_s, params, sp.Array([stat_point_ctrl_2[0][F]]))

# A_ctrl_2 = d_model_d_state_ctrl_2(y0[1:], t_s, params, u0)
# B_ctrl_2 = d_model_d_inputs_ctrl_2(y0[1:], t_s, params, u0)

print(A_ctrl.__repr__())
print(B_ctrl.__repr__())

A_sim_lnp = sp.lambdify((parameters,), A_sim, 'numpy')
B_sim_lnp = sp.lambdify((parameters,), B_sim, 'numpy')

A_ctrl_lnp = sp.lambdify((parameters,), A_ctrl, 'numpy')
B_ctrl_lnp = sp.lambdify((parameters,), B_ctrl, 'numpy')


# np.linalg.eig(np.array(A_sim).astype(np.float64))
# np.linalg.eig(np.array(A_ctrl).astype(np.float64))



# %% Codegen stuff starts here:
# https://www.sympy.org/scipy-2017-codegen-tutorial/notebooks/07-the-hard-way.html

# print(sp.printing.cxxcode(state_dot, assign_to="res", standard="C++11"))

# %%

# from sympy.printing.codeprinter import Assignment
# from sympy.printing.ccode import C99CodePrinter
# from sympy.printing.cxxcode import CXX17CodePrinter
#
# class CMatrixPrinter(CXX17CodePrinter):
#     def _print_ImmutableDenseMatrix(self, expr):
#         sub_exprs, simplified = sp.cse(expr)
#         lines = []
#         for var, sub_expr in sub_exprs:
#             lines.append('double ' + self._print(Assignment(var, sub_expr)))
#         M = sp.MatrixSymbol('M', *expr.shape)
#         return '\n'.join(lines) + '\n' + self._print(Assignment(M, simplified[0]))
#
#
# p = CMatrixPrinter()
# print(p.doprint(state_dot))

# %%


# from sympy.utilities.codegen import codegen
# from sympy.codegen.rewriting import optimize, optims_c99, create_expand_pow_optimization
#
# expand_opt = create_expand_pow_optimization(3)
#
# expand_opt(res[x_ddot])

# [(c_name, c_code), (h_name, c_header)] = \
#     codegen(('state_dot', state_dot), "C99", "test", header=False, empty=True)
#
# print("", c_code)
