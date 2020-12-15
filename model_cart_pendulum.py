import sympy as sp

# https://www.youtube.com/watch?v=5qJY-ZaKSic&t=320s&ab_channel=BriannoColler

# state =

parameters = sp.Matrix(sp.symbols("m_c, m_p, L, g"))
# parameters = sp.MatrixSymbol('parameters', 4, 1)
m_c, m_p, L, g = parameters

x, theta, x_dot, theta_dot, x_ddot, theta_ddot = sp.symbols("x, theta, x_dot, theta_dot, x_ddot, theta_ddot")
state = sp.Matrix([x, theta, x_dot, theta_dot])

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
state_ctrl_dot = sp.Matrix([theta_dot, res[x_ddot], res[theta_ddot]])

model_sim = sp.lambdify((state, t_sym, parameters, inputs), state_sim_dot, 'numpy')
model_ctrl = sp.lambdify((state, t_sym, parameters, inputs), state_ctrl_dot, 'numpy')

print(state_sim_dot.__repr__())
print(state_ctrl_dot.__repr__())


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
