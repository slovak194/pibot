import matplotlib

matplotlib.use('Qt5Agg')
# matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
import numpy as np

import control as ctl

from model_cart_pendulum import model_sim_lnp, A_ctrl_lnp, B_ctrl_lnp


class PendulumVis:

    running = True
    first = True

    def press(self, event):
        if event.key == ' ':
            self.running = False

    def update(self, l_state, l_inp):

        l_x = l_state[0]
        l_theta = l_state[1]
        F = l_inp[0]

        pend = np.array([np.cos(l_theta + np.pi/2), np.sin(l_theta + np.pi/2)])*1.0

        if self.first:

            self.f1 = plt.figure(1)
            self.ax = self.f1.add_subplot(111)

            self.f1.canvas.mpl_connect('key_press_event', self.press)

            self.ax.cla()

            self.cart = self.ax.plot(l_x, 0.0, 'o')
            self.pend = self.ax.plot([l_x, l_x + pend[0]], [0.0, pend[1]], '.-')
            self.force = self.ax.plot([l_x, l_x + F], [0.0, 0.0], '.-')

            self.ax.set_aspect("equal")

            self.ax.set_xlim([-2, 2])
            self.ax.set_ylim([-0.25, 1.25])

            self.ax.grid(True, which="major")
            self.ax.minorticks_on()
            self.ax.grid(True, which="minor", linestyle=':', linewidth=1)

            self.first = False

        else:
            self.cart[0].set_data(l_x, 0.0)
            self.pend[0].set_data([l_x, l_x + pend[0]], [0.0, pend[1]])
            self.force[0].set_data([l_x, l_x + F], [0.0, 0.0])


def rk4step(f, y_i, h, args=()):

    t_i = 0.0  # TODO, OLSLO, model is not time dependent!

    k1 = np.array(f(y_i, t_i, *args))
    k2 = np.array(f(y_i + k1 * h / 2., t_i + h / 2., *args))
    k3 = np.array(f(y_i + k2 * h / 2., t_i + h / 2., *args))
    k4 = np.array(f(y_i + k3 * h, t_i + h, *args))
    y_ip1 = y_i + (h / 6.) * (k1 + 2*k2 + 2*k3 + k4)
    return y_ip1

# %%

wheel_r = 0.035





#
# double k_p = 0.3;
# double k_d = 0.03;



y0 = np.array([
    0.0,
    30*np.pi/180,
    0.0,
    0.0
])

# m_c, m_p, L, g

params = np.array([
    0.390,
    0.614,
    0.154,
    9.81,
])

#     [    theta],
#     [    x_dot],
#     [theta_dot]])


Qx1 = np.diag([1, 0.1, 1])
Qu1a = np.diag([1])
K, X, E = ctl.lqr(A_ctrl_lnp(params), B_ctrl_lnp(params), Qx1, Qu1a)

K = np.array(K).squeeze()

y = y0

pend = PendulumVis()

ts = 0.01

Y = []
INP = []

while pend.running:

    inp = np.array([-K @ y[1:]])

    Y.append(y)
    INP.append(inp)

    y = rk4step(model_sim_lnp, y, ts, args=(params, inp))

    pend.update(y, inp)
    print(y)

    plt.pause(ts)

Y = np.array(Y)
INP = np.array(INP)

f3 = plt.figure(3)
f3.clf()

axs = []

for i in range(3):
    axs.append(f3.add_subplot(3, 1, i+1))
    axs[-1].grid(True, which="major")
    axs[-1].minorticks_on()
    axs[-1].grid(True, which="minor", linestyle=':', linewidth=1)


axs[0].plot(Y[:, 0], '.-', label="x")
axs[0].plot(Y[:, 2], '.-', label="x_dot")
axs[0].legend()


axs[1].plot(Y[:, 1], '.-', label="theta")
axs[1].plot(Y[:, 3], '.-', label="theta_dot")
axs[1].legend()

axs[2].plot(INP, '.-', label='F')
axs[2].legend()

# %%
#
#
# # %gui qt
# from IPython import get_ipython
# ipython = get_ipython()
# ipython.magic("gui qt")
#
# import numpy as np
# import pyqtgraph as pg
#
# data = np.random.normal(size=1000)
# # w = pg.plot(data, title="Simplest possible plotting example")
#
# win = pg.GraphicsWindow()
# win.resize(800, 800)
#
# p = win.addPlot()
# plot = p.plot(data)
#
# # %%
# data = np.random.normal(size=1000)
#
# plot.setData(data)

