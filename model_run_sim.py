import matplotlib

matplotlib.use('Qt5Agg')
# matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
import numpy as np

import control as ctl

from scipy.integrate import odeint

from model_cart_pendulum import model_sim_lnp, A_sim_lnp, B_sim_lnp, A_ctrl_lnp, B_ctrl_lnp

# model_sim(y0, 0.0, params, inp)
# t = np.linspace(0, 10, 101)
# sol = odeint(model_sim, y0, t, args=(params, inp))


class PendulumVis:

    running = True
    first = True

    def press(self, event):
        if event.key == ' ':
            self.running = False

    def update(self, l_state):

        l_x = l_state[0]
        l_theta = l_state[1]

        pend = np.array([np.cos(l_theta + np.pi/2), np.sin(l_theta + np.pi/2)])*1.0

        if self.first:

            self.f1 = plt.figure(1)
            self.ax = self.f1.add_subplot(111)

            self.f1.canvas.mpl_connect('key_press_event', self.press)

            self.ax.cla()

            self.cart = self.ax.plot(l_x, 0.0, 'o')
            self.pend = self.ax.plot([l_x, l_x + pend[0]], [0.0, pend[1]], '.-')

            self.ax.set_aspect("equal")

            self.ax.set_xlim([-1.5, 1.5])
            self.ax.set_ylim([-1.5, 1.5])

            self.ax.grid(True, which="major")
            self.ax.minorticks_on()
            self.ax.grid(True, which="minor", linestyle=':', linewidth=1)

            self.first = False

        else:
            self.cart[0].set_data(l_x, 0.0)
            self.pend[0].set_data([l_x, l_x + pend[0]], [0.0, pend[1]])


def rk4step(f, y_i, h, args=()):

    t_i = 0.0  # TODO, OLSLO, model is not time dependent!

    k1 = np.array(f(y_i, t_i, *args))
    k2 = np.array(f(y_i + k1 * h / 2., t_i + h / 2., *args))
    k3 = np.array(f(y_i + k2 * h / 2., t_i + h / 2., *args))
    k4 = np.array(f(y_i + k3 * h, t_i + h, *args))
    y_ip1 = y_i + (h / 6.) * (k1 + 2*k2 + 2*k3 + k4)
    return y_ip1

#
# class KeyCallback:
#     running = True
#
#     def press(self, event):
#         if event.key == ' ':
#             self.running = False


# kp = KeyCallback()


y0 = np.array([
    0.0,
    0.2,
    0.0,
    0.0
])

params = np.array([
    0.3,
    1.0,
    1.0,
    9.81,
])

Qx1 = np.diag([1, 1, 1])
Qu1a = np.diag([1])
K, X, E = ctl.lqr(A_ctrl_lnp(params), B_ctrl_lnp(params), Qx1, Qu1a)

K = np.array(K).squeeze()

# %%

y = y0

# kp.running = True

pend = PendulumVis()

ts = 0.1

# K = np.array([
#     0.0,
#     100.0,
#     0.0,
#     50.0,
# ])

while pend.running:

    # inp = np.array([1.0])
    inp = np.array([-K @ y[1:]])

    y = rk4step(model_sim_lnp, y, ts, args=(params, inp))

    pend.update(y)
    # vis_pendulum(y, ax)
    print(y)

    plt.pause(ts)


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

