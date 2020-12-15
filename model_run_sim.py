import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import odeint

from model_cart_pendulum import model_sim

# model_sim(y0, 0.0, params, inp)
# t = np.linspace(0, 10, 101)
# sol = odeint(model_sim, y0, t, args=(params, inp))


def vis_pendulum(l_state, l_ax):

    l_ax.cla()

    l_x = l_state[0]
    l_theta = l_state[1]

    pend = np.array([np.cos(l_theta + np.pi/2), np.sin(l_theta + np.pi/2)])*1.0

    l_ax.plot(l_x, 0.0, 'o')
    l_ax.plot([l_x, l_x + pend[0]], [0.0, pend[1]], '.-')

    l_ax.set_aspect("equal")

    l_ax.set_xlim([-1.5, 1.5])
    l_ax.set_ylim([-1.5, 1.5])


def rk4step(f, y_i, h, args=()):

    t_i = 0.0  # TODO, OLSLO, model is not time dependent!

    k1 = np.array(f(y_i, t_i, *args))
    k2 = np.array(f(y_i + k1 * h / 2., t_i + h / 2., *args))
    k3 = np.array(f(y_i + k2 * h / 2., t_i + h / 2., *args))
    k4 = np.array(f(y_i + k3 * h, t_i + h, *args))
    y_ip1 = y_i + (h / 6.) * (k1 + 2*k2 + 2*k3 + k4)
    return y_ip1


class KeyCallback:
    running = True

    def press(self, event):
        if event.key == ' ':
            self.running = False


kp = KeyCallback()

f1 = plt.figure(1)
ax = f1.add_subplot(111)

f1.canvas.mpl_connect('key_press_event', kp.press)

# %%

y0 = np.array([
    0.0,
    0.01,
    0.0,
    0.0
])

params = np.array([
    0.3,
    1.0,
    1.0,
    9.81,
])

inp = np.array([0.0])


y = y0

# %%
kp.running = True

ts = 0.01

while kp.running:

    y = rk4step(model_sim, y, ts, args=(params, inp))

    vis_pendulum(y, ax)
    print(y)

    plt.pause(ts)

