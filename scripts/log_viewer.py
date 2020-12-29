import numpy as np

import pandas as pd
import msgpack
import numbers

import matplotlib

matplotlib.use('Qt5Agg')

import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import make_axes_locatable

pd.set_option('display.max_rows', 1000)
pd.set_option('display.max_columns', 1000)
pd.set_option('display.width', 1000)
pd.set_option('display.max_colwidth', 1000)


def load_dataset(l_dump_path):
    data = []

    with open(l_dump_path, "rb") as data_file:
        data = [unpacked for unpacked in msgpack.Unpacker(data_file)]

    df = pd.DataFrame(data)

    for k in df.keys():
        if df.dtypes[k] == np.object:
            df[k] = df[k].apply(lambda x: np.array(
                x["data"], dtype=np.dtype(x["dtype"])).reshape(x["shape"], order=x["order"]))

    return df


def plot_df_entry(df, data_groups, skip_names=(), wrt_iloc=False, fig=None, tight=True):
    plt.style.use('default')
    #     plt.style.use('dark_background')
    if fig is None:
        f = plt.figure(figsize=(18, 9), dpi=80)
    elif isinstance(fig, numbers.Number):
        f = plt.figure(fig, figsize=(18, 9), dpi=80)
        f.clf()
    else:
        f = fig
        f.clf()

    ax = []

    for idx, data_group in enumerate(data_groups):
        if len(ax) == 0:
            ax.append(f.add_subplot(len(data_groups), 1, idx + 1))
        else:
            ax.append(f.add_subplot(len(data_groups), 1, idx + 1, sharex=ax[0]))

        if isinstance(data_group, str):
            data_group = [data_group]

        if df is not None:
            df_columns = df.columns

            for data_name in data_group:
                for column in df_columns:
                    if data_name in column:
                        if any([skip_name in column for skip_name in skip_names]):
                            continue
                        if df[column].dtypes == np.dtype('O'):
                            continue

                        if wrt_iloc:
                            ax[-1].plot(df[column].values,
                                        '.-', label=str(column))
                        else:
                            ax[-1].plot(df[column].index,
                                        df[column],
                                        '.-', label=str(column))

        if len(ax[-1].get_lines()) > 0:
            ax[-1].legend(loc='upper right')

        ax[-1].grid(True, which="major")
        ax[-1].minorticks_on()
        ax[-1].grid(True, which="minor", linestyle=':', linewidth=1)

    if tight:
        f.tight_layout()

    return f, ax

# %%


dump = load_dataset("/home/slovak/pibot/build/dump.msg")
dump.keys()
dump.timestamp = (dump.timestamp - dump.timestamp[0])*1e-6

f, axs = plot_df_entry(None, [
    [],
    [],
    [],
    [],
    [],
    [],
], wrt_iloc=True, fig=1, tight=False)

n = -1



n += 1
ax = axs[n]
ax.plot(dump.timestamp, dump.x, '.-', label="x")
xl = ax.legend()

n += 1
ax = axs[n]
ax.plot(dump.timestamp, dump.x_dot, '.-', label="x_dot")
xl = ax.legend()

n += 1
ax = axs[n]
ax.plot(dump.timestamp, dump.theta * 180.0/np.pi, '.-', label="theta")
xl = ax.legend()

n += 1
ax = axs[n]
ax.plot(dump.timestamp, dump.theta_dot, '.-', label="theta_dot")
xl = ax.legend()

n += 1
ax = axs[n]
ax.plot(dump.timestamp, dump.omega, '.-', label="omega")
xl = ax.legend()


# %%


dump = load_dataset("/home/slovak/pibot/build/dump.msg")
dump.keys()
dump.timestamp = (dump.timestamp - dump.timestamp[0])*1e-6


f, axs = plot_df_entry(None, [
    [],
    [],
    [],
    [],
], wrt_iloc=True, fig=1, tight=False)

n = -1

n += 1
ax = axs[n]
ax.plot(dump.timestamp, dump.m_f_theta, '.-', label="m_f_theta")
ax.plot(dump.timestamp, dump.m_f_theta_dot, '.-', label="m_f_theta_dot")
xl = ax.legend()

n += 1
ax = axs[n]
ax.plot(dump.timestamp, dump.m_f_x, '.-', label="m_f_x")
ax.plot(dump.timestamp, dump.m_f_x_dot, '.-', label="m_f_x_dot")
ax.plot(dump.timestamp, dump.m_f_x + dump.m_f_x_dot, '.-', label="dump.m_f_x + dump.m_f_x_dot")
xl = ax.legend()



n += 1
ax = axs[n]
ax.plot(dump.timestamp, dump.torque0, '.-', label="torque0")
ax.plot(dump.timestamp, dump.torque1, '.-', label="torque1")
xl = ax.legend()

n += 1
ax = axs[n]
ax.plot(dump.timestamp, dump.m_f_omega, '.-', label="m_f_omega")
# ax.plot(dump.timestamp, dump.mode_r, '.-', label="mode_r")
# ax.plot(dump.timestamp, dump.mode_r, '.-', label="mode_r")
# ax.plot(dump.timestamp, dump.position0, '.-', label="position0")
# ax.plot(dump.timestamp, dump.position1, '.-', label="position1")
xl = ax.legend()
#
#
# # plt.plot(np.diff(dump.timestamp), '.')


# %%


dump = load_dataset("/home/slovak/pibot/build/dump.msg")
dump.keys()
dump.timestamp = (dump.timestamp - dump.timestamp[0])*1e-6

f, axs = plot_df_entry(None, [
    [],
    [],
], wrt_iloc=True, fig=1, tight=False)

n = -1

velocity_raw_l_rot_per_sec = dump.velocity_raw_l
velocity_raw_r_rot_per_sec = dump.velocity_raw_r

theta_dot_rot_per_sec = dump.theta_dot / (2.0 * np.pi)

n += 1
ax = axs[n]
ax.plot(dump.timestamp, velocity_raw_l_rot_per_sec, '.-', label="velocity_raw_l_rot_per_sec")
ax.plot(dump.timestamp, velocity_raw_r_rot_per_sec, '.-', label="velocity_raw_r_rot_per_sec")
ax.plot(dump.timestamp, theta_dot_rot_per_sec, '.-', label="theta_dot_rot_per_sec")
xl = ax.legend()



n += 1
ax = axs[n]
ax.plot(dump.timestamp, velocity_raw_l_rot_per_sec + theta_dot_rot_per_sec, '.-', label="velocity_raw_l_rot_per_sec")
xl = ax.legend()

#
# n += 1
# ax = axs[n]
# xl = ax.legend()
