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

    df = pd.DataFrame(pd.json_normalize(data))

    for k in df.keys():
        if df.dtypes[k] == np.object:
            df[k] = df[k].apply(lambda x: np.array(
                x["data"], dtype=np.dtype(x["dtype"])).reshape(x["shape"], order=x["order"]))

    # return data
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

            for data_obj in data_group:
                for column in df_columns:

                    scale = 1.0

                    if isinstance(data_obj, dict):
                        scale = data_obj["scale"]
                        data_name = data_obj["name"]
                    else:
                        data_name = data_obj

                    if data_name == column:
                        if any([skip_name in column for skip_name in skip_names]):
                            continue
                        if df[column].dtypes == np.dtype('O'):
                            continue

                        if wrt_iloc:
                            ax[-1].plot(df[column].values * scale,
                                        '.-', label=str(column))
                        else:
                            ax[-1].plot(df[column].index,
                                        df[column] * scale,
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
dump = dump.set_index('timestamp')

f, axs = plot_df_entry(dump, [
    ["state.x"],
    ["state.x_dot"],
    [{"name": "state.theta", "scale": 180.0/np.pi}],
    ["state.theta_dot"],
    ["state.omega"],
], fig=1, tight=False)


# %%


dump = load_dataset("/home/slovak/pibot/build/dump.msg")
dump.keys()
dump.timestamp = (dump.timestamp - dump.timestamp[0])*1e-6
dump = dump.set_index('timestamp')

f, axs = plot_df_entry(dump, [
    ["ctrl.x_dot_ref", "state.x_dot"],
    ["ctrl.theta_ref", "state.theta"],
    ["ctrl.t_l", "ctrl.t_r"],
], fig=1, tight=False)

# %%
dump = load_dataset("/home/slovak/pibot/build/dump.msg")
dump.keys()
dump.timestamp = (dump.timestamp - dump.timestamp[0])*1e-6
dump = dump.set_index('timestamp')


f, axs = plot_df_entry(dump, [
    ["ctrl.m_f_theta", "ctrl.m_f_ff", "ctrl.m_f_theta_dot", "ctrl.m_f_x_dot"],
    ["state.x_dot", "ctrl.x_dot_ref"],
    ["ctrl.theta_ref"],
    ["ctrl.x_m_x"],
], fig=1, tight=False)

# %%

dump = load_dataset("/home/slovak/pibot/build/dump.msg")
dump.keys()
dump.timestamp = (dump.timestamp - dump.timestamp[0])*1e-6
dump = dump.set_index('timestamp')

f, axs = plot_df_entry(dump, [
    ["ctrl.m_f_theta", "ctrl.m_f_ff", "ctrl.m_f_theta_dot", "ctrl.m_f_x_dot"],
    ["ctrl.m_f_omega"],
    ["ctrl.m_f"],
], fig=1, tight=False)

# %%


f, axs = plot_df_entry(dump, [
    ["ctrl.m_f_theta_dot", "state.theta_dot"],
], fig=1, tight=False)


# %% Time
dump = load_dataset("/home/slovak/pibot/build/dump.msg")
dump.keys()
dump.timestamp = (dump.timestamp - dump.timestamp[0])*1e-6
dump = dump.set_index('timestamp')

plt.plot(np.diff(dump.timestamp), '.')


# %%

dump = load_dataset("/home/slovak/pibot/build/dump.msg")
dump.keys()
dump.timestamp = (dump.timestamp - dump.timestamp[0])*1e-6
dump = dump.set_index('timestamp')

f, axs = plot_df_entry(None, [
    [],
    [],
], fig=1, tight=False)

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


# %% IMU

dump = load_dataset("/home/slovak/pibot/build/dump.msg")
dump.keys()
dump.timestamp = (dump.timestamp - dump.timestamp[0])*1e-6
dump = dump.set_index('timestamp')

f, axs = plot_df_entry(dump, [
    ["x", "y", "z"],
    ["theta"]
], fig=1, tight=False)

theta2 = np.arctan(dump.x/dump.z)

axs[1].plot(-theta2)

# %%
# plt.plot(dump.x, dump.z, '.')
#
# plt.gca().set_xlim([-1000, 1000])
# plt.gca().set_ylim([-1000, 1000])

# %%

dump = load_dataset("/home/slovak/pibot/build/dump.msg")
dump.keys()
dump.timestamp = (dump.timestamp - dump.timestamp[0])*1e-6
dump = dump.set_index('timestamp')

f, axs = plot_df_entry(dump, [
    ["state.acc_x", "state.acc_y", "state.acc_z"],
    ["state.theta"]
], fig=1, tight=False)

theta2 = np.arctan(dump["state.acc_x"]/dump["state.acc_z"])
axs[1].plot(-theta2)
