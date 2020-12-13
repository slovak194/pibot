import signal
import sys
import time
import asyncio

import numpy as np

import board
import busio
import adafruit_bno055

import moteus
c1 = moteus.Controller()
c2 = moteus.Controller(id=2)

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

sensor.mode = adafruit_bno055.IMUPLUS_MODE


running = True


def signal_handler(signal, frame):
    print('Stopping gradually ....')
    global running
    running = False


signal.signal(signal.SIGINT, signal_handler)


async def main():
    i = 0

    while running:
        i = i + 1
        euler = np.array(sensor._euler) * np.pi/180.0
        gyro = np.array(sensor.gyro)

        k_p = 0.35

        k_d = 0.025

        # print("euler: ", euler)
        # print("gyro: ", gyro)

        # position=None,
        # velocity=None,
        # feedforward_torque=None,
        # kp_scale=None,
        # kd_scale=None,
        # maximum_torque=None,
        # stop_position=None,
        # watchdog_timeout=None,
        # query=False):

        t_1 = euler[1] * k_p + gyro[1] * k_d
        q1 = await c1.query()
        q2 = await c2.query()

        if np.abs(t_1) > 1.5:
            print("torque: ", t_1)
            break

        if np.abs(q1.values[2]) > 4.0 or np.abs(q2.values[2]) > 4.0:
            print("speed: ", q1.values[2])
            print("speed: ", q2.values[2])
            break

        if -1.0 < euler[1] < 1.0:
            pass
            await c1.set_position(kp_scale=0.0, kd_scale=0.0, feedforward_torque=t_1, watchdog_timeout=0.5)
            await c2.set_position(kp_scale=0.0, kd_scale=0.0, feedforward_torque=t_1 * -1, watchdog_timeout=0.5)
        if not (i % 10):
            pass
            print(await c1.query())
            # print(await c2.query())
        await asyncio.sleep(0.01)
    await c1.set_stop()
    await c2.set_stop()

asyncio.run(main())
