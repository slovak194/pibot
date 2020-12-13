import signal
import sys
import time
import asyncio

import moteus

c = moteus.Controller()

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
        await c.set_position(position=None, velocity=None, feedforward_torque=0.1, watchdog_timeout=0.5)

        if not (i % 10):
            print(await c.query())

        await asyncio.sleep(0.01)

    await c.set_stop()

asyncio.run(main())

# position=None,
# velocity=None,
# feedforward_torque=None,
# kp_scale=None,
# kd_scale=None,
# maximum_torque=None,
# stop_position=None,
# watchdog_timeout=None,
# query=False):