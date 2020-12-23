import time
import board
import busio
import adafruit_bno055

import numpy as np
np.set_printoptions(linewidth=np.inf, precision=6, suppress=True)

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

sensor.mode = adafruit_bno055.IMUPLUS_MODE

tt = None

for i in range(1000000):
    t0 = time.clock()
    g = sensor.gyro
    t1 = time.clock()

    t = (t1-t0)

    if tt is None:
        tt = t
    else:
        if t > tt:
            tt = t

    print(tt, t)
    # print(np.array(sensor._euler))
    # time.sleep(1.0/50.0)
