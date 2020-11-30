
# www.waveshare.com/wiki/Raspberry_Pi_Tutorial_Series:_I2C


# i2cset -y 1 0x28 0x3d 0x08
# i2cset -y 1 0x28 0x3d 0x08
# i2cset -y 1 0x28 0x3d 0x08

import smbus
import time
import binascii
import struct

# _CHIP_ID = const(0xA0)

# CONFIG_MODE = const(0x00)
# ACCONLY_MODE = const(0x01)
# MAGONLY_MODE = const(0x02)
# GYRONLY_MODE = const(0x03)
# ACCMAG_MODE = const(0x04)
# ACCGYRO_MODE = const(0x05)
# MAGGYRO_MODE = const(0x06)
# AMG_MODE = const(0x07)
# IMUPLUS_MODE = const(0x08)
# COMPASS_MODE = const(0x09)
# M4G_MODE = const(0x0A)
# NDOF_FMC_OFF_MODE = const(0x0B)
# NDOF_MODE = const(0x0C)

# ACCEL_2G = const(0x00)  # For accel_range property
# ACCEL_4G = const(0x01)  # Default
# ACCEL_8G = const(0x02)
# ACCEL_16G = const(0x03)
# ACCEL_7_81HZ = const(0x00)  # For accel_bandwidth property
# ACCEL_15_63HZ = const(0x04)
# ACCEL_31_25HZ = const(0x08)
# ACCEL_62_5HZ = const(0x0C)  # Default
# ACCEL_125HZ = const(0x10)
# ACCEL_250HZ = const(0x14)
# ACCEL_500HZ = const(0x18)
# ACCEL_1000HZ = const(0x1C)
# ACCEL_NORMAL_MODE = const(0x00)  # Default. For accel_mode property
# ACCEL_SUSPEND_MODE = const(0x20)
# ACCEL_LOWPOWER1_MODE = const(0x40)
# ACCEL_STANDBY_MODE = const(0x60)
# ACCEL_LOWPOWER2_MODE = const(0x80)
# ACCEL_DEEPSUSPEND_MODE = const(0xA0)

# GYRO_2000_DPS = const(0x00)  # Default. For gyro_range property
# GYRO_1000_DPS = const(0x01)
# GYRO_500_DPS = const(0x02)
# GYRO_250_DPS = const(0x03)
# GYRO_125_DPS = const(0x04)
# GYRO_523HZ = const(0x00)  # For gyro_bandwidth property
# GYRO_230HZ = const(0x08)
# GYRO_116HZ = const(0x10)
# GYRO_47HZ = const(0x18)
# GYRO_23HZ = const(0x20)
# GYRO_12HZ = const(0x28)
# GYRO_64HZ = const(0x30)
# GYRO_32HZ = const(0x38)  # Default
# GYRO_NORMAL_MODE = const(0x00)  # Default. For gyro_mode property
# GYRO_FASTPOWERUP_MODE = const(0x01)
# GYRO_DEEPSUSPEND_MODE = const(0x02)
# GYRO_SUSPEND_MODE = const(0x03)
# GYRO_ADVANCEDPOWERSAVE_MODE = const(0x04)

# MAGNET_2HZ = const(0x00)  # For magnet_rate property
# MAGNET_6HZ = const(0x01)
# MAGNET_8HZ = const(0x02)
# MAGNET_10HZ = const(0x03)
# MAGNET_15HZ = const(0x04)
# MAGNET_20HZ = const(0x05)  # Default
# MAGNET_25HZ = const(0x06)
# MAGNET_30HZ = const(0x07)
# MAGNET_LOWPOWER_MODE = const(0x00)  # For magnet_operation_mode property
# MAGNET_REGULAR_MODE = const(0x08)  # Default
# MAGNET_ENHANCEDREGULAR_MODE = const(0x10)
# MAGNET_ACCURACY_MODE = const(0x18)
# MAGNET_NORMAL_MODE = const(0x00)  # for magnet_power_mode property
# MAGNET_SLEEP_MODE = const(0x20)
# MAGNET_SUSPEND_MODE = const(0x40)
# MAGNET_FORCEMODE_MODE = const(0x60)  # Default

# _POWER_NORMAL = const(0x00)
# _POWER_LOW = const(0x01)
# _POWER_SUSPEND = const(0x02)

# _MODE_REGISTER = const(0x3D)
# _PAGE_REGISTER = const(0x07)
# _ACCEL_CONFIG_REGISTER = const(0x08)
# _MAGNET_CONFIG_REGISTER = const(0x09)
# _GYRO_CONFIG_0_REGISTER = const(0x0A)
# _GYRO_CONFIG_1_REGISTER = const(0x0B)
# _CALIBRATION_REGISTER = const(0x35)
# _OFFSET_ACCEL_REGISTER = const(0x55)
# _OFFSET_MAGNET_REGISTER = const(0x5B)
# _OFFSET_GYRO_REGISTER = const(0x61)
# _RADIUS_ACCEL_REGISTER = const(0x67)
# _RADIUS_MAGNET_REGISTER = const(0x69)
# _TRIGGER_REGISTER = const(0x3F)
# _POWER_REGISTER = const(0x3E)
# _ID_REGISTER = const(0x00)

# I2C addresses
BNO055_ADDRESS_A                     = 0x28
BNO055_ADDRESS_B                     = 0x29
BNO055_ID                            = 0xA0

# Page id register definition
BNO055_PAGE_ID_ADDR                  = 0X07

# PAGE0 REGISTER DEFINITION START
BNO055_CHIP_ID_ADDR                  = 0x00
BNO055_ACCEL_REV_ID_ADDR             = 0x01
BNO055_MAG_REV_ID_ADDR               = 0x02
BNO055_GYRO_REV_ID_ADDR              = 0x03
BNO055_SW_REV_ID_LSB_ADDR            = 0x04
BNO055_SW_REV_ID_MSB_ADDR            = 0x05
BNO055_BL_REV_ID_ADDR                = 0X06

# Accel data register
BNO055_ACCEL_DATA_X_LSB_ADDR         = 0X08
BNO055_ACCEL_DATA_X_MSB_ADDR         = 0X09
BNO055_ACCEL_DATA_Y_LSB_ADDR         = 0X0A
BNO055_ACCEL_DATA_Y_MSB_ADDR         = 0X0B
BNO055_ACCEL_DATA_Z_LSB_ADDR         = 0X0C
BNO055_ACCEL_DATA_Z_MSB_ADDR         = 0X0D

# Mag data register
BNO055_MAG_DATA_X_LSB_ADDR           = 0X0E
BNO055_MAG_DATA_X_MSB_ADDR           = 0X0F
BNO055_MAG_DATA_Y_LSB_ADDR           = 0X10
BNO055_MAG_DATA_Y_MSB_ADDR           = 0X11
BNO055_MAG_DATA_Z_LSB_ADDR           = 0X12
BNO055_MAG_DATA_Z_MSB_ADDR           = 0X13

# Gyro data registers
BNO055_GYRO_DATA_X_LSB_ADDR          = 0X14
BNO055_GYRO_DATA_X_MSB_ADDR          = 0X15
BNO055_GYRO_DATA_Y_LSB_ADDR          = 0X16
BNO055_GYRO_DATA_Y_MSB_ADDR          = 0X17
BNO055_GYRO_DATA_Z_LSB_ADDR          = 0X18
BNO055_GYRO_DATA_Z_MSB_ADDR          = 0X19

# Euler data registers
BNO055_EULER_H_LSB_ADDR              = 0X1A
BNO055_EULER_H_MSB_ADDR              = 0X1B
BNO055_EULER_R_LSB_ADDR              = 0X1C
BNO055_EULER_R_MSB_ADDR              = 0X1D
BNO055_EULER_P_LSB_ADDR              = 0X1E
BNO055_EULER_P_MSB_ADDR              = 0X1F

# Quaternion data registers
BNO055_QUATERNION_DATA_W_LSB_ADDR    = 0X20
BNO055_QUATERNION_DATA_W_MSB_ADDR    = 0X21
BNO055_QUATERNION_DATA_X_LSB_ADDR    = 0X22
BNO055_QUATERNION_DATA_X_MSB_ADDR    = 0X23
BNO055_QUATERNION_DATA_Y_LSB_ADDR    = 0X24
BNO055_QUATERNION_DATA_Y_MSB_ADDR    = 0X25
BNO055_QUATERNION_DATA_Z_LSB_ADDR    = 0X26
BNO055_QUATERNION_DATA_Z_MSB_ADDR    = 0X27

# Linear acceleration data registers
BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR  = 0X28
BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR  = 0X29
BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR  = 0X2A
BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR  = 0X2B
BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR  = 0X2C
BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR  = 0X2D

# Gravity data registers
BNO055_GRAVITY_DATA_X_LSB_ADDR       = 0X2E
BNO055_GRAVITY_DATA_X_MSB_ADDR       = 0X2F
BNO055_GRAVITY_DATA_Y_LSB_ADDR       = 0X30
BNO055_GRAVITY_DATA_Y_MSB_ADDR       = 0X31
BNO055_GRAVITY_DATA_Z_LSB_ADDR       = 0X32
BNO055_GRAVITY_DATA_Z_MSB_ADDR       = 0X33

# Temperature data register
BNO055_TEMP_ADDR                     = 0X34

# Status registers
BNO055_CALIB_STAT_ADDR               = 0X35
BNO055_SELFTEST_RESULT_ADDR          = 0X36
BNO055_INTR_STAT_ADDR                = 0X37

BNO055_SYS_CLK_STAT_ADDR             = 0X38
BNO055_SYS_STAT_ADDR                 = 0X39
BNO055_SYS_ERR_ADDR                  = 0X3A

# Unit selection register
BNO055_UNIT_SEL_ADDR                 = 0X3B
BNO055_DATA_SELECT_ADDR              = 0X3C

# Mode registers
BNO055_OPR_MODE_ADDR                 = 0X3D
BNO055_PWR_MODE_ADDR                 = 0X3E

BNO055_SYS_TRIGGER_ADDR              = 0X3F
BNO055_TEMP_SOURCE_ADDR              = 0X40

# Axis remap registers
BNO055_AXIS_MAP_CONFIG_ADDR          = 0X41
BNO055_AXIS_MAP_SIGN_ADDR            = 0X42

# Axis remap values
AXIS_REMAP_X                         = 0x00
AXIS_REMAP_Y                         = 0x01
AXIS_REMAP_Z                         = 0x02
AXIS_REMAP_POSITIVE                  = 0x00
AXIS_REMAP_NEGATIVE                  = 0x01

# SIC registers
BNO055_SIC_MATRIX_0_LSB_ADDR         = 0X43
BNO055_SIC_MATRIX_0_MSB_ADDR         = 0X44
BNO055_SIC_MATRIX_1_LSB_ADDR         = 0X45
BNO055_SIC_MATRIX_1_MSB_ADDR         = 0X46
BNO055_SIC_MATRIX_2_LSB_ADDR         = 0X47
BNO055_SIC_MATRIX_2_MSB_ADDR         = 0X48
BNO055_SIC_MATRIX_3_LSB_ADDR         = 0X49
BNO055_SIC_MATRIX_3_MSB_ADDR         = 0X4A
BNO055_SIC_MATRIX_4_LSB_ADDR         = 0X4B
BNO055_SIC_MATRIX_4_MSB_ADDR         = 0X4C
BNO055_SIC_MATRIX_5_LSB_ADDR         = 0X4D
BNO055_SIC_MATRIX_5_MSB_ADDR         = 0X4E
BNO055_SIC_MATRIX_6_LSB_ADDR         = 0X4F
BNO055_SIC_MATRIX_6_MSB_ADDR         = 0X50
BNO055_SIC_MATRIX_7_LSB_ADDR         = 0X51
BNO055_SIC_MATRIX_7_MSB_ADDR         = 0X52
BNO055_SIC_MATRIX_8_LSB_ADDR         = 0X53
BNO055_SIC_MATRIX_8_MSB_ADDR         = 0X54

# Accelerometer Offset registers
ACCEL_OFFSET_X_LSB_ADDR              = 0X55
ACCEL_OFFSET_X_MSB_ADDR              = 0X56
ACCEL_OFFSET_Y_LSB_ADDR              = 0X57
ACCEL_OFFSET_Y_MSB_ADDR              = 0X58
ACCEL_OFFSET_Z_LSB_ADDR              = 0X59
ACCEL_OFFSET_Z_MSB_ADDR              = 0X5A

# Magnetometer Offset registers
MAG_OFFSET_X_LSB_ADDR                = 0X5B
MAG_OFFSET_X_MSB_ADDR                = 0X5C
MAG_OFFSET_Y_LSB_ADDR                = 0X5D
MAG_OFFSET_Y_MSB_ADDR                = 0X5E
MAG_OFFSET_Z_LSB_ADDR                = 0X5F
MAG_OFFSET_Z_MSB_ADDR                = 0X60

# Gyroscope Offset register s
GYRO_OFFSET_X_LSB_ADDR               = 0X61
GYRO_OFFSET_X_MSB_ADDR               = 0X62
GYRO_OFFSET_Y_LSB_ADDR               = 0X63
GYRO_OFFSET_Y_MSB_ADDR               = 0X64
GYRO_OFFSET_Z_LSB_ADDR               = 0X65
GYRO_OFFSET_Z_MSB_ADDR               = 0X66

# Radius registers
ACCEL_RADIUS_LSB_ADDR                = 0X67
ACCEL_RADIUS_MSB_ADDR                = 0X68
MAG_RADIUS_LSB_ADDR                  = 0X69
MAG_RADIUS_MSB_ADDR                  = 0X6A

# Power modes
POWER_MODE_NORMAL                    = 0X00
POWER_MODE_LOWPOWER                  = 0X01
POWER_MODE_SUSPEND                   = 0X02

# Operation mode settings
OPERATION_MODE_CONFIG                = 0X00
OPERATION_MODE_ACCONLY               = 0X01
OPERATION_MODE_MAGONLY               = 0X02
OPERATION_MODE_GYRONLY               = 0X03
OPERATION_MODE_ACCMAG                = 0X04
OPERATION_MODE_ACCGYRO               = 0X05
OPERATION_MODE_MAGGYRO               = 0X06
OPERATION_MODE_AMG                   = 0X07
OPERATION_MODE_IMUPLUS               = 0X08
OPERATION_MODE_COMPASS               = 0X09
OPERATION_MODE_M4G                   = 0X0A
OPERATION_MODE_NDOF_FMC_OFF          = 0X0B
OPERATION_MODE_NDOF                  = 0X0C

address = 0x28

bus = smbus.SMBus(1)



# TODO, OLSLO, i2cset -y 1 0x28 0x3d 0x08 


def read_signed_short(lbus, laddr, lsb_addr, msb_addr=None):
    if msb_addr is None:
        msb_addr = lsb_addr + 1
    return struct.unpack("h",
    lbus.read_byte_data(laddr, lsb_addr).to_bytes(1, byteorder='little') +
    lbus.read_byte_data(laddr, msb_addr).to_bytes(1, byteorder='little'))[0] 


while True:

    res = read_signed_short(bus, address,
    BNO055_ACCEL_DATA_Z_LSB_ADDR)

    # print(res)

    lsb = bus.read_byte_data(address, BNO055_ACCEL_DATA_Z_LSB_ADDR)
    msb = bus.read_byte_data(address, BNO055_ACCEL_DATA_Z_MSB_ADDR)

    print(bin(msb), " ",  bin(lsb))
    print(hex(msb), " ",  hex(lsb))
    print("")
    time.sleep(1/20)

