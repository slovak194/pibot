#ifndef __BNO055_INTERFACE_H__
#define __BNO055_INTERFACE_H__

#include <pigpio.h>

#include "bno055.h"

class BNO055 {

 public:
  BNO055();
  ~BNO055();

  bno055_euler_double_t GetEuler();

 private:
  bno055_t m_sensor_parameters;
  static int16_t m_i2c_handle;
  int m_i2c_channel = 1;

  static s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
  static s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
  static void BNO055_delay_msek(u32 msek);

};

#endif