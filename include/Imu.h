#ifndef __IMU_H__
#define __IMU_H__

#include <cstdint>

#include <pigpio.h>

#include "bno055.h"

class Imu {

 public:
  Imu();
  ~Imu();

  struct state_t {
    std::uint64_t timestamp = 0U;
    double theta = 0.0;
    double theta_dot = 0.0;
    double omega = 0.0;
    bool valid = false;
  };

  state_t m_state = {0};
  state_t m_state_raw = {0};

  bno055_euler_double_t GetEuler();
  double GetGyroY();
  void Update();

 private:
  bno055_t m_sensor_parameters;
  static int16_t m_i2c_handle;
  int m_i2c_channel = 1;

  static s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
  static s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
  static void BNO055_delay_msek(u32 msek);

};

#endif