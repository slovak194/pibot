#include "bno055_interface.h"

#include <iostream>

#include <thread>
#include <chrono>

#include <math.h>

using namespace std::chrono_literals;

int16_t BNO055::m_i2c_handle = -1;

BNO055::BNO055() {
  s32 comres = BNO055_ERROR;

  if (gpioInitialise() < 0)
  {
    std::cout <<"Initialisation error of the GPIO \n Closing program..."<< std::endl;
    exit(1);
  }

  m_i2c_handle=i2cOpen(m_i2c_channel, BNO055_I2C_ADDR1,0);

  m_sensor_parameters.bus_write = BNO055::BNO055_I2C_bus_write;
  m_sensor_parameters.bus_read = BNO055::BNO055_I2C_bus_read;
  m_sensor_parameters.delay_msec = BNO055::BNO055_delay_msek;
  m_sensor_parameters.dev_addr = BNO055_I2C_ADDR1;

  comres = bno055_init(&m_sensor_parameters);
  comres += bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);

  /************************* START READ RAW FUSION DATA ********
   * For reading fusion data it is required to set the
   * operation modes of the sensor
   * operation mode can set from the register
   * page - page0
   * register - 0x3D
   * bit - 0 to 3
   * for sensor data read following operation mode have to set
   * FUSION MODE
   * 0x08 - BNO055_OPERATION_MODE_IMUPLUS
   * 0x09 - BNO055_OPERATION_MODE_COMPASS
   * 0x0A - BNO055_OPERATION_MODE_M4G
   * 0x0B - BNO055_OPERATION_MODE_NDOF_FMC_OFF
   * 0x0C - BNO055_OPERATION_MODE_NDOF
   * based on the user need configure the operation mode*/
  comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_IMUPLUS);
//  comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);

}

BNO055::~BNO055() {

  s32 comres = BNO055_ERROR;

  /*  For de - initializing the BNO sensor it is required
   * to the operation mode of the sensor as SUSPEND
   * Suspend mode can set from the register
   * Page - page0
   * register - 0x3E
   * bit positions - 0 and 1*/

  comres += bno055_set_power_mode(BNO055_POWER_MODE_SUSPEND);

}


bno055_euler_double_t BNO055::GetEuler() {
  struct bno055_euler_double_t d_euler_hpr;
  auto result = bno055_convert_double_euler_hpr_deg(&d_euler_hpr);
  return d_euler_hpr;
}

bno055_gyro_double_t BNO055::GetGyro() {

  struct bno055_gyro_double_t d_gyro_xyz_rps;
  auto result = bno055_convert_double_gyro_xyz_rps(&d_gyro_xyz_rps);
  return d_gyro_xyz_rps;
}

double BNO055::GetGyroY() {
  double d_gyro_y;
//  auto result = bno055_convert_double_gyro_y_rps(&d_gyro_y);  // This causes unintentional mode switching.
  auto result = bno055_convert_double_gyro_y_dps(&d_gyro_y);
  if (result == BNO055_SUCCESS) {
    return d_gyro_y * M_PI / 180.0;
  } else {
    return 0.0;
  }

}


s8 BNO055::BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{

  int result = i2cWriteI2CBlockData(m_i2c_handle, static_cast<unsigned>(reg_addr), static_cast<char *>(static_cast<void *>(reg_data)), static_cast<unsigned>(cnt));

  s32 BNO055_iERROR = BNO055_INIT_VALUE;

  if (result == 0) {
    BNO055_iERROR = BNO055_SUCCESS;
  } else {
    BNO055_iERROR = BNO055_ERROR;
  }

  return (s8)BNO055_iERROR;
}

s8 BNO055::BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{

//  int i2cReadI2CBlockData(unsigned handle, unsigned i2cReg, char *buf, unsigned count);
  int result = i2cReadI2CBlockData(m_i2c_handle, static_cast<unsigned>(reg_addr), static_cast<char *>(static_cast<void *>(reg_data)), static_cast<unsigned>(cnt));

  s32 BNO055_iERROR = BNO055_INIT_VALUE;

  if (result > 0) {
    BNO055_iERROR = BNO055_SUCCESS;
  } else {
    BNO055_iERROR = BNO055_ERROR;
  }

  return (s8)BNO055_iERROR;
}

void BNO055::BNO055_delay_msek(u32 msek)
{

//  printf("%d\n", msek);

  std::this_thread::sleep_for(std::chrono::milliseconds(msek));
//    gpioSleep(PI_TIME_RELATIVE, 0, static_cast<int>(1000*msek));

}
