#include "bno055_interface.h"

#include <iostream>

#include <thread>
#include <chrono>

#include <pigpio.h>

using namespace std::chrono_literals;


/*----------------------------------------------------------------------------*
 *  struct bno055_t parameters can be accessed by using BNO055
 *  BNO055_t having the following parameters
 *  Bus write function pointer: BNO055_WR_FUNC_PTR
 *  Bus read function pointer: BNO055_RD_FUNC_PTR
 *  Burst read function pointer: BNO055_BRD_FUNC_PTR
 *  Delay function pointer: delay_msec
 *  I2C address: dev_addr
 *  Chip id of the sensor: chip_id
 *---------------------------------------------------------------------------*/
struct bno055_t bno055;

int16_t _HandleBNO;
int _i2cChannel = 1;

s32 bno055_suspend(void) {

  s32 comres = BNO055_ERROR;

  u8 power_mode = BNO055_INIT_VALUE;

  /*  For de - initializing the BNO sensor it is required
   * to the operation mode of the sensor as SUSPEND
   * Suspend mode can set from the register
   * Page - page0
   * register - 0x3E
   * bit positions - 0 and 1*/
  power_mode = BNO055_POWER_MODE_SUSPEND;

  /* set the power mode as SUSPEND*/
  comres += bno055_set_power_mode(power_mode);

  return comres;

}


s32 bno055_init(void) {

  s32 comres = BNO055_ERROR;

  u8 power_mode = BNO055_INIT_VALUE;

  I2C_routine();

  /*--------------------------------------------------------------------------*
   *  This API used to assign the value/reference of
   *  the following parameters
   *  I2C address
   *  Bus Write
   *  Bus read
   *  Chip id
   *  Page id
   *  Accel revision id
   *  Mag revision id
   *  Gyro revision id
   *  Boot loader revision id
   *  Software revision id
   *-------------------------------------------------------------------------*/
  comres = bno055_init(&bno055);

  /*  For initializing the BNO sensor it is required to the operation mode
   * of the sensor as NORMAL
   * Normal mode can set from the register
   * Page - page0
   * register - 0x3E
   * bit positions - 0 and 1*/
  power_mode = BNO055_POWER_MODE_NORMAL;

  /* set the power mode as NORMAL*/
  comres += bno055_set_power_mode(power_mode);


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
  comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);

  /*----------------------------------------------------------------*
   ************************* END INITIALIZATION *************************
   *-----------------------------------------------------------------*/

  return comres;
}

#ifdef  BNO055_API

/*--------------------------------------------------------------------------*
 *  The following API is used to map the I2C bus read, write, delay and
 *  device address with global structure bno055_t
 *-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*
 *  By using bno055 the following structure parameter can be accessed
 *  Bus write function pointer: BNO055_WR_FUNC_PTR
 *  Bus read function pointer: BNO055_RD_FUNC_PTR
 *  Delay function pointer: delay_msec
 *  I2C address: dev_addr
 *--------------------------------------------------------------------------*/
s8 I2C_routine(void)
{
    if (gpioInitialise() <0)
    {
      std::cout <<"Initialisation error of the GPIO \n Closing program..."<< std::endl;
      return -1;
    }

    _HandleBNO=i2cOpen(_i2cChannel, BNO055_I2C_ADDR1,0);

    bno055.bus_write = BNO055_I2C_bus_write;
    bno055.bus_read = BNO055_I2C_bus_read;
    bno055.delay_msec = BNO055_delay_msek;
    bno055.dev_addr = BNO055_I2C_ADDR1;

    return BNO055_INIT_VALUE;
}

/************** I2C buffer length******/

#define I2C_BUFFER_LEN 8
#define I2C0           5

/*-------------------------------------------------------------------*
 *
 *  This is a sample code for read and write the data by using I2C
 *  Use either I2C  based on your need
 *  The device address defined in the bno055.h file
 *
 *--------------------------------------------------------------------*/

/*  \Brief: The API is used as I2C bus write
 *  \Return : Status of the I2C write
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register,
 *   will data is going to be written
 *  \param reg_data : It is a value hold in the array,
 *      will be used for write the value into the register
 *  \param cnt : The no of byte of data to be write
 */
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{

//    printf("[WR] ");
//    printf("dev_addr: 0x%02x | ", dev_addr);
//    printf("reg_addr: 0x%02x | ", reg_addr);
//    printf("cnt: 0x%02x | ", cnt);
//    printf("reg_data: ");
//
//    for (int i = 0; i < cnt; i++) {
//      printf("0x%02x ", *(reg_data + i));
//    }
//
//    printf("\n");

    int result = i2cWriteI2CBlockData(_HandleBNO, static_cast<unsigned>(reg_addr), static_cast<char *>(static_cast<void *>(reg_data)), static_cast<unsigned>(cnt));

    s32 BNO055_iERROR = BNO055_INIT_VALUE;

//    u8 array[I2C_BUFFER_LEN];
//    u8 stringpos = BNO055_INIT_VALUE;

//    array[BNO055_INIT_VALUE] = reg_addr;
//    for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
//    {
//        array[stringpos + BNO055_I2C_BUS_WRITE_ARRAY_INDEX] = *(reg_data + stringpos);
//    }
//}

/*
 * Please take the below APIs as your reference for
 * write the data using I2C communication
 * "BNO055_iERROR = I2C_WRITE_STRING(DEV_ADDR, ARRAY, CNT+1)"
 * add your I2C write APIs here
 * BNO055_iERROR is an return value of I2C read API
 * Please select your valid return value
 * In the driver BNO055_SUCCESS defined as 0
 * and FAILURE defined as -1
 * Note :
 * This is a full duplex operation,
 * The first read data is discarded, for that extra write operation
 * have to be initiated. For that cnt+1 operation done
 * in the I2C write string function
 * For more information please refer data sheet SPI communication:
 */


  if (result == 0) {
    BNO055_iERROR = BNO055_SUCCESS;
  } else {
    BNO055_iERROR = BNO055_ERROR;
  }



return (s8)BNO055_iERROR;
}

/*  \Brief: The API is used as I2C bus read
 *  \Return : Status of the I2C read
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register,
 *  will data is going to be read
 *  \param reg_data : This data read from the sensor,
 *   which is hold in an array
 *  \param cnt : The no of byte of data to be read
 */
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{

//  int i2cReadI2CBlockData(unsigned handle, unsigned i2cReg, char *buf, unsigned count);
    int result = i2cReadI2CBlockData(_HandleBNO, static_cast<unsigned>(reg_addr), static_cast<char *>(static_cast<void *>(reg_data)), static_cast<unsigned>(cnt));

//
//    printf("[RD] ");
//    printf("dev_addr: 0x%02x | ", dev_addr);
//    printf("reg_addr: 0x%02x | ", reg_addr);
//    printf("cnt: 0x%02x | ", cnt);
//    printf("reg_data: ");
//
//      for (int i = 0; i < cnt; i++) {
//        printf("0x%02x ", static_cast<std::uint8_t>(*(reg_data + i)));
//      }
//
//    printf("\n");

    s32 BNO055_iERROR = BNO055_INIT_VALUE;
//    u8 array[I2C_BUFFER_LEN] = { BNO055_INIT_VALUE };
//    u8 stringpos = BNO055_INIT_VALUE;
//
//    array[BNO055_INIT_VALUE] = reg_addr;

    /* Please take the below API as your reference
     * for read the data using I2C communication
     * add your I2C read API here.
     * "BNO055_iERROR = I2C_WRITE_READ_STRING(DEV_ADDR,
     * ARRAY, ARRAY, 1, CNT)"
     * BNO055_iERROR is an return value of SPI write API
     * Please select your valid return value
     * In the driver BNO055_SUCCESS defined as 0
     * and FAILURE defined as -1
     */

//    for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
//    {
//        *(reg_data + stringpos) = array[stringpos];
//    }


    if (result > 0) {
      BNO055_iERROR = BNO055_SUCCESS;
    } else {
      BNO055_iERROR = BNO055_ERROR;
    }

    return (s8)BNO055_iERROR;
}

/*  Brief : The delay routine
 *  \param : delay in ms
 */
void BNO055_delay_msek(u32 msek)
{
//  printf("[DL] ");
//  printf("msek: %d\n", msek);

    /*Here you can write your own delay routine*/

  std::this_thread::sleep_for(std::chrono::milliseconds(msek));
//    gpioSleep(PI_TIME_RELATIVE, 0, static_cast<int>(1000*msek));

}

#endif
