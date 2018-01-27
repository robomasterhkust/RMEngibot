/**
 * This is device realize "read through write" paradigm. This is not
 * standard, but most of I2C devices use this paradigm.
 * You must write to device reading address, send restart to bus,
 * and then begin reading process.
 */

#include "ch.h"
#include "hal.h"

//#include "telemetry.h"
#include "mpu6500.h"
#include "flash.h"
#include "math_misc.h"
#include "chprintf.h"
/* C libraries: */
#include <string.h>

#define MPU6500_RX_BUF_SIZE       0x0E
#define MPU6500_TX_BUF_SIZE       0x05

/* MPU6050 useful registers */
#define MPU6500_SMPLRT_DIV        0x19
#define MPU6500_CONFIG            0x1A
#define MPU6500_GYRO_CONFIG       0x1B
#define MPU6500_ACCEL_CONFIG_1    0x1C
#define MPU6500_ACCEL_CONFIG_2    0x20
#define MPU6500_ACCEL_XOUT_H      0x3B
#define MPU6500_ACCEL_XOUT_L      0x3C
#define MPU6500_ACCEL_YOUT_H      0x3D
#define MPU6500_ACCEL_YOUT_L      0x3E
#define MPU6500_ACCEL_ZOUT_H      0x3F
#define MPU6500_ACCEL_ZOUT_L      0x40
#define MPU6500_TEMP_OUT_H        0x41
#define MPU6500_TEMP_OUT_L        0x42
#define MPU6500_GYRO_XOUT_H       0x43
#define MPU6500_GYRO_XOUT_L       0x44
#define MPU6500_GYRO_YOUT_H       0x45
#define MPU6500_GYRO_YOUT_L       0x46
#define MPU6500_GYRO_ZOUT_H       0x47
#define MPU6500_GYRO_ZOUT_L       0x48
#define MPU6500_PWR_MGMT_1        0x6B

/* I2C read transaction time-out in milliseconds. */
#define MPU6500_READ_TIMEOUT_MS   0x01
/* I2C write transaction time-out in milliseconds. */
#define MPU6500_WRITE_TIMEOUT_MS  0x01

#define MPU6500_SENSOR_RESET      0x80
#define MPU6500_SENSOR_SLEEP      0x40
#define MPU6500_AUTO_SELECT_CLK   0x01

#define MPU6500_SPI_READ          0x80

typedef enum{
  DLPF_250HZ  =  0,
  DLPF_184HZ  =  1,
  DLPF_92HZ   =  2,
  DLPF_41HZ   =  3,
  DLPF_20HZ   =  4,
  DLPF_10HZ   =  5,
  DLPF_5HZ    =  6,
  DLPF_3600HZ =  7
} mpu6500_dlpf_config_t;

typedef enum{
  ADLPF_460HZ =  0,
  ADLPF_184HZ =  1,
  ADLPF_92HZ  =  2,
  ADLPF_41HZ  =  3,
  ADLPF_20HZ  =  4,
  ADLPF_10HZ  =  5,
  ADLPF_5HZ   =  6,
  ADLPF_460HZ =  7
} mpu6500_acc_dlpf_config_t;

static const SPIConfig MPU6500_SPI_cfg =
{
  NULL,
  GPIO_CS,
  GPIO_Pin_CS,
  SPI_CR1_BR_2 | SPI_CR1_MSTR |
  SPI_CR1_CPHA | SPI_CR1_CPOL
};

/* IMU data structure. */
IMUStruct g_IMU1;

/* I2C error info structure. */
I2CErrorStruct g_i2cErrorInfo = {0, 0, 0};

PIMUStruct MPU6500_get(void)
{
  return  &g_IMU1;
}

/**
 * Local variables
 */
/* Data buffers */
static int16_t mpu6050Data[7];

static uint8_t mpu6050RXData[MPU6500_RX_BUF_SIZE];
static uint8_t mpu6050TXData[MPU6500_TX_BUF_SIZE];

/**
 * @brief  Initialization function of IMU data structure.
 * @param  pIMU - pointer to IMU data structure;
 * @param  fAddrLow - IMU address pin A0 is pulled low flag.
 */
static void imuStructureInit(PIMUStruct pIMU, IMUConfigStruct* imu_conf)
{
  memset((void *)pIMU, 0, sizeof(IMUStruct));
  pIMU->imu_spi = imu_conf->imu_spi;
  pIMU->imu_Thd = NULL;

  pIMU->mpu_spi = imu_conf->mpu_spi;
  pIMU->addr = imu_conf->a0_high;

  pIMU->accelT[0][0] = 1.0f;
  pIMU->accelT[1][1] = 1.0f;
  pIMU->accelT[2][2] = 1.0f;
  pIMU->accelT[0][1] = 0.0f;
  pIMU->accelT[1][2] = 0.0f;
  pIMU->accelT[2][0] = 0.0f;
  pIMU->accelT[0][2] = 0.0f;
  pIMU->accelT[1][0] = 0.0f;
  pIMU->accelT[2][1] = 0.0f;

  pIMU->accelBias[0] = 0.0f;
  pIMU->accelBias[1] = 0.0f;
  pIMU->accelBias[2] = 0.0f;

  float flash_test;
  flashRead(IMU_CAL_FLASH, &flash_test, 4);
  if(isfinite(flash_test))
    flashRead(IMU_CAL_FLASH, (char*)(pIMU->accelBias), 60);

  switch(imu_conf->gyroConf)
  {
    case MPU6500_GYRO_SCALE_250:
      pIMU->gyro_psc = (1.0f / 131.0f) * M_PI/180.0f;
      break;
    case MPU6500_GYRO_SCALE_500:
      pIMU->gyro_psc = (1.0f /  65.5f) * M_PI/180.0f;
      break;
    case MPU6500_GYRO_SCALE_1000:
      pIMU->gyro_psc = (1.0f /  32.8f) * M_PI/180.0f;
      break;
    case MPU6500_GYRO_SCALE_2000:
      pIMU->gyro_psc = (1.0f /  16.4f) * M_PI/180.0f;
      break;
  }

  switch(imu_conf->accelConf)
  {
    case MPU6500_ACCEL_SCALE_2G:
      pIMU->accel_psc = (GRAV / 16384.0f);
      break;
    case MPU6500_ACCEL_SCALE_4G:
      pIMU->accel_psc = (GRAV /  8192.0f);
      break;
    case MPU6500_ACCEL_SCALE_8G:
      pIMU->accel_psc = (GRAV /  4096.0f);
      break;
    case MPU6500_ACCEL_SCALE_16G:
      pIMU->accel_psc = (GRAV /  2048.0f);
      break;
  }
}

static void trans_accel_offset(PIMUStruct pIMU, float accelData[3])
{
  float accelData_temp[3];

  accelData_temp[X] = accelData[X] - pIMU->accelBias[X];
  accelData_temp[Y] = accelData[Y] - pIMU->accelBias[Y];
  accelData_temp[Z] = accelData[Z] - pIMU->accelBias[Z];

  matrix33_multiply_vector3(pIMU->accelT, accelData_temp, pIMU->accelData);
}

/**
 * @brief  Reads new data from the sensor
 * @param  pIMU - pointer to IMU data structure;
 * @return 1 - if reading was successful;
 *         0 - if reading failed.
 */
uint8_t mpu6050GetData(PIMUStruct pIMU)
{
  uint32_t tcurr = chVTGetSystemTimeX();
  pIMU->dt = ST2US(tcurr - pIMU->tprev)/1000000.0f;
  pIMU->tprev = tcurr;

  float accelData[3],gyroData[3];
  uint8_t error =  mpu6050GetDataRaw(pIMU, accelData, gyroData);

  if(!error)
  {
    trans_accel_offset(pIMU, accelData);
    pIMU->gyroData[X] = gyroData[X];
    pIMU->gyroData[Y] = gyroData[Y];
    pIMU->gyroData[Z] = gyroData[Z];
  }

  return error;
}

/**
 * @brief  Reads new data from the sensor
 * @param  pIMU - pointer to IMU data structure;
 * @return 1 - if reading was successful;
 *         0 - if reading failed.
 */
uint8_t mpu6050GetDataRaw(PIMUStruct pIMU, float AccelRaw[3], float GyroRaw[3])
{
  msg_t status = MSG_OK;

  /* Set the start register address for bulk data transfer. */
  mpu6050TXData[0] = MPU6500_ACCEL_XOUT_H | MPU6500_SPI_READ;
  spiAcquireBus(pIMU->mpu_spi);
  spiSend(pIMU->mpu_spi, 1, mpu6050TXData);
  spiReceive(pIMU->mpu_spi, 14, mpu6050RXData);
	spiReleaseBus(pIMU->mpu_spi);

  mpu6050Data[0] = (int16_t)((mpu6050RXData[ 0]<<8) | mpu6050RXData[ 1]); /* Accel X */
  mpu6050Data[1] = (int16_t)((mpu6050RXData[ 2]<<8) | mpu6050RXData[ 3]); /* Accel Y */
  mpu6050Data[2] = (int16_t)((mpu6050RXData[ 4]<<8) | mpu6050RXData[ 5]); /* Accel Z */
  mpu6050Data[3] = (int16_t)((mpu6050RXData[ 8]<<8) | mpu6050RXData[ 9]); /* Gyro X  */
  mpu6050Data[4] = (int16_t)((mpu6050RXData[10]<<8) | mpu6050RXData[11]); /* Gyro Y  */
  mpu6050Data[5] = (int16_t)((mpu6050RXData[12]<<8) | mpu6050RXData[13]); /* Gyro Z  */
  mpu6050Data[6] = (int16_t)((mpu6050RXData[6]<<8) | mpu6050RXData[7]); /* Temperature */

  /* X: */
  AccelRaw[X] = (float)mpu6050Data[0] * pIMU->accel_psc;
  GyroRaw[X]  = (float)mpu6050Data[3] * pIMU->gyro_psc;

  /* Y: */
  AccelRaw[Y] = (float)mpu6050Data[1] * pIMU->accel_psc;
  GyroRaw[Y]  = (float)mpu6050Data[4] * pIMU->gyro_psc;

  /* Z: */
  AccelRaw[Z] = (float)mpu6050Data[2] * pIMU->accel_psc;
  GyroRaw[Z]  = (float)mpu6050Data[5] * pIMU->gyro_psc;

  return IMU_OK;
}

/**
 * @brief  Initialization function for the MPU6050 sensor.
 * @param  addr - I2C address of MPU6050 chip.
 * @return 1 - if initialization was successful;
 *         0 - if initialization failed.
 */
uint8_t mpu6050Init(PIMUStruct pIMU, const IMUConfigStruct* const imu_conf)
{
  msg_t status = MSG_OK;

  imuStructureInit(pIMU, imu_conf);

  spiStart(pIMU->mpu_spi, &MPU6500_SPI_cfg);

  /* Reset all MPU6050 registers to their default values */
  mpu6050TXData[0] = MPU6500_PWR_MGMT_1;  // Start register address;
  mpu6050TXData[1] = MPU6500_SENSOR_RESET;

  spiAcquireBus(pIMU->mpu_spi);
  spiSend(pIMU->mpu_spi, 2, mpu6050TXData);
  spiReleaseBus(pIMU->mpu_spi);

  /* Wait 100 ms for the MPU6050 to reset */
  chThdSleepMilliseconds(100);

  /* Clear the SLEEP flag, set the clock and start measuring. */
  mpu6050TXData[0] = MPU6500_PWR_MGMT_1;  // Start register address;
  mpu6050TXData[1] = MPU6500_AUTO_SELECT_CLK;

  spiAcquireBus(pIMU->mpu_spi);
  spiSend(pIMU->mpu_spi, 2, mpu6050TXData);
  spiReleaseBus(pIMU->mpu_spi);


  /* Configure the MPU6050 sensor        */
  /* NOTE:                               */
  /* - SLEEP flag must be cleared before */
  /*   configuring the sensor.           */
  mpu6050TXData[0] = MPU6500_CONFIG;  // Start register address;
  mpu6050TXData[1] = DLPF_41HZ;          // CONFIG register value DLPF_CFG;
  mpu6050TXData[2] = (uint8_t)(imu_conf->gyroConf << 3U);          // GYRO_CONFIG register value
  mpu6050TXData[3] = (uint8_t)(imu_conf->accelConf << 3U);          // ACCEL_CONFIG_1 register value
  mpu6050TXData[4] = ADLPF_41HZ;          // ACCEL_CONFIG_2 register value

  spiAcquireBus(pIMU->mpu_spi);
  spiSend(pIMU->mpu_spi, 5, mpu6050TXData);
  spiReleaseBus(pIMU->mpu_spi);

  pIMU->tprev = chVTGetSystemTimeX();
  pIMU->inited = 1;
  return IMU_OK;
}
