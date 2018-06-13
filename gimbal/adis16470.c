//Created by Alex Wong on 2/6/2018
//Tested on 2018 Type-A RM Dev Board
//
//Connection:
//MISO PE5
//MOSI PE6
//NSS  PE4
//SEK  PE12
//

#include "ch.h"
#include "hal.h"
#include "adis16470.h"
#include "math_misc.h"
#include "string.h"

#include "system_error.h"

//===DO NOT XIA JB CHANGE THIS=====
#define IMU_DATA_X 0U
#define IMU_DATA_Y 1U
#define IMU_DATA_Z 2U
//=================================

#define ADIS16470_BURST_SIZE         10U
#define ADIS16470_MAX_ERROR_COUNT    20U

#define ADIS16470_BURST_CMD                    0x6800
#define ADIS16470_X_GYRO_LOW                   0x04
#define ADIS16470_X_GYRO_OUT                   0x06
#define ADIS16470_Y_GYRO_LOW                   0x08
#define ADIS16470_Y_GYRO_OUT                   0x0A
#define ADIS16470_Z_GYRO_LOW                   0x0C
#define ADIS16470_Z_GYRO_OUT                   0x0E
#define ADIS16470_X_ACCL_LOW                   0x10
#define ADIS16470_X_ACCL_OUT                   0x12
#define ADIS16470_Y_ACCL_LOW                   0x14
#define ADIS16470_Y_ACCL_OUT                   0x16
#define ADIS16470_Z_ACCL_LOW                   0x18
#define ADIS16470_Z_ACCL_OUT                   0x1A

#define ADIS16470_XG_BIAS_LOW                   0x40
#define ADIS16470_XG_BIAS_HIGH                  0x42
#define ADIS16470_YG_BIAS_LOW                   0x44
#define ADIS16470_YG_BIAS_HIGH                  0x46
#define ADIS16470_ZG_BIAS_LOW                   0x48
#define ADIS16470_ZG_BIAS_HIGH                  0x4A
#define ADIS16470_XA_BIAS_LOW                   0x4C
#define ADIS16470_XA_BIAS_HIGH                  0x4E
#define ADIS16470_YA_BIAS_LOW                   0x50
#define ADIS16470_YA_BIAS_HIGH                  0x52
#define ADIS16470_ZA_BIAS_LOW                   0x54
#define ADIS16470_ZA_BIAS_HIGH                  0x56

#define ADIS16470_FILT_CTRL                     0x5C
#define ADIS16470_MSC_CTRL                      0x60
#define ADIS16470_GLOB_CMD                      0x68

#define ADIS16470_PROD_ID                     0x4056 //16470
#define ADIS16470_PROD_ID_ADDR                0x7200
#define ADIS16470_GYRO_CAL_ID                   0x76
#define ADIS16470_ACCL_CAL_ID                   0x77 //Store the calibration id data
                                                     //NOTE: see explanation in adis16470.h line.69

typedef struct
{
  uint16_t diag_stat;                   //For diagnostic
  int16_t gyroData[3];                  //1LSB = 0.1deg/s
  int16_t accelData[3];                 //1LSB = 1.25mg
  int16_t  temperature;                 //1LSB = 0.1deg C
  uint16_t data_cnt;                    //1LSB = 500us
  uint16_t  checksum : 8;               //16-bit checksum
  uint16_t  res      : 8;               //reserved
}__attribute__((packed)) adis16470_data_t;

static adis16470_data_t burst_data;
static uint8_t error_counter;
static adis16470Struct adis16470;

static uint8_t axis_conf[3];
static bool    axis_rev[3];
static adis16265_conf_t conf;
static uint16_t onboard_calibration_id;  //Calibration id retrieved on sensor;

static const SPIConfig adis16470SpiCfg = {

  NULL,
  ADIS16470_port,
  ADIS16470_pin,
  SPI_CR1_MSTR | SPI_CR1_DFF | SPI_CR1_BR_2 | SPI_CR1_BR_0 | SPI_CR1_CPHA | SPI_CR1_CPOL

};

adis16470Struct* adis16470_get(void)
{
  return &adis16470;
}

uint16_t adis16470_get_error(void)
{
  return adis16470.error;
}

void adis16470_clear_error(void)
{
  adis16470.error &= ~(ADIS16470_SENSOR_ERROR |
                       ADIS16470_UNCONNECTED |
                       ADIS16470_DATA_INVALID);
}


static uint16_t readword(uint8_t address)
{
  uint16_t data = 0;
  data = ((uint16_t)address) << 8;

  spiAcquireBus(ADIS16470_SPID);            //request from user specified address
  spiSelect(ADIS16470_SPID);
  spiSend(ADIS16470_SPID, 1, &data);
  spiUnselect(ADIS16470_SPID);

  chThdSleep(US2ST(16));                    //stall time specified by data sheet

  spiSelect(ADIS16470_SPID);                //read word
  spiReceive(ADIS16470_SPID, 1, &data);
  spiUnselect(ADIS16470_SPID);
  spiReleaseBus(ADIS16470_SPID);

  chThdSleep(US2ST(16));                    //stall time specified by data sheet

  return data;
}

static void writeword(uint8_t lowerAddress, uint16_t data)
{
  uint16_t txbuffer[2];
  //concatenate write bit, address and data to be written
  txbuffer[0] = 0x8000 | lowerAddress << 8 | (uint8_t)data;     //pack lower word
  txbuffer[1] = 0x8000 | (lowerAddress + 1) << 8 | data >> 8;   //pack higher word

  spiAcquireBus(ADIS16470_SPID);
  spiSelect(ADIS16470_SPID);
  spiSend(ADIS16470_SPID, 1, &txbuffer);        //send lower word
  spiUnselect(ADIS16470_SPID);

  chThdSleep(US2ST(16));                        //stall time specified by data sheet

  spiSelect(ADIS16470_SPID);
  spiSend(ADIS16470_SPID, 1, &txbuffer[1]);     //send higher word
  spiUnselect(ADIS16470_SPID);
  spiReleaseBus(ADIS16470_SPID);

  chThdSleep(US2ST(16));                    //stall time specified by data sheet
}

static bool burstRead(void)
{
  static const uint16_t burst_cmd = ADIS16470_BURST_CMD;

  spiAcquireBus(ADIS16470_SPID);            //request burst read
  spiSelect(ADIS16470_SPID);
  spiSend(ADIS16470_SPID, 1, &burst_cmd);

  spiReceive(ADIS16470_SPID, ADIS16470_BURST_SIZE, &burst_data);    //receive data
  spiUnselect(ADIS16470_SPID);
  spiReleaseBus(ADIS16470_SPID);

  uint8_t i = 0;                            //calculate checksum
  uint8_t checksum = 0;
  uint8_t* buf_addr = (uint8_t*)&burst_data;
  for (i = 0; i < ADIS16470_BURST_SIZE*2 - 2; i++)
    checksum += buf_addr[i];

  return checksum != burst_data.checksum;
}

void adis16470_set_calibration_id(uint8_t calibration_id_accl, uint8_t calibration_id_gyro)
{
  uint16_t new_calibration_id = onboard_calibration_id;
  if(calibration_id_gyro)
  {
    new_calibration_id &= 0xFF00;
    new_calibration_id |= calibration_id_gyro;
  }
  if(calibration_id_accl)
  {
    new_calibration_id &= 0x00FF;
    new_calibration_id |= ((uint16_t)calibration_id_accl << 8);
  }

  writeword(ADIS16470_GYRO_CAL_ID, new_calibration_id);
}

void adis16470_reset_calibration(void)
{
  writeword(ADIS16470_GLOB_CMD, 0x0002);          //Reset all bias
}

void adis16470_bias_update(int32_t accelBias[3], int32_t gyroBias[3])
{
  writeword(ADIS16470_XG_BIAS_LOW , (uint16_t)(gyroBias[X]));
  writeword(ADIS16470_XG_BIAS_HIGH, (uint16_t)(gyroBias[X] >> 16));
  writeword(ADIS16470_YG_BIAS_LOW , (uint16_t)(gyroBias[Y]));
  writeword(ADIS16470_YG_BIAS_HIGH, (uint16_t)(gyroBias[Y] >> 16));
  writeword(ADIS16470_ZG_BIAS_LOW , (uint16_t)(gyroBias[Z]));
  writeword(ADIS16470_ZG_BIAS_HIGH, (uint16_t)(gyroBias[Z] >> 16));

  writeword(ADIS16470_XA_BIAS_LOW , (uint16_t)(accelBias[X]));
  writeword(ADIS16470_XA_BIAS_HIGH, (uint16_t)(accelBias[X] >> 16));
  writeword(ADIS16470_YA_BIAS_LOW , (uint16_t)(accelBias[Y]));
  writeword(ADIS16470_YA_BIAS_HIGH, (uint16_t)(accelBias[Y] >> 16));
  writeword(ADIS16470_ZA_BIAS_LOW , (uint16_t)(accelBias[Z]));
  writeword(ADIS16470_ZA_BIAS_HIGH, (uint16_t)(accelBias[Z] >> 16));

  chThdSleepMilliseconds(200);

  writeword(ADIS16470_GLOB_CMD, 0x0008);          //save to flash
  chThdSleepSeconds(2);

  //Reset and re-config the sensor
  ADIS_RST();
  chThdSleepMilliseconds(100);
  ADIS_RST_RELEASE();

  uint16_t mscCtrl = readword(ADIS16470_MSC_CTRL);                          //read MSC_CTRL register
  mscCtrl = (mscCtrl & (~0x001C)) | ADIS16470_INTERNAL_CLK;   //set bits [2:4], mask the rest
  writeword(ADIS16470_MSC_CTRL, mscCtrl);                                   //set MSC_CTRL with new command

  //set filter to be the same in beck's code
  writeword(ADIS16470_FILT_CTRL, 0x0005);
}

void adis16470_get_gyro_raw(int32_t gyroRawData[3])
{
  gyroRawData[X] = readword(ADIS16470_X_GYRO_LOW)
        | (readword(ADIS16470_X_GYRO_OUT) << 16);
  gyroRawData[Y] = readword(ADIS16470_Y_GYRO_LOW)
        | (readword(ADIS16470_Y_GYRO_OUT) << 16);
  gyroRawData[Z] = readword(ADIS16470_Z_GYRO_LOW)
        | (readword(ADIS16470_Z_GYRO_OUT) << 16);
}

void adis16470_get_accel_raw(int32_t accelRawData[3])
{
  accelRawData[X] = readword(ADIS16470_X_ACCL_LOW)
        | (readword(ADIS16470_X_ACCL_OUT) << 16);
  accelRawData[Y] = readword(ADIS16470_Y_ACCL_LOW)
        | (readword(ADIS16470_Y_ACCL_OUT) << 16);
  accelRawData[Z] = readword(ADIS16470_Z_ACCL_LOW)
        | (readword(ADIS16470_Z_ACCL_OUT) << 16);
}

void adis16470_get_gyro_bias(int32_t gyroBias[3])
{
  gyroBias[X] = readword(ADIS16470_XG_BIAS_LOW)
        | (readword(ADIS16470_XG_BIAS_HIGH) << 16);
  gyroBias[Y] = readword(ADIS16470_YG_BIAS_LOW)
        | (readword(ADIS16470_YG_BIAS_HIGH) << 16);
  gyroBias[Z] = readword(ADIS16470_ZG_BIAS_LOW)
        | (readword(ADIS16470_ZG_BIAS_HIGH) << 16);
}

void adis16470_get_accel_bias(int32_t accelBias[3])
{
  accelBias[X] = readword(ADIS16470_XA_BIAS_LOW)
        | (readword(ADIS16470_XA_BIAS_HIGH) << 16);
  accelBias[Y] = readword(ADIS16470_YA_BIAS_LOW)
        | (readword(ADIS16470_YA_BIAS_HIGH) << 16);
  accelBias[Z] = readword(ADIS16470_ZA_BIAS_LOW)
        | (readword(ADIS16470_ZA_BIAS_HIGH) << 16);
}

static void adis16470_update(void)
{
  if(burstRead())
  {
    adis16470.error |= ADIS16470_DATA_INVALID;
    error_counter++;
    return;
  }
  else
    adis16470.diag_stat = burst_data.diag_stat;

  if(!adis16470.diag_stat) //TODO Add diagnostics and system error
  {
    chSysLock();

    if(axis_rev[X])
    {
      adis16470.gyroData[X] = -burst_data.gyroData[axis_conf[X]]
        * ADIS16470_GYRO_DATA_PSC_16;
      adis16470.accelData[X] = -burst_data.accelData[axis_conf[X]]
        * ADIS16470_ACCEL_DATA_PSC_16;
    }
    else
    {
      adis16470.gyroData[X] = burst_data.gyroData[axis_conf[X]]
        * ADIS16470_GYRO_DATA_PSC_16;
      adis16470.accelData[X] = burst_data.accelData[axis_conf[X]]
        * ADIS16470_ACCEL_DATA_PSC_16;
    }

    if(axis_rev[Y])
    {
      adis16470.gyroData[Y] = -burst_data.gyroData[axis_conf[Y]]
        * ADIS16470_GYRO_DATA_PSC_16;
      adis16470.accelData[Y] = -burst_data.accelData[axis_conf[Y]]
        * ADIS16470_ACCEL_DATA_PSC_16;
    }
    else
    {
      adis16470.gyroData[Y] = burst_data.gyroData[axis_conf[Y]]
        * ADIS16470_GYRO_DATA_PSC_16;
      adis16470.accelData[Y] = burst_data.accelData[axis_conf[Y]]
        * ADIS16470_ACCEL_DATA_PSC_16;
    }

    if(axis_rev[Z])
    {
      adis16470.gyroData[Z] = -burst_data.gyroData[axis_conf[Z]]
        * ADIS16470_GYRO_DATA_PSC_16;
      adis16470.accelData[Z] = -burst_data.accelData[axis_conf[Z]]
        * ADIS16470_ACCEL_DATA_PSC_16;
    }
    else
    {
      adis16470.gyroData[Z] = burst_data.gyroData[axis_conf[Z]]
        * ADIS16470_GYRO_DATA_PSC_16;
      adis16470.accelData[Z] = burst_data.accelData[axis_conf[Z]]
        * ADIS16470_ACCEL_DATA_PSC_16;
    }

    adis16470.temperature = burst_data.temperature * ADIS16470_TEMP_PSC;

    static uint32_t stamp_upper16; //Timestamp upper 16 bit
    static uint16_t prev_data_cnt;

    if(prev_data_cnt > burst_data.data_cnt)
      stamp_upper16++;

    uint32_t stamp = stamp_upper16 << 16 | burst_data.data_cnt; //Update timestamp
    prev_data_cnt = burst_data.data_cnt;
    int32_t dt = stamp - adis16470.stamp;

    if(dt > 50 || dt < 0) //Indicates incorrect timestamp
    {
      adis16470.error |= ADIS16470_DATA_INVALID;
      system_setTempWarningFlag();
      error_counter++;
    }
    else
    {
      adis16470.error &= ~ADIS16470_DATA_INVALID;
      error_counter = 0;
    }
    adis16470.stamp = stamp;

    chSysUnlock();
  }
  else
  {
    system_setErrorFlag();
    adis16470.error |= ADIS16470_SENSOR_ERROR;
  }

  if(error_counter > ADIS16470_MAX_ERROR_COUNT)
  {
    system_setErrorFlag();
    adis16470.error |= ADIS16470_READING_ERROR;
  }
}

#define adis16470_update_CBR()    (writeword(ADIS16470_GLOB_CMD, 0x0001))

static THD_WORKING_AREA(adis16470Thd_wa, 1024);
static THD_FUNCTION(adis16470Thd, p) {

  (void)p;

  //set clock source
  uint16_t mscCtrl = readword(ADIS16470_MSC_CTRL);                          //read MSC_CTRL register
  mscCtrl = (mscCtrl & (~0x001C)) | ADIS16470_INTERNAL_CLK;   //set bits [2:4], mask the rest
  writeword(ADIS16470_MSC_CTRL, mscCtrl);                                   //set MSC_CTRL with new command

  //set filter to be the same in beck's code
  writeword(ADIS16470_FILT_CTRL, 0x0005);

  adis16470.state = ADIS16470_READY;

  uint32_t count = 1;
  while(!chThdShouldTerminateX())
  {
    if(adis16470.state == ADIS16470_CALIBRATING)
    {
      chSysLock();
      chThdSuspendS(&adis16470.imu_Thd);
      chSysUnlock();
    }
    else
    {
      if(adis16470.error & (ADIS16470_ERROR | ADIS16470_DATA_INVALID))
        adis16470.state = ADIS16470_NOT_READY;
      else
        adis16470.state = ADIS16470_READY;
    }

    if(!(count % (ADIS16470_SAMPLE_FREQ * ADIS16470_BIAS_UPDATE_PERIOD_S)))
      adis16470_update_CBR();

    adis16470_update();
    chThdSleep(ADIS16470_UPDATE_PERIOD);
  }
}

static uint8_t verify_configuration(const adis16265_conf_t* conf)
{
  int8_t x_axis_vector[3] = {0, 0, 0};
  int8_t y_axis_vector[3] = {0, 0, 0};
  int8_t z_axis_vector[3] = {0, 0, 0};
  uint8_t i;

  /* check whether the axis conf value is valid */
  for(i = 0; i < 7U; i++)
  {
    if(conf->x_axis == (1U << i))  break;
    else if(i == 6U)  return 1;
  }
  for(i = 0; i < 7U; i++)
  {
    if(conf->y_axis == (1U << i))  break;
    else if(i == 6U)  return 1;
  }
  for(i = 0; i < 7U; i++)
  {
    if(conf->z_axis == (1U << i))  break;
    else if(i == 6U)  return 1;
  }

  /* Verify that the configured coordinate is right-handed */
  for (i = 0; i < 3U; i++)
  {
    if(conf->x_axis == (0b010000 >> 2*i))  x_axis_vector[i] = 1;
    else if(conf->x_axis == (0b100000 >> 2*i))  x_axis_vector[i] = -1;
  }
  for (i = 0; i < 3U; i++)
  {
    if(conf->y_axis == (0b010000 >> 2*i))  y_axis_vector[i] = 1;
    else if(conf->y_axis == (0b100000 >> 2*i))  y_axis_vector[i] = -1;
  }
  for (i = 0; i < 3U; i++)
  {
    if(conf->z_axis == (0b010000 >> 2*i))  z_axis_vector[i] = 1;
    else if(conf->z_axis == (0b100000 >> 2*i))  z_axis_vector[i] = -1;
  }

  //Do the cross-product and check result
  int8_t result[3] =
  {
    x_axis_vector[1]*y_axis_vector[2] - x_axis_vector[2]*y_axis_vector[1],
    x_axis_vector[2]*y_axis_vector[0] - x_axis_vector[0]*y_axis_vector[2],
    x_axis_vector[0]*y_axis_vector[1] - x_axis_vector[1]*y_axis_vector[0]
  };

  if(result[0] == z_axis_vector[0] &&
     result[1] == z_axis_vector[1] &&
     result[2] == z_axis_vector[2])
    return 0;
  else
    return 1;
}

void adis16470_init(const adis16265_conf_t* sensor_conf) {

  //check whether the configuration is valid
  if(verify_configuration(sensor_conf))
  {
    adis16470.state |= ADIS16470_AXIS_CONF_ERROR;
    system_setErrorFlag();
    return;
  }

  ADIS_RST();
  chThdSleepMilliseconds(200);
  ADIS_RST_RELEASE();

  uint8_t i;
  for (i = 0; i < 3U; i++)
  {
    if(sensor_conf->x_axis & (0b110000 >> 2*i))
      axis_conf[X] = i;
    if(sensor_conf->x_axis & (0b100000 >> 2*i))
      axis_rev[X] = true;
  }
  for (i = 0; i < 3U; i++)
  {
    if(sensor_conf->y_axis & (0b110000 >> 2*i))
      axis_conf[Y] = i;
    if(sensor_conf->y_axis & (0b100000 >> 2*i))
      axis_rev[Y] = true;
  }
  for (i = 0; i < 3U; i++)
  {
    if(sensor_conf->z_axis & (0b110000 >> 2*i))
      axis_conf[Z] = i;
    if(sensor_conf->z_axis & (0b100000 >> 2*i))
      axis_rev[Z] = true;
  }

  (*ADIS16470_SPID).rxdmamode |= STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD;
  (*ADIS16470_SPID).txdmamode |= STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD;
  spiStart(ADIS16470_SPID, &adis16470SpiCfg);
  chThdSleepMilliseconds(100);
  uint16_t prod_id = 0;
  i = 0;

/*
  do{
    uint16_t tx = ADIS16470_PROD_ID_ADDR;

    spiAcquireBus(ADIS16470_SPID);            //request burst read
    spiSelect(ADIS16470_SPID);
    spiExchange(ADIS16470_SPID, 1, &tx, &prod_id);
    spiUnselect(ADIS16470_SPID);
    spiReleaseBus(ADIS16470_SPID);

    chThdSleepMilliseconds(1);
    if(i++ > ADIS16470_MAX_ERROR_COUNT)
    {
      system_setErrorFlag();
      adis16470.error |= ADIS16470_READING_ERROR;
      return;
    }
  } while(prod_id != ADIS16470_PROD_ID);


  uint16_t calibration_id = readword(ADIS16470_GYRO_CAL_ID);
  if((uint8_t)calibration_id != sensor_conf->calibration_id)
  {
    system_setWarningFlag();
    adis16470.error |= ADIS16470_GYRO_NOT_CALIBRATED;
  }
  if((uint8_t)(calibration_id >> 8) != sensor_conf->calibration_id)
  {
    system_setWarningFlag();
    adis16470.error |= ADIS16470_ACCEL_NOT_CALIBRATED;
  }
*/

  uint16_t calibration_id = readword(ADIS16470_GYRO_CAL_ID);
  adis16470.calibration_id = sensor_conf->calibration_id;
  onboard_calibration_id = calibration_id;
  chThdCreateStatic(adis16470Thd_wa, sizeof(adis16470Thd_wa),
                    NORMALPRIO + 7, adis16470Thd, NULL);

}
