//
//Created by Alex Wong on 2/6/2018
//

#ifndef INC_ADIS16470_H_
#define INC_ADIS16470_H_

#define ADIS16470_SPID          &SPID4
#define IMU_USE_EULER_ANGLE

#define ADIS16470_pin           GPIOE_SPI4_ADIS_NSS
#define ADIS16470_port          GPIOE

#define ADIS16470_RST_PIN           GPIOB_PIN1
#define ADIS16470_RST_PORT          GPIOB

#define ADIS16470_INTERNAL_CLK  0x0000
#define ADIS16470_EXTERNAL_CLK  0x0008

#define ADIS16470_UPDATE_PERIOD US2ST(500)
#define ADIS16470_SAMPLE_FREQ               2000U
#define ADIS16470_BIAS_UPDATE_PERIOD_S        32U

#define ADIS16470_GYRO_DATA_PSC_16         1.745329252e-3f
#define ADIS16470_ACCEL_DATA_PSC_16        1.25e-3f * GRAV
#define ADIS16470_GYRO_DATA_PSC_32         2.66316109e-8f
#define ADIS16470_ACCEL_DATA_PSC_32        1.907348633e-8f * GRAV
#define ADIS16470_TEMP_PSC                 0.1f

//==========================================================================//
//NOTE DO NOT CHANGE THIS! NOTE DO NOT CHANGE THIS! NOTE DO NOT CHANGE THIS!
//==========================================================================//
typedef enum {
  ADIS16470_X =     0b010000,
  ADIS16470_Y =     0b000100,
  ADIS16470_Z =     0b000001,
  ADIS16470_X_REV = 0b100000,
  ADIS16470_Y_REV = 0b001000,
  ADIS16470_Z_REV = 0b000010
} adis16470_axis_conf_t;
//=============================================================================//
//Otherwise your planetary system will be invaded, your world will be annexed!!//
//=============================================================================//

typedef enum
{
  ADIS16470_UNINIT               = 0,
  ADIS16470_READY,
  ADIS16470_CALIBRATING,
  ADIS16470_NOT_READY
} adis16470_state_t;

typedef enum
{
  ADIS16470_ERROR                = 0xFF00,
  ADIS16470_SENSOR_ERROR         = 0x1000,
  ADIS16470_AXIS_CONF_ERROR      = 0x2000,
  ADIS16470_READING_ERROR        = 0x4000,
  ADIS16470_UNCONNECTED          = 0x8000,
  ADIS16470_WARNING              = 0x00FF,
  ADIS16470_ACCEL_NOT_CALIBRATED = 0x0010,
  ADIS16470_GYRO_NOT_CALIBRATED  = 0x0020,
  ADIS16470_DATA_INVALID         = 0x0040,
  ADIS16470_LOSE_FRAME           = 0x0080,
  ADIS16470_CORRUPT_Q_DATA       = 0x0001,
} adis16470_error_t;

typedef struct{
  uint8_t calibration_id;
  //Set an 8-bit data as the indication of recent calibration
  //NOTE: If this calibration_id in configuration does not match the on-board data,
  //      it means re-calibration needed
  int8_t x_axis;
  int8_t y_axis;
  int8_t z_axis;  //Used to remap the original axis of ADIS16265

}adis16265_conf_t;

typedef struct data_16470_t{

//==============sensor direct data output===================//
  uint16_t diag_stat;                   //For diagnostic
  volatile float gyroData[3];          //1LSB = 0.1deg/s
  volatile float accelData[3];         //1LSB = 1.25mg
  float temperature;                   //1LSB = 0.1deg C
  volatile uint32_t stamp;
//==============end of sensor data output===================//

  adis16470_state_t state;
  adis16470_error_t error;
  uint8_t calibration_id;

  float qIMU[4];          /* Attitude quaternion of the IMU. */

  #ifdef  IMU_USE_EULER_ANGLE
    float euler_angle[3];      /* Euler angle of the IMU. */
    float d_euler_angle[3];    /* Euler angle changing rate of the IMU. */

    int rev;                 /* used to detect zero-crossing */
    float prev_yaw; //TODO change this inside a static global variable
  #else
    float dqIMU[4];         /* Attitude quaternion changing rate of the IMU. */
  #endif

  thread_reference_t imu_Thd;
} adis16470Struct;

typedef adis16470Struct IMUStruct, *PIMUStruct;

typedef enum calRegAddr {

  ADIS16470_GYROSCOPE = 0x40,
  ADIS16470_ACCELEROMETER = 0x4C

}calRegAddr;

#define ADIS_RST()          (palClearPad(ADIS16470_RST_PORT, ADIS16470_RST_PIN))
#define ADIS_RST_RELEASE()  (palSetPad(ADIS16470_RST_PORT, ADIS16470_RST_PIN))

adis16470Struct* adis16470_get(void);

adis16470_error_t adis16470_get_error(void);
void adis16470_clear_error(void);

void adis16470_reset_calibration(void);
void adis16470_set_calibration_id(uint8_t calibration_id_accl, uint8_t calibration_id_gyro);
void adis16470_bias_update(int32_t accelBias[3], int32_t gyroBias[3]);
void adis16470_get_gyro_bias(int32_t gyroBias[3]);
void adis16470_get_accel_bias(int32_t accelBias[3]);
void adis16470_get_accel_raw(int32_t accelRawData[3]);
void adis16470_get_gyro_raw(int32_t gyroRawData[3]);

void adis16470_init(const adis16265_conf_t* sensor_conf);

#endif /* INC_ADIS16470_H_ */
