/**
 * Edward ZHANG
 * @file    shellcfg.c
 * @brief   definitions of shell command functions
 */
#include "main.h"
#include "shell.h"
#include <string.h>
#include "rangefinder.h"
#include "island.h"
#define SERIAL_CMD       &SDU1
#define SERIAL_DATA      &SDU1

static thread_t* matlab_thread_handler = NULL;
/**
 * @brief Transmit uint32_t and float through serial port to host machine
 * @require Initialization of ChibiOS serial driver before using this function
 *
 * @param[in] chp         pointer to a @p BaseSequentialStream implementing object
 * @param[in] txbuf_d     array of 32-bit integers to tramsmit, can be signed or unsigned
 * @param[in] txbuf_f     array of float point numbers to tramsmit
 * @param[in] num_int     number of 32-bit integers to tramsmit
 * @param[in] num_float   number of float point numbers to tramsmit
 *
 * @TODO improve the transmission protocol to enable easier setup for the host machine
 */
#define SYNC_SEQ  0xaabbccdd
static void transmit_matlab
  (BaseSequentialStream* chp,
    uint32_t* const txbuf_d, float* const txbuf_f,
    const uint8_t num_int, const uint8_t num_float)
{
  uint32_t sync = SYNC_SEQ;
  char* byte = (char*)&sync;

  uint8_t i;
  for (i = 0; i < 4; i++)
    chSequentialStreamPut(chp, *byte++);

  byte = (char*)txbuf_d;
  for (i = 0; i < 4*num_int; i++)
    chSequentialStreamPut(chp, *byte++);

  byte = (char*)txbuf_f;
  for (i = 0; i < 4*num_float; i++)
    chSequentialStreamPut(chp, *byte++);
}

#define HOST_TRANSMIT_FREQ  100U
static THD_WORKING_AREA(matlab_thread_wa, 512);
static THD_FUNCTION(matlab_thread, p)
{
  (void)p;
  chRegSetThreadName("matlab tramsmitter");

  int32_t txbuf_d[16];
  float txbuf_f[16];
  BaseSequentialStream* chp = (BaseSequentialStream*)SERIAL_DATA;

  PIMUStruct PIMU = imu_get();
  chassisStruct* chassis = chassis_get();
//  GimbalStruct* gimbal = gimbal_get();

  uint32_t tick = chVTGetSystemTimeX();
  const uint16_t period = US2ST(1000000/HOST_TRANSMIT_FREQ);
  while (!chThdShouldTerminateX())
  {
    tick += period;
    if(tick > chVTGetSystemTimeX())
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }

    txbuf_f[0] = PIMU->euler_angle[Roll];
    txbuf_f[1] = PIMU->euler_angle[Pitch];
    txbuf_f[2] = PIMU->euler_angle[Yaw];

    transmit_matlab(chp, NULL, txbuf_f, 0, 3);
  }
}

void enable_all_rangefinder(){
  island_robotSetState(STATE_TEST);
  int i ;
  for( i = 0 ; i < RANGEFINDER_NUM ; i++)
    rangeFinder_control(i, ENABLE);
}

void disable_all_rangefinder(){
  int i ;
  for( i = 0 ; i < RANGEFINDER_NUM ; i++)
    rangeFinder_control(i, DISABLE);
  island_robotSetState(STATE_GROUND);
}
/*===========================================================================*/
/* Definitions of shell command functions                                    */
/*===========================================================================*/
static THD_WORKING_AREA(Shell_thread_wa, 1024);
void cmd_test(BaseSequentialStream * chp, int argc, char *argv[])
{
  (void) argc,argv;
  PIMUStruct pIMU = imu_get();
  PGyroStruct PGyro = gyro_get();
  chassisStruct* chassis = chassis_get();

  chprintf(chp, "AccelX: %f\r\n", pIMU->accelData[X]);
  chprintf(chp, "AccelY: %f\r\n", pIMU->accelData[Y]);
  chprintf(chp, "AccelZ: %f\r\n", pIMU->accelData[Z]);

  chprintf(chp, "Roll: %f\r\n", pIMU->euler_angle[Roll]);
  chprintf(chp, "Pitch: %f\r\n", pIMU->euler_angle[Pitch]);
  chprintf(chp, "Yaw: %f\r\n", pIMU->euler_angle[Yaw]);

  chprintf(chp, "motors0: %f\r\n", chassis->_motors[0].speed_sp);
  chprintf(chp, "motors1: %f\r\n", chassis->_motors[1].speed_sp);
  chprintf(chp, "motors2: %f\r\n", chassis->_motors[2].speed_sp);
  chprintf(chp, "motors3: %f\r\n", chassis->_motors[3].speed_sp);

  chprintf(chp, "motors0: %f\r\n", chassis->_motors[0]._speed);
  chprintf(chp, "motors1: %f\r\n", chassis->_motors[1]._speed);
  chprintf(chp, "motors2: %f\r\n", chassis->_motors[2]._speed);
  chprintf(chp, "motors3: %f\r\n", chassis->_motors[3]._speed);
}

void cmd_test_RF(BaseSequentialStream * chp, int argc, char *argv[])
{
  enable_all_rangefinder();
  chprintf(chp, "Testing rangefinder...\r\n");
  chThdSleepMilliseconds(1000);
  int j;
  for (j = 0; j < RANGEFINDER_NUM; ++j)
  {
     chprintf(chp, "rangefinder %i:%f\r\n",j,rangeFinder_getDistance(j));
  }
  disable_all_rangefinder();
}

void cmd_drive(BaseSequentialStream * chp, int argc, char *argv[])
{
  switch(*(argv[0]))
  {
    case 's':
      chassis_autoCmd(CHASSIS_STRAFE, -1.0f); break;
    case 'd':
      chassis_autoCmd(CHASSIS_DRIVE, -1.0f); break;
    case 'h':
      chassis_autoCmd(CHASSIS_HEADING, -1.0f); break;
    case 'k':
      chassis_killAutoDriver(); break;
  }


}

void cmd_error(BaseSequentialStream * chp, int argc, char *argv[])
{
  uint32_t error = chassis_getError();
  if(error)
    chprintf(chp,"CHASSIS CONNECTION ERROR: %X\r\n", error);

  error = lift_getError();
  if(error)
    chprintf(chp,"LIFT CONNECTION ERROR: %X\r\n", error);

  chprintf(chp, "GRIPPER CONNECTION ERROR: %X\r\n",gripper_getError());

  chassisStruct* chassis = chassis_get();

  chprintf(chp,"Chassis State: %X\r\n",  chassis->state);
  chprintf(chp,"island State: %X\r\n",   island_getRobotState());

  system_clearWarningFlag();
}

/**
 * @brief Start the data tramsmission to matlab
 * @note caution of data flooding to the serial port
 */
void cmd_data(BaseSequentialStream * chp, int argc, char *argv[])
{
  uint8_t sec = 10;

  if(argc && matlab_thread_handler == NULL)
  {
    char *toNumber = argv[0];
    uint32_t finalNum=0;
    while(*toNumber>='0' && *toNumber<='9')
      finalNum=finalNum*10+*(toNumber++)-'0';

    if(finalNum == 0)
      finalNum = 10;

    sec = (finalNum < 60 ? finalNum : 60);

    chprintf(chp,"Data transmission start in %d seconds...\r\n", sec);
    chThdSleepSeconds(sec);

    matlab_thread_handler = chThdCreateStatic(matlab_thread_wa, sizeof(matlab_thread_wa),
        NORMALPRIO - 3,
        matlab_thread, NULL);
  }
  else if(matlab_thread_handler != NULL)
  {
    chThdTerminate(matlab_thread_handler);
    matlab_thread_handler = NULL;
  }
}

void cmd_calibrate(BaseSequentialStream * chp, int argc, char *argv[])
{
  PIMUStruct pIMU = imu_get();
  PGyroStruct pGyro = gyro_get();
  if(argc)
  {
    if(!strcmp(argv[0], "accl"))
    {
      pIMU->accelerometer_not_calibrated = true;
      chThdSleepMilliseconds(10);
      calibrate_accelerometer(pIMU);
      chThdResume(&(pIMU->imu_Thd), MSG_OK);
    }
    else if(!strcmp(argv[0], "gyro"))
    {
      pIMU->gyroscope_not_calibrated = true;
      chThdSleepMilliseconds(10);
      calibrate_gyroscope(pIMU);
      chThdResume(&(pIMU->imu_Thd), MSG_OK);
    }
    else if(!strcmp(argv[0], "adi"))
    {
      pGyro->adis_gyroscope_not_calibrated = true;
      chThdSleepMilliseconds(10);
      if(argc && !strcmp(argv[1],"fast"))
        gyro_cal(pGyro,false); //fast calibration ~30s
      else if(argc && strcmp(argv[1],"full"))
        chprintf(chp,"Invalid parameter!\r\n");
      else
        gyro_cal(pGyro,true); //full calibration ~5min
      chThdResume(&(pGyro->adis_Thd), MSG_OK);
    }
    param_save_flash();
  }
  else
    chprintf(chp,"Calibration: gyro, accl, adi fast, adi full\r\n");
}

void cmd_temp(BaseSequentialStream * chp, int argc, char *argv[])
{
  (void) argc,argv;
//  uint32_t tick = chVTGetSystemTimeX();
//  tick += US2ST(5U);

//  while(1){ // you can uncomment this so that it continuously send the data out.
              // this is useful in tuning the Temperature PID
      PIMUStruct _pimu = imu_get();
//      pTPIDStruct _tempPID = TPID_get();
      chprintf(chp,"%f\n", _pimu->temperature);
//      chprintf(chp,"Temperature: %f\f\n", _pimu->temperature);
//      chprintf(chp,"PID_value: %i\i\n", _tempPID->PID_Value);
//      chThdSleep(MS2ST(500));
//  }
}

void cmd_gyro(BaseSequentialStream * chp, int argc, char *argv[])
{
      (void) argc,argv;

      PGyroStruct _pGyro = gyro_get();
      chprintf(chp,"Offset: %f\n", _pGyro->offset);
      chprintf(chp,"Angle_vel: %f\n", _pGyro->angle_vel);
      chprintf(chp,"Angle: %f\n", _pGyro->angle);
}

void cmd_lift_check(BaseSequentialStream * chp, int argc, char *argv[]){
  (void) argc,argv;
  motorPosStruct* lifts = lift_get();
  chprintf(chp,"lift1 :%f\r\n", lifts[0].pos_sp);
  chprintf(chp,"lift2 :%f\r\n", lifts[1].pos_sp);
  chprintf(chp,"lift3 :%f\r\n", lifts[2].pos_sp);
  chprintf(chp,"lift4 :%f\r\n", lifts[3].pos_sp);
}

/**
 * @brief array of shell commands, put the corresponding command and functions below
 * {"command", callback_function}
 */
static const ShellCommand commands[] =
{
  {"test", cmd_test},
  {"rf", cmd_test_RF},
  {"drive", cmd_drive},
  {"cal", cmd_calibrate},
  {"temp", cmd_temp},
  {"gyro", cmd_gyro},
  {"WTF", cmd_error},
  {"lift_check",cmd_lift_check},
  {"\xEE", cmd_data},
  #ifdef PARAMS_USE_USB
    {"\xFD",cmd_param_scale},
    {"\xFB",cmd_param_update},
    {"\xFA",cmd_param_tx},
    {"\xF9",cmd_param_rx},
  #endif
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 =
{
  (BaseSequentialStream *)SERIAL_CMD,
  commands
};

/**
 * @brief start the shell service
 * @require enable the corresponding serial ports in mcuconf.h and board.h
 *          Select the SERIAL_CMD port in main.h
 *
 * @api
 */
static const SerialConfig SERIAL_CMD_CONFIG = {
  115200,               //Baud Rate
  USART_CR1_UE,         //CR1 Register
  USART_CR2_LINEN,      //CR2 Register
  0                     //CR3 Register
};

void shellStart(void)
{
  //sdStart(SERIAL_CMD, &SERIAL_CMD_CONFIG);
  /*
   * Initializes a serial-over-USB CDC driver.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);


  //  * Activates the USB driver and then the USB bus pull-up on D+.
  //  * Note, a delay is inserted in order to not have to disconnect the cable
  //  * after a reset.



  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1500);

  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

  shellInit();

  shellCreateStatic(&shell_cfg1, Shell_thread_wa,
      sizeof(Shell_thread_wa), NORMALPRIO);

}
