#include "ch.h"
#include "hal.h"

#include "camera.h"
#include "dbus.h"

#define CAMERA_OUTPUT_MAX 10000U

static GM3510Struct motor;
static GimbalEncoder_canStruct* _encoder;
static param_t pos_sp[2];
static pid_controller_t controller;

static lpfilterStruct lp_angle;
static RC_Ctl_t* rc;

#define CAMERA_SPEED_BUFFER_LEN      50U
static float _speed_buffer[CAMERA_SPEED_BUFFER_LEN];
static uint8_t _speed_count_buffer[CAMERA_SPEED_BUFFER_LEN];
static uint8_t _count_sum;
static uint32_t _speed_count;


#define CAMERA_CONNECTION_ERROR_COUNT 10U
/**
 *  @brief                  process data from gimbal encoder
 *  @param[in,out]  motor   pointer to corresponding motor
 *  @param[in]         id   GIMBAL_YAW or GIMBAL_PITCH
 */
static void camera_encoderUpdate(void)
{
  if(_encoder->updated)
  {

    //Check validiaty of can connection
    _encoder->updated = false;

    float angle_input = _encoder->radian_angle;

    motor._angle = lpfilter_apply(&lp_angle, angle_input);

    //add FIR filter to encoder speed
    _count_sum += motor._wait_count;
    _count_sum -= _speed_count_buffer[_speed_count % CAMERA_SPEED_BUFFER_LEN];
    float diff = _encoder.radian_angle -
      _speed_buffer[_speed_count % CAMERA_SPEED_BUFFER_LEN];

    motor._speed_enc = diff * CAMERA_CONTROL_FREQ /_count_sum[id];
    _speed_buffer[_speed_count % CAMERA_SPEED_BUFFER_LEN] = _encoder.radian_angle;
    _speed_count_buffer[_speed_count % CAMERA_SPEED_BUFFER_LEN] = motor._wait_count;
    _speed_count++;

    motor._wait_count = 1;
  }
  else
  {
    motor._wait_count++;
    if(motor._wait_count > CAMERA_CONNECTION_ERROR_COUNT)
      motor._wait_count = 1;
  }
}

static inline float camera_controlPos(pid_controller_t* const controller,
  const float _error, const float _d_error)
{
  controller->error_int += _error * controller->ki;
  bound(&(controller->error_int), controller->error_int_max);

  float output = _error * controller->kp + controller->error_int - _d_error * controller->kd;
  bound(&output, CAMERA_OUTPUT_MAX);

  return output;
}

#define CAMERA_CONTROL_PERIOD_ST     US2ST(1000000U/CAMERA_CONTROL_FREQ)
static THD_WORKING_AREA(camera_thread_wa, 512);
static THD_FUNCTION(camera_thread,p)
{
  (void)p;
  chRegSetThreadName("Camera controller");

  uint32_t tick = chVTGetSystemTimeX();
  while(true)
  {
    tick += CAMERA_CONTROL_PERIOD_ST;
    if(chVTGetSystemTimeX() < tick)
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }

    camera_encoderUpdate();

    float error = pos_sp[0] - motor._pos;
    float output = camera_controlPos(&controller, error, motor.speed_enc);
    can_motorSetCurrent(&CAND1, 0x1FF, output, 0 ,0 ,0);
  }
}

const char controllerName[] = "CameraPID"
const char posName[] = "CameraPos"
const char subName[] = "0 1"

void camera_init(void)
{
  memset(&motor, 0 ,sizeof(GM3510Struct));
  _encoder = can_getGimbalMotor();
  rc = RC_get();

  lpfilter_init(&lp_angle, CAMERA_CONTROL_FREQ, CAMERA_CUTOFF_FREQ);
  params_set(&controller, 22, 3,  controllerName,  subname_PID, PARAM_PUBLIC);
  params_set(&controller, 23, 2,  posName,  subname, PARAM_PUBLIC);

  chThdCreateStatic(camera_thread_wa, sizeof(camera_thread_wa),
                    NORMALPRIO - 5, camera_thread, NULL);
}
