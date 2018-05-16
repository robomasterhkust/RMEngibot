#include "ch.h"
#include "hal.h"

#include "gripper.h"
#include "canBusProcess.h"
#include <math.h>

static ChassisEncoder_canStruct* gripper_encoders;
static volatile motorPosStruct motors[GRIPPER_MOTOR_NUM];
static float offset[GRIPPER_MOTOR_NUM];
static lpfilterStruct lp_speed[GRIPPER_MOTOR_NUM];
static bool in_position[GRIPPER_MOTOR_NUM] = {0 , 0};

static pid_controller_t controllers[GRIPPER_MOTOR_NUM];
static gripper_state_t gripper_state = GRIPPER_UNINIT;

static gripper_error_t gripper_error = 0;

motorPosStruct* gripper_get(void)
{
  return motors;
}

void gripper_kill(void)
{
  LEDR_ON();
  gripper_state = GRIPPER_ERROR;
  can_motorSetCurrent(GRIPPER_CAN, GRIPPER_CAN_EID, 0, 0, 0, 0);
}

gripper_state_t gripper_getError(void)
{
  uint32_t errorFlag = gripper_error;
  gripper_error = 0;
  return errorFlag;
}

#define  GRIPPER_ANGLE_PSC 7.6699e-4 //2*M_PI/0x1FFF

bool gripper_inPosition(const uint8_t id)
{
  if(id > GRIPPER_MOTOR_NUM)
    return false;

  return in_position[id];
}

static void gripper_encoderUpdate(void)
{
  uint8_t i;
  for (i = 0; i < 2U; i++)
  {
    if(gripper_encoders[i].updated)
    {
      //Check validiaty of can connection
      gripper_encoders[i].updated = false;

      float pos_input = gripper_encoders[i].raw_angle*GRIPPER_ANGLE_PSC;
      float speed_input = gripper_encoders[i].raw_speed/ gripper_gear_ratio[i];

      // if((motors[i]._prev < 0.6f && pos_input > 5.68f) ||
      //   (speed_input < -80.0f && pos_input > motors[i]._prev))
      //   motors[i].rev--;
      // if((motors[i]._prev > 5.68f && pos_input < 0.6f) ||
      //   (speed_input > 80.0f && pos_input < motors[i]._prev))
      //   motors[i].rev++;
      motors[i].rev = gripper_encoders[i].round_count;

      motors[i]._prev = pos_input;
      pos_input += motors[i].rev * 2*M_PI;

      motors[i]._pos =  pos_input / gripper_gear_ratio[i];
      motors[i]._speed = lpfilter_apply(&lp_speed[i], speed_input);
      motors[i]._wait_count = 1;
    }
    else
    {
      motors[i]._wait_count++;
      if(motors[i]._wait_count > GRIPPER_CONNECTION_ERROR_COUNT)
      {
        motors[i]._wait_count = 1;
        gripper_error |= GRIPPER_ARM_NOT_CONNECTED << i;
        gripper_kill();
      }
    }

    if((motors[i].pos_sp - motors[i]._pos) < 2 * M_PI &&
       (motors[i].pos_sp - motors[i]._pos) > -2 * M_PI)
      motors[i].in_position++;
    else
      motors[i].in_position = 0;

    if(motors[i].in_position > GRIPPER_IN_POSITION_COUNT)
    {
      motors[i].in_position = GRIPPER_IN_POSITION_COUNT;
      in_position[i] = true; //Motor is regarded as moved to postion
    }
  }
}


void gripper_changePos(const float pos_sp1, const float pos_sp2)
{
  if(offset[0] - pos_sp1 != motors[0].pos_sp)
  {
    in_position[0] = false;
    motors[0].pos_sp = offset[0] - pos_sp1;
  }
  if(offset[1] - pos_sp2 != motors[1].pos_sp)
  {
    in_position[1] = false;
    motors[1].pos_sp = offset[1] - pos_sp2;
  }
}


static int16_t gripper_controlPos
  (const motorPosStruct* const motor, pid_controller_t* const controller,
  const int16_t output_max)
{
  float error = motor->pos_sp - motor->_pos;
  controller->error_int += error * controller->ki;
  controller->error_int = boundOutput(controller->error_int, controller->error_int_max);
  float output =
    error*controller->kp + controller->error_int - motor->_speed * controller->kd;

  return (int16_t)(boundOutput(output,output_max));
}

#define GRIPPER_UPDATE_PERIOD_US  1000000/GRIPPER_CONTROL_FREQ
static THD_WORKING_AREA(gripper_control_wa, 512);
static THD_FUNCTION(gripper_control, p)
{
  (void)p;
  chRegSetThreadName("gripper controller");

  float output[2];
  float output_max[2];
  uint32_t tick = chVTGetSystemTimeX();
  while(true)
  {
    tick += US2ST(GRIPPER_UPDATE_PERIOD_US);
    if(tick > chVTGetSystemTimeX())
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }

    gripper_encoderUpdate();

    uint8_t i;
    for(i = 0; i < GRIPPER_MOTOR_NUM; i++)
    {
      if(gripper_state == GRIPPER_INITING)
      {
        output_max[0] = 7000;
        output_max[1] = 3500;
      }
      else
        output_max[i] = gripper_output_max[i];

      output[i] =
         gripper_controlPos(&motors[i], &controllers[i], output_max[i]);
    }

    can_motorSetCurrent(GRIPPER_CAN, GRIPPER_CAN_EID,
        output[0], output[1], 0, 0);
  }
}

#define STALL_COUNT_MAX 100U
int count = 0;
void gripper_calibrate(void)
{
  //To initialize the lift wheel, a calibration is needed
  //CAUTION!! Moving lift wheel may cause injury, stay back during power up!!

  bool init_state[GRIPPER_MOTOR_NUM] = {0, 0};

  float prev_pos[GRIPPER_MOTOR_NUM];

  uint8_t init_count = 0;
  uint8_t stall_count[GRIPPER_MOTOR_NUM] = {0, 0};

  const float motor_step[GRIPPER_MOTOR_NUM] = {0.002f, 0.02f};
  
  while(init_count < GRIPPER_MOTOR_NUM)
  {
    uint8_t i;
    init_count = 0;  //finish initialization only if all motor calibration finishes
    for (i = 0; i < GRIPPER_MOTOR_NUM; i++)
    {
      if(stall_count[i] < STALL_COUNT_MAX)
      {
        motors[i].pos_sp += motor_step[i];
        if(motors[i]._pos - prev_pos[i] < motor_step[i] * 0.2f)
          stall_count[i]++;
        else if(stall_count[i] > 10)
          stall_count[i] -= 10;
        else
          stall_count[i] = 0;

        prev_pos[i] = motors[i]._pos;
      }
      else
      {
        init_state[i] = true;
        offset[i] = motors[i]._pos;
      }

      init_count += init_state[i] ? 1 : 0;
      if(count == 0 && init_state[0] == true){
      ++count;
      motors[0].pos_sp = offset[0] - 0.1f;
      }
    }

    chThdSleepMilliseconds(2);
  }
  motors[1].pos_sp = offset[1] - 0.1f;
  gripper_state = GRIPPER_RUNNING;
}

static const ArmName = "Gripper Arm";
static const HandName = "Gripper Hand";

#define GRIPPER_ERROR_INT_MAX  30000
void gripper_init(void)
{
  memset(&motors, 0, sizeof(motorPosStruct) * GRIPPER_MOTOR_NUM);
  gripper_encoders = can_getExtraMotor() + 4;

  uint8_t i;
  for (i = 0; i < GRIPPER_MOTOR_NUM; i++) {
    lpfilter_init(lp_speed + i, GRIPPER_CONTROL_FREQ, 24);
    motors[i].speed_sp = 0.0f;
    motors[i]._speed = 0.0f;
    motors[i].pos_sp = 0.0f;
    motors[i]._pos = 0.0f;
    motors[i]._prev = 0.0f;

    controllers[i].error_int = 0.0f;
    controllers[i].error_int_max = GRIPPER_ERROR_INT_MAX;
  }
  params_set(&controllers[0], 20,3,ArmName, subname_PID,PARAM_PUBLIC);
  params_set(&controllers[1], 21,3,HandName,subname_PID,PARAM_PUBLIC);

  gripper_state = GRIPPER_INITING;

  chThdCreateStatic(gripper_control_wa, sizeof(gripper_control_wa),
                          NORMALPRIO, gripper_control, NULL);
}
