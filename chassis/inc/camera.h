#ifndef _CAMERA_H_
#define _CAMERA_H_

#define CAMERA_CAN      &CAND1
#define CAMERA_CAN_EID  0x1FF

#define CAMERA_CONTROL_FREQ 1000U
#define CAMERA_CUTOFF_FREQ    30U

typedef struct{
  uint8_t _wait_count;
  float _angle;
  float _current;

  float _speed_enc;
  int8_t _dir;

  float _speed_cmd;
  float _speed;
} GM3510Struct;

#endif
