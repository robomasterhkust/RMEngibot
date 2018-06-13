#ifndef _SYSTEM_ERROR_H_
#define _SYSTEM_ERROR_H_

#define SYSTEM_TEMP_WARNING_DURATION  6U

void system_setErrorFlag(void);
void system_setWarningFlag(void);
void system_setTempWarningFlag(void);
void system_clearWarningFlag(void);

void system_error_init(void);

typedef enum{
  SYSTEM_ERROR_DUMMY = 0,
  SYSTEM_ERROR = 1,
  SYSTEM_WARNING = 2,
  SYSTEM_TEMP_WARNING = 4
} system_error_t;

#endif
