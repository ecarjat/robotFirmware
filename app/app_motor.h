#ifndef APP_MOTOR_H
#define APP_MOTOR_H

#include "param_storage.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint8_t enabled;
  float left;
  float right;
} app_motor_manual_t;

extern app_motor_manual_t s_motor_manual;

void app_motor_manual_apply(void);
uint8_t app_motor_manual_run(uint8_t side, float intensity);
uint8_t app_motor_manual_enable(bool enable);

#ifdef __cplusplus
}
#endif

#endif /* APP_MOTOR_H */