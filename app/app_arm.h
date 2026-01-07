#ifndef APP_ARM_H
#define APP_ARM_H

#include "stdbool.h"

#ifdef __cplusplus
extern "C" {
#endif

bool app_try_arm_balancing(bool prepare_balance);
void app_disarm_robot(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_ARM_H */