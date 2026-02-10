#include "app_arm.h"

bool g_app_arm_can_arm = true;

bool app_try_arm_balancing(bool prepare_balance)
{
    (void)prepare_balance;
    return g_app_arm_can_arm;
}

void app_disarm_robot(void)
{
}
