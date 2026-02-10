#include "app_motor.h"

app_motor_manual_t s_motor_manual = {0};

void app_motor_manual_apply(void)
{
}

uint8_t app_motor_manual_run(uint8_t side, float intensity)
{
    (void)side;
    s_motor_manual.left = intensity;
    s_motor_manual.right = intensity;
    return 0U;
}

uint8_t app_motor_manual_enable(bool enable)
{
    s_motor_manual.enabled = enable ? 1U : 0U;
    return 0U;
}
