#include "stm32h7xx_hal.h"

static bool app_in_isr(void) { return (__get_IPSR() != 0U); }