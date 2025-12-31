#ifndef STM32_APP_MAIN_H
#define STM32_APP_MAIN_H

#include "param_storage.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Global robot parameters loaded from flash at startup
 *
 * This structure is initialized by app_main() and contains calibration
 * data, PID gains, and other persistent configuration. Modules should
 * access this for runtime configuration.
 */
extern robot_params_t g_robot_params;

void app_main(void);

#ifdef __cplusplus
}
#endif

#endif /* STM32_APP_MAIN_H */
