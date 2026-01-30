#include "motor_backend.h"
#include "app_config.h"

extern const motor_backend_ops_t motor_backend_robust_uart_ops;
extern const motor_backend_ops_t motor_backend_steadywin_can_ops;

const motor_backend_ops_t *motor_backend_get(void) {
#if defined(MOTOR_BACKEND_STEADYWIN_CAN) && MOTOR_BACKEND_STEADYWIN_CAN
  return &motor_backend_steadywin_can_ops;
#else
  return &motor_backend_robust_uart_ops;
#endif
}
