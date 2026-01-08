#ifndef APP_PROFILING_H
#define APP_PROFILING_H

#include "app_config.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  APP_PROF_LINK = 0,
  APP_PROF_MOTOR,
  APP_PROF_IMU,
  APP_PROF_IDLE,
  APP_PROF_COUNT
} app_prof_id_t;

#if APP_ENABLE_PROFILING

#define APP_PROFILE_GET_TIME() (DWT->CYCCNT)

void app_profiling_init(void);
void app_profiling_record(app_prof_id_t id, uint32_t start_cyc);
void app_profiling_log(void);

#else

#define APP_PROFILE_GET_TIME() 0U

static inline void app_profiling_init(void) {}
static inline void app_profiling_record(app_prof_id_t id, uint32_t start_cyc) { (void)id; (void)start_cyc; }
static inline void app_profiling_log(void) {}

#endif

#ifdef __cplusplus
}
#endif

#endif /* APP_PROFILING_H */