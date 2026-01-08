#include "app_profiling.h"

#if APP_ENABLE_PROFILING

#include "app_config.h"
#include "control_timer.h"
#include <string.h>

typedef struct {
  uint32_t max_us;
  uint64_t sum_us;
  uint32_t count;
} app_profile_stats_t;

static app_profile_stats_t s_stats[APP_PROF_COUNT];
static uint32_t s_freq_mhz = 0;

void app_profiling_init(void) {
  memset(s_stats, 0, sizeof(s_stats));
  s_freq_mhz = HAL_RCC_GetHCLKFreq() / 1000000;
  if (s_freq_mhz == 0) {
    s_freq_mhz = 480;
  }
}

void app_profiling_record(app_prof_id_t id, uint32_t start_cyc) {
  if (id >= APP_PROF_COUNT) {
    return;
  }

  uint32_t end = DWT->CYCCNT;
  uint32_t cycles = end - start_cyc;

  if (s_freq_mhz == 0) {
    s_freq_mhz = HAL_RCC_GetHCLKFreq() / 1000000;
    if (s_freq_mhz == 0) {
      s_freq_mhz = 480;
    }
  }

  uint32_t us = cycles / s_freq_mhz;
  app_profile_stats_t *p = &s_stats[id];

  if (us > p->max_us) {
    p->max_us = us;
  }
  p->sum_us += us;
  p->count++;
}

void app_profiling_log(void) {
  control_timing_diag_t c_diag;
  control_timer_get_diag(&c_diag);

  uint32_t ctrl_avg = c_diag.loop_count ? (uint32_t)(c_diag.sum_execution_us / c_diag.loop_count) : 0;

  uint32_t avg[APP_PROF_COUNT];
  for (int i = 0; i < APP_PROF_COUNT; i++) {
    avg[i] = s_stats[i].count ? (uint32_t)(s_stats[i].sum_us / s_stats[i].count) : 0;
  }

  APP_LOG_INFO("PROF: Ctrl max=%lu avg=%lu, Link max=%lu avg=%lu, Mot max=%lu avg=%lu, IMU max=%lu avg=%lu, Idle max=%lu avg=%lu",
               (unsigned long)c_diag.max_execution_us, (unsigned long)ctrl_avg,
               (unsigned long)s_stats[APP_PROF_LINK].max_us, (unsigned long)avg[APP_PROF_LINK],
               (unsigned long)s_stats[APP_PROF_MOTOR].max_us, (unsigned long)avg[APP_PROF_MOTOR],
               (unsigned long)s_stats[APP_PROF_IMU].max_us, (unsigned long)avg[APP_PROF_IMU],
               (unsigned long)s_stats[APP_PROF_IDLE].max_us, (unsigned long)avg[APP_PROF_IDLE]);

  memset(s_stats, 0, sizeof(s_stats));
  control_timer_reset_diag();
}

#endif /* APP_ENABLE_PROFILING */