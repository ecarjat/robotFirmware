#include "imu_bmi270.h"
#include "imu_icm42688.h"

#include <stddef.h>

static bool s_bmi_have = false;
static imu_bmi270_sample_t s_bmi_sample = {};
static uint32_t s_bmi_irq_ms = 0;

static bool s_icm_have = false;
static imu_icm42688_sample_t s_icm_sample = {};
static uint32_t s_icm_irq_ms = 0;

bool imu_bmi270_init(void) { return true; }
void imu_bmi270_poll(void) {}
void imu_bmi270_handle_int1(void) {}
bool imu_bmi270_kick(void) { return true; }
uint32_t imu_bmi270_get_last_irq_ms(void) { return s_bmi_irq_ms; }

bool imu_bmi270_try_get_latest(imu_bmi270_sample_t *out, uint32_t *seq)
{
    (void)seq;
    if (!s_bmi_have || out == NULL) {
        return false;
    }
    *out = s_bmi_sample;
    return true;
}

bool imu_icm42688_init(void) { return true; }
void imu_icm42688_poll(void) {}
void imu_icm42688_handle_int1(void) {}
bool imu_icm42688_kick(void) { return true; }
uint32_t imu_icm42688_get_last_irq_ms(void) { return s_icm_irq_ms; }

bool imu_icm42688_try_get_latest(imu_icm42688_sample_t *out, uint32_t *seq)
{
    (void)seq;
    if (!s_icm_have || out == NULL) {
        return false;
    }
    *out = s_icm_sample;
    return true;
}

/* Test helpers (optional) */
void imu_fake_set_bmi_sample(const imu_bmi270_sample_t *sample, uint32_t irq_ms)
{
    if (sample) {
        s_bmi_sample = *sample;
        s_bmi_have = true;
    } else {
        s_bmi_have = false;
    }
    s_bmi_irq_ms = irq_ms;
}

void imu_fake_set_icm_sample(const imu_icm42688_sample_t *sample, uint32_t irq_ms)
{
    if (sample) {
        s_icm_sample = *sample;
        s_icm_have = true;
    } else {
        s_icm_have = false;
    }
    s_icm_irq_ms = irq_ms;
}
