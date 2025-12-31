#include "imu_sched.h"

#include <string.h>

#include "imu_bmi270.h"
#include "imu_bmi270_config.h"
#include "imu_bmm150.h"
#include "imu_bmm150_config.h"
#include "imu_bus.h"
#include "imu_icm42688.h"
#include "imu_icm42688_config.h"
#include "imu_sched_config.h"
#include "spi_bus.h"
#include "stm32h7xx_hal.h"

typedef bool (*imu_sched_kick_fn_t)(void);

static const imu_sched_kick_fn_t s_kick_fns[IMU_SCHED_SENSOR_COUNT] = {
    imu_bmi270_kick,
    imu_icm42688_kick,
    imu_bmm150_kick
};

static volatile uint32_t s_pending_mask = 0U;
static uint32_t s_pending_ts[IMU_SCHED_SENSOR_COUNT];
static uint32_t s_last_served_ms[IMU_SCHED_SENSOR_COUNT];
static uint32_t s_min_interval_ms[IMU_SCHED_SENSOR_COUNT];
static uint32_t s_timeout_ms[IMU_SCHED_SENSOR_COUNT];
static volatile int8_t s_active_sensor = -1;
static volatile uint32_t s_active_start_ms = 0U;
static uint8_t s_rr_cursor = 0U;
static volatile uint8_t s_running = 0U;

static uint32_t imu_sched_mask(imu_sched_sensor_t sensor)
{
    return (uint32_t)1U << (uint32_t)sensor;
}

static void imu_sched_set_pending(imu_sched_sensor_t sensor, uint32_t now_ms)
{
    uint32_t mask = imu_sched_mask(sensor);
    __disable_irq();
    s_pending_mask |= mask;
    s_pending_ts[sensor] = now_ms;
    __enable_irq();
}

static void imu_sched_clear_pending(imu_sched_sensor_t sensor)
{
    uint32_t mask = ~imu_sched_mask(sensor);
    __disable_irq();
    s_pending_mask &= mask;
    __enable_irq();
}

static int imu_sched_pick_rr(uint32_t mask)
{
    for (uint32_t i = 0U; i < IMU_SCHED_SENSOR_COUNT; ++i)
    {
        uint32_t idx = (uint32_t)(s_rr_cursor + i) % IMU_SCHED_SENSOR_COUNT;
        if (mask & ((uint32_t)1U << idx))
        {
            return (int)idx;
        }
    }
    return -1;
}

static int imu_sched_pick_oldest(uint32_t mask, uint32_t now_ms)
{
    int selected = -1;
    uint32_t best_age = 0U;
    for (uint32_t idx = 0U; idx < IMU_SCHED_SENSOR_COUNT; ++idx)
    {
        uint32_t bit = (uint32_t)1U << idx;
        if (!(mask & bit))
        {
            continue;
        }
        uint32_t age = now_ms - s_pending_ts[idx];
        if (selected < 0 || age > best_age)
        {
            selected = (int)idx;
            best_age = age;
        }
    }
    return selected;
}

static uint32_t imu_sched_eligible_mask(uint32_t pending, uint32_t now_ms)
{
    uint32_t eligible = 0U;
    for (uint32_t idx = 0U; idx < IMU_SCHED_SENSOR_COUNT; ++idx)
    {
        uint32_t bit = (uint32_t)1U << idx;
        if (!(pending & bit))
        {
            continue;
        }
        uint32_t min_interval = s_min_interval_ms[idx];
        if (min_interval == 0U || (now_ms - s_last_served_ms[idx]) >= min_interval)
        {
            eligible |= bit;
        }
    }
    return eligible;
}

void imu_sched_init(void)
{
    memset(s_pending_ts, 0, sizeof(s_pending_ts));
    memset(s_last_served_ms, 0, sizeof(s_last_served_ms));
    memset(s_min_interval_ms, 0, sizeof(s_min_interval_ms));
    memset(s_timeout_ms, 0, sizeof(s_timeout_ms));
    s_pending_mask = 0U;
    s_rr_cursor = 0U;
    s_running = 0U;
    s_active_sensor = -1;
    s_active_start_ms = 0U;
    s_min_interval_ms[IMU_SCHED_SENSOR_BMI270] = IMU_SCHED_BMI270_MIN_INTERVAL_MS;
    s_min_interval_ms[IMU_SCHED_SENSOR_ICM42688] = IMU_SCHED_ICM42688_MIN_INTERVAL_MS;
    s_min_interval_ms[IMU_SCHED_SENSOR_BMM150] = IMU_SCHED_BMM150_MIN_INTERVAL_MS;
    s_timeout_ms[IMU_SCHED_SENSOR_BMI270] = BMI270_CFG_DMA_TIMEOUT_MS;
    s_timeout_ms[IMU_SCHED_SENSOR_ICM42688] = ICM42688_CFG_DMA_TIMEOUT_MS;
    s_timeout_ms[IMU_SCHED_SENSOR_BMM150] = BMM150_CFG_DMA_TIMEOUT_MS;
}

void imu_sched_set_min_interval(imu_sched_sensor_t sensor, uint32_t min_interval_ms)
{
    if (sensor >= IMU_SCHED_SENSOR_COUNT)
    {
        return;
    }
    s_min_interval_ms[sensor] = min_interval_ms;
}

uint32_t imu_sched_get_min_interval(imu_sched_sensor_t sensor)
{
    if (sensor >= IMU_SCHED_SENSOR_COUNT)
    {
        return 0U;
    }
    return s_min_interval_ms[sensor];
}

void imu_sched_request(imu_sched_sensor_t sensor)
{
    if (sensor >= IMU_SCHED_SENSOR_COUNT)
    {
        return;
    }
    imu_sched_set_pending(sensor, HAL_GetTick());
}

void imu_sched_on_dma_done(imu_sched_sensor_t sensor, int status)
{
    if (sensor >= IMU_SCHED_SENSOR_COUNT)
    {
        return;
    }

    __disable_irq();
    s_last_served_ms[sensor] = HAL_GetTick();
    if (s_active_sensor == (int8_t)sensor)
    {
        s_active_sensor = -1;
    }
    __enable_irq();

    if (status != 0)
    {
        imu_sched_set_pending(sensor, HAL_GetTick());
    }
}

void imu_sched_run(void)
{
    if (!imu_bus_is_ready() || spi_bus_is_busy())
    {
        return;
    }
    if (__LDREXB(&s_running) || __STREXB(1U, &s_running))
    {
        return;
    }

    uint32_t pending = s_pending_mask;
    if (pending == 0U)
    {
        s_running = 0U;
        return;
    }

    uint32_t now_ms = HAL_GetTick();
    uint32_t eligible = imu_sched_eligible_mask(pending, now_ms);
    uint32_t candidate = (eligible != 0U) ? eligible : pending;
    uint32_t attempted = 0U;

    for (uint32_t attempt = 0U; attempt < IMU_SCHED_SENSOR_COUNT && candidate != 0U; ++attempt)
    {
        int selected = (eligible != 0U) ? imu_sched_pick_rr(candidate) : imu_sched_pick_oldest(candidate, now_ms);
        if (selected < 0)
        {
            break;
        }

        uint32_t bit = (uint32_t)1U << (uint32_t)selected;
        attempted |= bit;
        imu_sched_clear_pending((imu_sched_sensor_t)selected);

        bool started = s_kick_fns[selected]();
        if (started)
        {
            s_active_sensor = (int8_t)selected;
            s_active_start_ms = now_ms;
            s_rr_cursor = (uint8_t)((selected + 1U) % IMU_SCHED_SENSOR_COUNT);
            s_running = 0U;
            return;
        }

        imu_sched_set_pending((imu_sched_sensor_t)selected, now_ms);
        candidate &= ~bit;
        if (candidate == 0U && eligible != 0U)
        {
            eligible = 0U;
            candidate = pending & ~attempted;
        }
    }

    s_running = 0U;
}

void imu_sched_tick(void)
{
    if (!imu_bus_is_ready())
    {
        return;
    }

    if (BMM150_CFG_FALLBACK_POLL_MS > 0U)
    {
        uint32_t now_ms = HAL_GetTick();
        uint32_t mask = imu_sched_mask(IMU_SCHED_SENSOR_BMM150);
        __disable_irq();
        uint32_t pending = s_pending_mask;
        uint32_t last_served = s_last_served_ms[IMU_SCHED_SENSOR_BMM150];
        __enable_irq();
        if ((pending & mask) == 0U &&
            (now_ms - last_served) >= BMM150_CFG_FALLBACK_POLL_MS)
        {
            imu_sched_set_pending(IMU_SCHED_SENSOR_BMM150, now_ms);
        }
    }

    if (!spi_bus_is_busy())
    {
        imu_sched_run();
        return;
    }

    __disable_irq();
    int8_t active = s_active_sensor;
    uint32_t start_ms = s_active_start_ms;
    __enable_irq();

    if (active < 0 || active >= (int8_t)IMU_SCHED_SENSOR_COUNT)
    {
        return;
    }

    uint32_t timeout_ms = s_timeout_ms[active];
    if (timeout_ms == 0U)
    {
        return;
    }

    uint32_t now_ms = HAL_GetTick();
    if ((now_ms - start_ms) < timeout_ms)
    {
        return;
    }

    spi_bus_abort();
    imu_sched_run();
}
