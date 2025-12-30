#include "imu_bmi270.h"
#include "imu_icm42688.h"
#include "imu_bus.h"
#include "spi_bus.h"

void spi_bus_idle_hook(void)
{
    if (!imu_bus_is_ready())
    {
        return;
    }

    /* After each DMA completes, try to kick both sensors.
     * Each kick() checks its own s_irq_pending flag, so only
     * sensors with pending data will attempt DMA. The first
     * one to successfully start DMA wins; the other will
     * retry on next hook or via poll(). */
    imu_bmi270_kick();
    imu_icm42688_kick();
}
