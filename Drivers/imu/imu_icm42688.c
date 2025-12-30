#include "imu_icm42688.h"

#include <string.h>

#include "app_config.h"
#include "imu_bmi270.h"
#include "imu_icm42688_config.h"
#include "imu_bus.h"
#include "main.h"
#include "spi_bus.h"

#define ICM42688_SPI_READ_MASK      0x80U
#define ICM42688_SPI_MAX_REG_XFER   32U
#define ICM42688_REG_TIMEOUT_MS     2U

#define ICM42688_REG_BANK_SEL       0x76U
#define ICM42688_REG_DEVICE_CONFIG  0x11U
#define ICM42688_REG_INT_CONFIG     0x14U
#define ICM42688_REG_TEMP_DATA1     0x1DU
#define ICM42688_REG_WHO_AM_I       0x75U
#define ICM42688_REG_INT_CONFIG1    0x64U
#define ICM42688_REG_INT_SOURCE0    0x65U
#define ICM42688_REG_PWR_MGMT0      0x4EU
#define ICM42688_REG_GYRO_CONFIG0   0x4FU
#define ICM42688_REG_ACCEL_CONFIG0  0x50U

#define ICM42688_WHO_AM_I_VALUE     0x47U

#define ICM42688_DATA_LEN           14U
#define ICM42688_DATA_FRAME_LEN     (ICM42688_DATA_LEN + 1U)

extern SPI_HandleTypeDef hspi6;

static spi_bus_device_t s_icm_spi;
static volatile uint8_t s_init_ok = 0U;
static volatile uint8_t s_dma_inflight = 0U;
static volatile uint8_t s_irq_pending = 0U;
static volatile uint32_t s_sample_seq = 0U;
static imu_icm42688_sample_t s_latest_sample;
static uint8_t s_bank = 0U;
static uint32_t s_last_poll_ms = 0U;
static uint32_t s_last_sample_ms = 0U;
static uint32_t s_blocking_fail = 0U;
static uint32_t s_blocking_ok = 0U;
static uint32_t s_dma_start_ms = 0U;
static uint32_t s_dma_fail = 0U;
static volatile uint32_t s_irq_seen = 0U;
static uint8_t s_irq_logged = 0U;
static uint8_t s_irq_missing_logged = 0U;
static uint32_t s_init_ms = 0U;

static uint8_t s_data_tx[ICM42688_DATA_FRAME_LEN] __attribute__((section(".bdma_buffer"), aligned(32)));
static uint8_t s_data_rx[ICM42688_DATA_FRAME_LEN] __attribute__((section(".bdma_buffer"), aligned(32)));

static int icm_spi_read(uint8_t reg, uint8_t *buf, uint32_t len);
static int icm_spi_write(uint8_t reg, const uint8_t *buf, uint32_t len);
static int icm_set_bank(uint8_t bank);
static void icm_store_sample(const imu_icm42688_sample_t *sample);
static void icm_parse_sample(const uint8_t *data, imu_icm42688_sample_t *sample);
static void icm_dma_done(void *ctx, int status);
static void icm_start_dma_read(void);
static int icm_soft_reset(void);
static int icm_read_blocking(imu_icm42688_sample_t *sample);

static int icm_spi_read(uint8_t reg, uint8_t *buf, uint32_t len)
{
    if (buf == NULL || len == 0U || len > ICM42688_SPI_MAX_REG_XFER)
    {
        return -1;
    }

    uint8_t tx[ICM42688_SPI_MAX_REG_XFER + 1U];
    uint8_t rx[ICM42688_SPI_MAX_REG_XFER + 1U];

    tx[0] = (uint8_t)(reg | ICM42688_SPI_READ_MASK);
    memset(&tx[1], 0x00, len);

    if (spi_bus_transfer_blocking(&s_icm_spi, tx, rx, len + 1U, ICM42688_REG_TIMEOUT_MS) != 0)
    {
        return -1;
    }

    memcpy(buf, &rx[1], len);
    return 0;
}

static int icm_spi_write(uint8_t reg, const uint8_t *buf, uint32_t len)
{
    if (buf == NULL || len == 0U || len > ICM42688_SPI_MAX_REG_XFER)
    {
        return -1;
    }

    uint8_t tx[ICM42688_SPI_MAX_REG_XFER + 1U];
    uint8_t rx[ICM42688_SPI_MAX_REG_XFER + 1U];

    tx[0] = (uint8_t)(reg & ~ICM42688_SPI_READ_MASK);
    memcpy(&tx[1], buf, len);

    if (spi_bus_transfer_blocking(&s_icm_spi, tx, rx, len + 1U, ICM42688_REG_TIMEOUT_MS) != 0)
    {
        return -1;
    }

    return 0;
}

static int icm_set_bank(uint8_t bank)
{
    if (s_bank == bank)
    {
        return 0;
    }
    s_bank = bank;
    return icm_spi_write(ICM42688_REG_BANK_SEL, &bank, 1U);
}

static int icm_soft_reset(void)
{
    uint8_t val = 0x01U;
    if (icm_set_bank(0U) != 0)
    {
        return -1;
    }
    if (icm_spi_write(ICM42688_REG_DEVICE_CONFIG, &val, 1U) != 0)
    {
        return -1;
    }
    HAL_Delay(1);
    return 0;
}

static void icm_store_sample(const imu_icm42688_sample_t *sample)
{
    if (sample == NULL)
    {
        return;
    }

    uint32_t seq = s_sample_seq;
    s_sample_seq = seq + 1U;
    __DMB();
    s_latest_sample = *sample;
    __DMB();
    s_sample_seq = seq + 2U;
}

static void icm_parse_sample(const uint8_t *data, imu_icm42688_sample_t *sample)
{
    if (data == NULL || sample == NULL)
    {
        return;
    }

    memset(sample, 0, sizeof(*sample));
    sample->timestamp_ms = HAL_GetTick();

    sample->temperature = (int16_t)((data[0] << 8) | data[1]);
    sample->accel[0] = (int16_t)((data[2] << 8) | data[3]);
    sample->accel[1] = (int16_t)((data[4] << 8) | data[5]);
    sample->accel[2] = (int16_t)((data[6] << 8) | data[7]);
    sample->gyro[0] = (int16_t)((data[8] << 8) | data[9]);
    sample->gyro[1] = (int16_t)((data[10] << 8) | data[11]);
    sample->gyro[2] = (int16_t)((data[12] << 8) | data[13]);
}

static volatile uint32_t s_dma_done_cnt = 0U;
static volatile uint32_t s_dma_done_err_cnt = 0U;
static volatile uint32_t s_kick_cnt = 0U;
static volatile uint32_t s_kick_blocked_cnt = 0U;

static void icm_dma_done(void *ctx, int status)
{
    (void)ctx;
    s_dma_inflight = 0U;
    if (status == 0)
    {
        s_dma_done_cnt++;
        imu_icm42688_sample_t sample;
        icm_parse_sample(&s_data_rx[1], &sample);
        icm_store_sample(&sample);
        s_last_sample_ms = HAL_GetTick();
    }
    else
    {
        s_dma_done_err_cnt++;
        s_irq_pending = 1U;
    }
}

static void icm_start_dma_read(void)
{
    if (!s_init_ok || s_dma_inflight)
    {
        return;
    }

    int rc = spi_bus_transfer_dma(&s_icm_spi, s_data_tx, s_data_rx, sizeof(s_data_tx), icm_dma_done, NULL);
    if (rc == SPI_BUS_OK)
    {
        s_dma_inflight = 1U;
        s_dma_start_ms = HAL_GetTick();
    }
    else if (rc == SPI_BUS_BUSY)
    {
        s_irq_pending = 1U;
    }
    else
    {
        s_irq_pending = 1U;
        if (s_dma_fail < 5U)
        {
            s_dma_fail++;
            APP_LOG_ERROR("ICM42688 DMA start failed rc=%d busy=%d state=0x%lx err=0x%lx", rc,
                          spi_bus_is_busy(), (unsigned long)hspi6.State,
                          (unsigned long)HAL_SPI_GetError(&hspi6));
        }
    }
}

static int icm_read_blocking(imu_icm42688_sample_t *sample)
{
    if (sample == NULL)
    {
        return -1;
    }

    uint8_t tx[ICM42688_DATA_FRAME_LEN];
    uint8_t rx[ICM42688_DATA_FRAME_LEN];
    memset(tx, 0, sizeof(tx));
    tx[0] = (uint8_t)(ICM42688_REG_TEMP_DATA1 | ICM42688_SPI_READ_MASK);

    int rc = spi_bus_transfer_blocking(&s_icm_spi, tx, rx, sizeof(tx), ICM42688_REG_TIMEOUT_MS);
    if (rc != 0)
    {
        return rc;
    }

    icm_parse_sample(&rx[1], sample);
    return 0;
}

bool imu_icm42688_init(void)
{
    memset(&s_icm_spi, 0, sizeof(s_icm_spi));

    (void)spi_bus_init(&hspi6);
    spi_bus_device_init(&s_icm_spi,
                        ICM42688_CS_GPIO_Port,
                        ICM42688_CS_Pin,
                        SPI_POLARITY_HIGH,
                        SPI_PHASE_2EDGE,
                        hspi6.Init.BaudRatePrescaler,
                        hspi6.Init.DataSize);

    if (icm_soft_reset() != 0)
    {
        APP_LOG_ERROR("ICM42688 reset failed");
        return false;
    }

    uint8_t whoami = 0U;
    if (icm_spi_read(ICM42688_REG_WHO_AM_I, &whoami, 1U) != 0)
    {
        APP_LOG_ERROR("ICM42688 WHO_AM_I read failed");
        return false;
    }
    APP_LOG_INFO("ICM42688 WHO_AM_I=0x%02x", whoami);
    if (whoami != ICM42688_WHO_AM_I_VALUE)
    {
        APP_LOG_ERROR("ICM42688 WHO_AM_I mismatch (expected 0x%02x)", ICM42688_WHO_AM_I_VALUE);
        return false;
    }

    if (icm_set_bank(0U) != 0)
    {
        APP_LOG_ERROR("ICM42688 bank select failed");
        return false;
    }

    uint8_t pwr = ICM42688_CFG_PWR_MGMT0;
    if (icm_spi_write(ICM42688_REG_PWR_MGMT0, &pwr, 1U) != 0)
    {
        APP_LOG_ERROR("ICM42688 PWR_MGMT0 failed");
        return false;
    }

    uint8_t accel_cfg = (uint8_t)((ICM42688_CFG_ACC_FS << 5) | ICM42688_CFG_ACC_ODR);
    if (icm_spi_write(ICM42688_REG_ACCEL_CONFIG0, &accel_cfg, 1U) != 0)
    {
        APP_LOG_ERROR("ICM42688 accel config failed");
        return false;
    }

    uint8_t gyro_cfg = (uint8_t)((ICM42688_CFG_GYR_FS << 5) | ICM42688_CFG_GYR_ODR);
    if (icm_spi_write(ICM42688_REG_GYRO_CONFIG0, &gyro_cfg, 1U) != 0)
    {
        APP_LOG_ERROR("ICM42688 gyro config failed");
        return false;
    }

    uint8_t int_cfg = ICM42688_CFG_INT_CONFIG;
    if (icm_spi_write(ICM42688_REG_INT_CONFIG, &int_cfg, 1U) != 0)
    {
        APP_LOG_ERROR("ICM42688 INT_CONFIG failed");
        return false;
    }

    uint8_t int_cfg1 = 0U;
    if (icm_spi_read(ICM42688_REG_INT_CONFIG1, &int_cfg1, 1U) != 0)
    {
        APP_LOG_ERROR("ICM42688 INT_CONFIG1 read failed");
        return false;
    }
    int_cfg1 &= (uint8_t)~ICM42688_CFG_INT_CONFIG1_CLR_MASK;
    if (icm_spi_write(ICM42688_REG_INT_CONFIG1, &int_cfg1, 1U) != 0)
    {
        APP_LOG_ERROR("ICM42688 INT_CONFIG1 write failed");
        return false;
    }

    uint8_t int_src0 = ICM42688_CFG_INT_SOURCE0;
    if (icm_spi_write(ICM42688_REG_INT_SOURCE0, &int_src0, 1U) != 0)
    {
        APP_LOG_ERROR("ICM42688 INT_SOURCE0 failed");
        return false;
    }

    memset(s_data_tx, 0, sizeof(s_data_tx));
    s_data_tx[0] = (uint8_t)(ICM42688_REG_TEMP_DATA1 | ICM42688_SPI_READ_MASK);

    s_init_ok = 1U;
    s_dma_inflight = 0U;
    s_irq_pending = 1U;
    s_sample_seq = 0U;
    memset(&s_latest_sample, 0, sizeof(s_latest_sample));
    s_irq_seen = 0U;
    s_irq_logged = 0U;
    s_irq_missing_logged = 0U;
    s_init_ms = HAL_GetTick();

    APP_LOG_INFO("ICM42688 initialized (SPI6 + DMA)");
    return true;
}

void imu_icm42688_handle_int1(void)
{
    if (!s_init_ok)
    {
        return;
    }
    s_irq_seen++;
    s_irq_pending = 1U;
    if (imu_bus_is_ready())
    {
        imu_icm42688_kick();
    }
}

void imu_icm42688_kick(void)
{
    if (!s_init_ok || s_dma_inflight || !s_irq_pending || !imu_bus_is_ready())
    {
        s_kick_blocked_cnt++;
        return;
    }
    s_kick_cnt++;
    s_irq_pending = 0U;
    icm_start_dma_read();
}

static uint32_t s_diag_last_ms = 0U;
static uint32_t s_poll_cnt = 0U;

void imu_icm42688_poll(void)
{
    s_poll_cnt++;

    if (!s_init_ok)
    {
        APP_LOG_ERROR("ICM42688 no initialized");
        return;
    }

    if (imu_bus_is_ready() && s_irq_seen != 0U && s_irq_logged == 0U)
    {
        s_irq_logged = 1U;
        APP_LOG_INFO("ICM42688 INT1 seen");
    }
    else if (imu_bus_is_ready() && s_irq_seen == 0U && s_irq_missing_logged == 0U &&
             (HAL_GetTick() - s_init_ms) > 1000U)
    {
        s_irq_missing_logged = 1U;
        APP_LOG_ERROR("ICM42688 INT1 not seen");
    }

    /* Diagnostic: log state every 500ms */
    uint32_t now_diag = HAL_GetTick();
    if ((now_diag - s_diag_last_ms) >= 500U)
    {
        s_diag_last_ms = now_diag;
        APP_LOG_INFO("ICM diag: poll=%lu irq=%lu pend=%u infl=%u kick=%lu blk=%lu done=%lu err=%lu",
                     (unsigned long)s_poll_cnt,
                     (unsigned long)s_irq_seen, s_irq_pending, s_dma_inflight,
                     (unsigned long)s_kick_cnt, (unsigned long)s_kick_blocked_cnt,
                     (unsigned long)s_dma_done_cnt, (unsigned long)s_dma_done_err_cnt);
    }

    imu_icm42688_kick();

#if (ICM42688_CFG_FALLBACK_POLL_MS > 0U) || (ICM42688_CFG_DMA_TIMEOUT_MS > 0U) || \
    (ICM42688_CFG_FALLBACK_BLOCKING_MS > 0U)
    uint32_t now = HAL_GetTick();
#endif

#if ICM42688_CFG_FALLBACK_POLL_MS > 0U
    if ((now - s_last_poll_ms) >= ICM42688_CFG_FALLBACK_POLL_MS)
    {
        s_last_poll_ms = now;
        icm_start_dma_read();
    }
#endif

#if ICM42688_CFG_DMA_TIMEOUT_MS > 0U
    if (s_dma_inflight && ((now - s_dma_start_ms) >= ICM42688_CFG_DMA_TIMEOUT_MS))
    {
        s_dma_inflight = 0U;
        spi_bus_abort();
        s_irq_pending = 1U;
        APP_LOG_ERROR("ICM42688 DMA timeout, aborting");
    }
#endif

#if ICM42688_CFG_FALLBACK_BLOCKING_MS > 0U
    if (!s_dma_inflight && ((now - s_last_sample_ms) >= ICM42688_CFG_FALLBACK_BLOCKING_MS))
    {
        imu_icm42688_sample_t sample;
        int rc = icm_read_blocking(&sample);
        if (rc == 0)
        {
            icm_store_sample(&sample);
            s_last_sample_ms = now;
            if (s_blocking_ok < 3U)
            {
                s_blocking_ok++;
                APP_LOG_INFO("ICM42688 blocking read ok");
            }
            s_blocking_fail = 0U;
        }
        else
        {
            if (s_blocking_fail < 5U)
            {
                s_blocking_fail++;
                APP_LOG_ERROR("ICM42688 blocking read failed rc=%d busy=%d", rc,
                              spi_bus_is_busy());
            }
        }
    }
#endif
}

bool imu_icm42688_try_get_latest(imu_icm42688_sample_t *out, uint32_t *seq)
{
    if (!s_init_ok || out == NULL || seq == NULL)
    {
        return false;
    }

    uint32_t seq1 = s_sample_seq;
    if (seq1 & 1U)
    {
        return false;
    }

    __DMB();
    imu_icm42688_sample_t sample = s_latest_sample;
    __DMB();

    uint32_t seq2 = s_sample_seq;
    if (seq1 != seq2 || (seq2 & 1U))
    {
        return false;
    }

    if (seq2 == *seq)
    {
        return false;
    }

    *seq = seq2;
    *out = sample;
    return true;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ICM42688_INT1_Pin)
    {
        imu_icm42688_handle_int1();
    }
    else if (GPIO_Pin == BMI270_INT1_Pin)
    {
        imu_bmi270_handle_int1();
    }
}
