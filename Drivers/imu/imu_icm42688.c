#include "imu_icm42688.h"

#include <string.h>

#include "app_config.h"
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

#define ICM42688_ODR_200HZ          0x07U
#define ICM42688_GYRO_FS_2000DPS    0x00U
#define ICM42688_ACCEL_FS_16G       0x00U

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

static uint8_t s_data_tx[ICM42688_DATA_FRAME_LEN] __attribute__((section(".dma_buffer"), aligned(32)));
static uint8_t s_data_rx[ICM42688_DATA_FRAME_LEN] __attribute__((section(".dma_buffer"), aligned(32)));

static int icm_spi_read(uint8_t reg, uint8_t *buf, uint32_t len);
static int icm_spi_write(uint8_t reg, const uint8_t *buf, uint32_t len);
static int icm_set_bank(uint8_t bank);
static void icm_store_sample(const imu_icm42688_sample_t *sample);
static void icm_parse_sample(const uint8_t *data, imu_icm42688_sample_t *sample);
static void icm_dma_done(void *ctx, int status);
static void icm_start_dma_read(void);
static int icm_soft_reset(void);

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

static void icm_dma_done(void *ctx, int status)
{
    (void)ctx;
    s_dma_inflight = 0U;
    if (status == 0)
    {
        imu_icm42688_sample_t sample;
        icm_parse_sample(&s_data_rx[1], &sample);
        icm_store_sample(&sample);
    }
    else
    {
        s_irq_pending = 1U;
    }

    if (s_irq_pending && !s_dma_inflight)
    {
        s_irq_pending = 0U;
        icm_start_dma_read();
    }
}

static void icm_start_dma_read(void)
{
    if (!s_init_ok || s_dma_inflight)
    {
        return;
    }

    if (spi_bus_transfer_dma(&s_icm_spi, s_data_tx, s_data_rx, sizeof(s_data_tx), icm_dma_done, NULL) == 0)
    {
        s_dma_inflight = 1U;
    }
    else
    {
        s_irq_pending = 1U;
    }
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

    uint8_t pwr = 0x0FU;
    if (icm_spi_write(ICM42688_REG_PWR_MGMT0, &pwr, 1U) != 0)
    {
        APP_LOG_ERROR("ICM42688 PWR_MGMT0 failed");
        return false;
    }

    uint8_t accel_cfg = (uint8_t)((ICM42688_ACCEL_FS_16G << 5) | ICM42688_ODR_200HZ);
    if (icm_spi_write(ICM42688_REG_ACCEL_CONFIG0, &accel_cfg, 1U) != 0)
    {
        APP_LOG_ERROR("ICM42688 accel config failed");
        return false;
    }

    uint8_t gyro_cfg = (uint8_t)((ICM42688_GYRO_FS_2000DPS << 5) | ICM42688_ODR_200HZ);
    if (icm_spi_write(ICM42688_REG_GYRO_CONFIG0, &gyro_cfg, 1U) != 0)
    {
        APP_LOG_ERROR("ICM42688 gyro config failed");
        return false;
    }

    uint8_t int_cfg = 0x1BU;
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
    int_cfg1 &= (uint8_t)~0x10U;
    if (icm_spi_write(ICM42688_REG_INT_CONFIG1, &int_cfg1, 1U) != 0)
    {
        APP_LOG_ERROR("ICM42688 INT_CONFIG1 write failed");
        return false;
    }

    uint8_t int_src0 = 0x18U;
    if (icm_spi_write(ICM42688_REG_INT_SOURCE0, &int_src0, 1U) != 0)
    {
        APP_LOG_ERROR("ICM42688 INT_SOURCE0 failed");
        return false;
    }

    memset(s_data_tx, 0, sizeof(s_data_tx));
    s_data_tx[0] = (uint8_t)(ICM42688_REG_TEMP_DATA1 | ICM42688_SPI_READ_MASK);

    s_init_ok = 1U;
    s_dma_inflight = 0U;
    s_irq_pending = 0U;
    s_sample_seq = 0U;
    memset(&s_latest_sample, 0, sizeof(s_latest_sample));

    APP_LOG_INFO("ICM42688 initialized (SPI6 + DMA)");
    return true;
}

void imu_icm42688_handle_int1(void)
{
    if (!s_init_ok)
    {
        return;
    }
    if (s_dma_inflight)
    {
        s_irq_pending = 1U;
        return;
    }
    icm_start_dma_read();
}

void imu_icm42688_poll(void)
{
    if (!s_init_ok)
    {
        return;
    }

    if (s_irq_pending && !s_dma_inflight)
    {
        s_irq_pending = 0U;
        icm_start_dma_read();
    }
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
}
