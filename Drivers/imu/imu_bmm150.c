#include "imu_bmm150.h"

#include <string.h>

#include "app_config.h"
#include "bmm150.h"
#include "imu_bmm150_config.h"
#include "imu_bus.h"
#include "imu_sched.h"
#include "main.h"
#include "spi_bus.h"

#define BMM150_SPI_READ_MASK   0x80U
#define BMM150_SPI_MAX_XFER    32U
#define BMM150_REG_TIMEOUT_MS  2U

#define BMM150_DMA_DATA_START  BMM150_REG_DATA_X_LSB
#define BMM150_DMA_DATA_LEN    BMM150_LEN_XYZR_DATA
#define BMM150_DMA_FRAME_LEN   (1U + BMM150_DMA_DATA_LEN)
#define BMM150_DMA_RX_OFFSET   1U

extern SPI_HandleTypeDef hspi6;

static struct bmm150_dev s_bmm;
static spi_bus_device_t s_bmm_spi;
static volatile uint8_t s_init_ok = 0U;
static volatile uint8_t s_dma_inflight = 0U;
static volatile uint32_t s_sample_seq = 0U;
static imu_bmm150_sample_t s_latest_sample;
static uint8_t s_dma_fail = 0U;
static volatile uint32_t s_irq_seen = 0U;
static uint8_t s_irq_logged = 0U;
static uint8_t s_irq_missing_logged = 0U;
static uint32_t s_init_ms = 0U;

static uint8_t s_data_tx[BMM150_DMA_FRAME_LEN] __attribute__((section(".bdma_buffer"), aligned(32)));
static uint8_t s_data_rx[BMM150_DMA_FRAME_LEN] __attribute__((section(".bdma_buffer"), aligned(32)));

static BMM150_INTF_RET_TYPE bmm_spi_read(uint8_t reg_addr,
                                         uint8_t *reg_data,
                                         uint32_t len,
                                         void *intf_ptr);
static BMM150_INTF_RET_TYPE bmm_spi_write(uint8_t reg_addr,
                                          const uint8_t *reg_data,
                                          uint32_t len,
                                          void *intf_ptr);
static void bmm_delay_us(uint32_t period, void *intf_ptr);
static void bmm_store_sample(const imu_bmm150_sample_t *sample);
static bool bmm_parse_sample(const uint8_t *data, imu_bmm150_sample_t *sample);
static void bmm_dma_done(void *ctx, int status);
static bool bmm_start_dma_read(void);

static BMM150_INTF_RET_TYPE bmm_spi_read(uint8_t reg_addr,
                                         uint8_t *reg_data,
                                         uint32_t len,
                                         void *intf_ptr)
{
    if (reg_data == NULL || len == 0U || len > BMM150_SPI_MAX_XFER || intf_ptr == NULL)
    {
        return BMM150_E_NULL_PTR;
    }

    spi_bus_device_t *dev = (spi_bus_device_t *)intf_ptr;
    uint8_t tx[BMM150_SPI_MAX_XFER + 1U];
    uint8_t rx[BMM150_SPI_MAX_XFER + 1U];

    tx[0] = reg_addr;
    memset(&tx[1], 0x00, len);

    if (spi_bus_transfer_blocking(dev, tx, rx, len + 1U, BMM150_REG_TIMEOUT_MS) != 0)
    {
        return BMM150_E_COM_FAIL;
    }

    memcpy(reg_data, &rx[1], len);
    return BMM150_OK;
}

static BMM150_INTF_RET_TYPE bmm_spi_write(uint8_t reg_addr,
                                          const uint8_t *reg_data,
                                          uint32_t len,
                                          void *intf_ptr)
{
    if (reg_data == NULL || len == 0U || len > BMM150_SPI_MAX_XFER || intf_ptr == NULL)
    {
        return BMM150_E_NULL_PTR;
    }

    spi_bus_device_t *dev = (spi_bus_device_t *)intf_ptr;
    uint8_t tx[BMM150_SPI_MAX_XFER + 1U];
    uint8_t rx[BMM150_SPI_MAX_XFER + 1U];

    tx[0] = reg_addr;
    memcpy(&tx[1], reg_data, len);

    if (spi_bus_transfer_blocking(dev, tx, rx, len + 1U, BMM150_REG_TIMEOUT_MS) != 0)
    {
        return BMM150_E_COM_FAIL;
    }

    return BMM150_OK;
}

static void bmm_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    if (period == 0U)
    {
        return;
    }
    uint32_t ms = (period + 999U) / 1000U;
    HAL_Delay(ms);
}

static void bmm_store_sample(const imu_bmm150_sample_t *sample)
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

static bool bmm_parse_sample(const uint8_t *data, imu_bmm150_sample_t *sample)
{
    if (data == NULL || sample == NULL)
    {
        return false;
    }

    uint8_t raw[BMM150_LEN_XYZR_DATA];
    memcpy(raw, data, sizeof(raw));

    struct bmm150_mag_data mag = {0};
    if (bmm150_aux_mag_data(raw, &mag, &s_bmm) != BMM150_OK)
    {
        return false;
    }

    sample->timestamp_ms = HAL_GetTick();
    sample->mag[0] = (int16_t)mag.x;
    sample->mag[1] = (int16_t)mag.y;
    sample->mag[2] = (int16_t)mag.z;
    return true;
}

static void bmm_dma_done(void *ctx, int status)
{
    (void)ctx;
    s_dma_inflight = 0U;
    if (status == 0)
    {
        imu_bmm150_sample_t sample;
        if (bmm_parse_sample(&s_data_rx[BMM150_DMA_RX_OFFSET], &sample))
        {
            bmm_store_sample(&sample);
        }
    }
    imu_sched_on_dma_done(IMU_SCHED_SENSOR_BMM150, status);
}

static bool bmm_start_dma_read(void)
{
    if (!s_init_ok)
    {
        return false;
    }
    if (__LDREXB(&s_dma_inflight) || __STREXB(1U, &s_dma_inflight))
    {
        return false;
    }

    int rc = spi_bus_transfer_dma(&s_bmm_spi, s_data_tx, s_data_rx, sizeof(s_data_tx), bmm_dma_done, NULL);
    if (rc == SPI_BUS_OK)
    {
        return true;
    }
    s_dma_inflight = 0U;
    if (rc == SPI_BUS_BUSY)
    {
        imu_sched_request(IMU_SCHED_SENSOR_BMM150);
        return false;
    }

    imu_sched_request(IMU_SCHED_SENSOR_BMM150);
    if (s_dma_fail < 5U)
    {
        s_dma_fail++;
        APP_LOG_ERROR("BMM150 DMA start failed rc=%d busy=%d state=0x%lx err=0x%lx", rc,
                      spi_bus_is_busy(), (unsigned long)hspi6.State,
                      (unsigned long)HAL_SPI_GetError(&hspi6));
    }
    return false;
}

bool imu_bmm150_init(void)
{
    memset(&s_bmm, 0, sizeof(s_bmm));
    memset(&s_bmm_spi, 0, sizeof(s_bmm_spi));

    (void)spi_bus_init(&hspi6);
    spi_bus_device_init(&s_bmm_spi,
                        BMM150_CS_GPIO_Port,
                        BMM150_CS_Pin,
                        SPI_POLARITY_HIGH,
                        SPI_PHASE_2EDGE,
                        hspi6.Init.BaudRatePrescaler,
                        hspi6.Init.DataSize);

    s_bmm.intf = BMM150_SPI_INTF;
    s_bmm.read = bmm_spi_read;
    s_bmm.write = bmm_spi_write;
    s_bmm.delay_us = bmm_delay_us;
    s_bmm.intf_ptr = &s_bmm_spi;

    int8_t rslt = bmm150_init(&s_bmm);
    if (rslt != BMM150_OK)
    {
        APP_LOG_ERROR("BMM150 init failed rc=%d", rslt);
        return false;
    }
    APP_LOG_INFO("BMM150 chip id=0x%02x", s_bmm.chip_id);

    struct bmm150_settings settings = {0};
    settings.pwr_mode = BMM150_CFG_PWR_MODE;
    rslt = bmm150_set_op_mode(&settings, &s_bmm);
    if (rslt != BMM150_OK)
    {
        APP_LOG_ERROR("BMM150 set op mode failed rc=%d", rslt);
        return false;
    }

    settings.preset_mode = BMM150_CFG_PRESET_MODE;
    rslt = bmm150_set_presetmode(&settings, &s_bmm);
    if (rslt != BMM150_OK)
    {
        APP_LOG_ERROR("BMM150 set preset failed rc=%d", rslt);
        return false;
    }

    settings.int_settings.drdy_pin_en = BMM150_INT_ENABLE;
    settings.int_settings.drdy_polarity = BMM150_CFG_DRDY_POLARITY;
    settings.int_settings.int_pin_en = BMM150_INT_ENABLE;
    settings.int_settings.int_latch = BMM150_CFG_INT_LATCH;
    settings.int_settings.int_polarity = BMM150_CFG_INT_POLARITY;
    uint16_t desired = (uint16_t)(BMM150_SEL_DRDY_PIN_EN |
                                  BMM150_SEL_DRDY_POLARITY |
                                  BMM150_SEL_INT_PIN_EN |
                                  BMM150_SEL_INT_LATCH |
                                  BMM150_SEL_INT_POLARITY);
    rslt = bmm150_set_sensor_settings(desired, &settings, &s_bmm);
    if (rslt != BMM150_OK)
    {
        APP_LOG_ERROR("BMM150 int config failed rc=%d", rslt);
        return false;
    }

    s_init_ok = 1U;
    s_dma_inflight = 0U;
    s_sample_seq = 0U;
    memset(&s_latest_sample, 0, sizeof(s_latest_sample));
    memset(s_data_tx, 0, sizeof(s_data_tx));
    s_data_tx[0] = (uint8_t)(BMM150_DMA_DATA_START | BMM150_SPI_READ_MASK);
    s_irq_seen = 0U;
    s_irq_logged = 0U;
    s_irq_missing_logged = 0U;
    s_init_ms = HAL_GetTick();

    APP_LOG_INFO("BMM150 initialized (SPI6 + DMA)");
    return true;
}

void imu_bmm150_handle_int1(void)
{
    if (!s_init_ok)
    {
        return;
    }
    s_irq_seen++;
    imu_sched_request(IMU_SCHED_SENSOR_BMM150);
}

void imu_bmm150_poll(void)
{
    if (!s_init_ok)
    {
        return;
    }

    if (imu_bus_is_ready() && s_irq_seen != 0U && s_irq_logged == 0U)
    {
        s_irq_logged = 1U;
        APP_LOG_INFO("BMM150 INT1 seen");
    }
    else if (imu_bus_is_ready() && s_irq_seen == 0U && s_irq_missing_logged == 0U &&
             (HAL_GetTick() - s_init_ms) > 1000U)
    {
        s_irq_missing_logged = 1U;
        APP_LOG_ERROR("BMM150 INT1 not seen");
    }

    /* Diagnostics only; scheduler owns the bus. */
}

bool imu_bmm150_kick(void)
{
    if (!s_init_ok || s_dma_inflight || !imu_bus_is_ready())
    {
        return false;
    }
    return bmm_start_dma_read();
}

bool imu_bmm150_try_get_latest(imu_bmm150_sample_t *out, uint32_t *seq)
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
    imu_bmm150_sample_t sample = s_latest_sample;
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
