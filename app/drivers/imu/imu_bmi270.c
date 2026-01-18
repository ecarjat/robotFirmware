#include "imu_bmi270.h"

#include <string.h>

#include "app_config.h"
#include "bmi270.h"
#include "bmi2.h"
#include "bmi2_defs.h"
#include "imu_bmi270_config.h"
#include "imu_bus.h"
#include "imu_sched.h"
#include "main.h"
#include "spi_bus.h"

#define BMI270_SPI_READ_MASK    0x80U
#define BMI270_SPI_MAX_XFER     64U
#define BMI270_REG_TIMEOUT_MS   2U
#define BMI270_DMA_DATA_START   BMI2_STATUS_ADDR
#define BMI270_DMA_DATA_LEN     (BMI2_TEMPERATURE_1_ADDR - BMI2_STATUS_ADDR + 1U)
#define BMI270_DMA_FRAME_LEN    (1U + 1U + BMI270_DMA_DATA_LEN)
#define BMI270_DMA_RX_OFFSET    2U
#define BMI270_ACC_OFFSET       (BMI2_ACC_X_LSB_ADDR - BMI270_DMA_DATA_START)
#define BMI270_TEMP_OFFSET      (BMI2_TEMPERATURE_0_ADDR - BMI270_DMA_DATA_START)

extern SPI_HandleTypeDef hspi6;

static struct bmi2_dev s_bmi;
static spi_bus_device_t s_bmi_spi;
static volatile uint8_t s_init_ok = 0U;
static volatile uint8_t s_dma_inflight = 0U;
static volatile uint32_t s_sample_seq = 0U;
static imu_bmi270_sample_t s_latest_sample;
static uint8_t s_dma_fail = 0U;
static volatile uint32_t s_irq_seen = 0U;
static uint8_t s_irq_logged = 0U;
static uint8_t s_irq_missing_logged = 0U;
static uint32_t s_init_ms = 0U;
static volatile uint32_t s_last_irq_ms = 0U;
static uint32_t s_dma_timestamp_ms = 0U;
static uint8_t s_data_tx[BMI270_DMA_FRAME_LEN] __attribute__((section(".bdma_buffer"), aligned(32)));
static uint8_t s_data_rx[BMI270_DMA_FRAME_LEN] __attribute__((section(".bdma_buffer"), aligned(32)));

static BMI2_INTF_RETURN_TYPE bmi_spi_read(uint8_t reg_addr,
                                          uint8_t *reg_data,
                                          uint32_t len,
                                          void *intf_ptr);
static BMI2_INTF_RETURN_TYPE bmi_spi_write(uint8_t reg_addr,
                                           const uint8_t *reg_data,
                                           uint32_t len,
                                           void *intf_ptr);
static void bmi_delay_us(uint32_t period, void *intf_ptr);
static void bmi_store_sample(const imu_bmi270_sample_t *sample);
static void bmi_parse_sample(const uint8_t *data, imu_bmi270_sample_t *sample);
static void bmi_dma_done(void *ctx, int status);
static bool bmi_start_dma_read(void);

static BMI2_INTF_RETURN_TYPE bmi_spi_read(uint8_t reg_addr,
                                          uint8_t *reg_data,
                                          uint32_t len,
                                          void *intf_ptr)
{
    if (reg_data == NULL || len == 0U || len > BMI270_SPI_MAX_XFER || intf_ptr == NULL)
    {
        return BMI2_E_INVALID_SENSOR;
    }

    spi_bus_device_t *dev = (spi_bus_device_t *)intf_ptr;
    uint8_t tx[BMI270_SPI_MAX_XFER + 1U];
    uint8_t rx[BMI270_SPI_MAX_XFER + 1U];

    tx[0] = (uint8_t)(reg_addr | BMI270_SPI_READ_MASK);
    memset(&tx[1], 0x00, len);

    if (spi_bus_transfer_blocking(dev, tx, rx, len + 1U, BMI270_REG_TIMEOUT_MS) != 0)
    {
        return BMI2_E_COM_FAIL;
    }

    memcpy(reg_data, &rx[1], len);
    return BMI2_OK;
}

static BMI2_INTF_RETURN_TYPE bmi_spi_write(uint8_t reg_addr,
                                           const uint8_t *reg_data,
                                           uint32_t len,
                                           void *intf_ptr)
{
    if (reg_data == NULL || len == 0U || len > BMI270_SPI_MAX_XFER || intf_ptr == NULL)
    {
        return BMI2_E_INVALID_SENSOR;
    }

    spi_bus_device_t *dev = (spi_bus_device_t *)intf_ptr;
    uint8_t tx[BMI270_SPI_MAX_XFER + 1U];
    uint8_t rx[BMI270_SPI_MAX_XFER + 1U];

    tx[0] = (uint8_t)(reg_addr & ~BMI270_SPI_READ_MASK);
    memcpy(&tx[1], reg_data, len);

    if (spi_bus_transfer_blocking(dev, tx, rx, len + 1U, BMI270_REG_TIMEOUT_MS) != 0)
    {
        return BMI2_E_COM_FAIL;
    }

    return BMI2_OK;
}

static void bmi_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    if (period == 0U)
    {
        return;
    }
    uint32_t ms = (period + 999U) / 1000U;
    HAL_Delay(ms);
}

static void bmi_store_sample(const imu_bmi270_sample_t *sample)
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

static void bmi_parse_sample(const uint8_t *data, imu_bmi270_sample_t *sample)
{
    if (data == NULL || sample == NULL)
    {
        return;
    }

    sample->timestamp_ms = s_dma_timestamp_ms;
    const uint8_t *acc = &data[BMI270_ACC_OFFSET];
    sample->accel[0] = (int16_t)((uint16_t)acc[1] << 8 | acc[0]);
    sample->accel[1] = (int16_t)((uint16_t)acc[3] << 8 | acc[2]);
    sample->accel[2] = (int16_t)((uint16_t)acc[5] << 8 | acc[4]);
    sample->gyro[0] = (int16_t)((uint16_t)acc[7] << 8 | acc[6]);
    sample->gyro[1] = (int16_t)((uint16_t)acc[9] << 8 | acc[8]);
    sample->gyro[2] = (int16_t)((uint16_t)acc[11] << 8 | acc[10]);
    sample->temperature = (int16_t)((uint16_t)data[BMI270_TEMP_OFFSET + 1U] << 8 |
                                    data[BMI270_TEMP_OFFSET]);
}

static void bmi_dma_done(void *ctx, int status)
{
    (void)ctx;
    s_dma_inflight = 0U;
    if (status == 0)
    {
        imu_bmi270_sample_t sample;
        bmi_parse_sample(&s_data_rx[BMI270_DMA_RX_OFFSET], &sample);
        bmi_store_sample(&sample);
    }
    imu_sched_on_dma_done(IMU_SCHED_SENSOR_BMI270, status);
}

static bool bmi_start_dma_read(void)
{
    if (!s_init_ok)
    {
        return false;
    }
    if (__LDREXB(&s_dma_inflight) || __STREXB(1U, &s_dma_inflight))
    {
        return false;
    }

    s_dma_timestamp_ms = s_last_irq_ms;
    if (s_dma_timestamp_ms == 0U)
    {
        s_dma_timestamp_ms = HAL_GetTick();
    }

    int rc = spi_bus_transfer_dma(&s_bmi_spi, s_data_tx, s_data_rx, sizeof(s_data_tx), bmi_dma_done, NULL);
    if (rc == SPI_BUS_OK)
    {
        return true;
    }
    s_dma_inflight = 0U;
    if (rc == SPI_BUS_BUSY)
    {
        imu_sched_request(IMU_SCHED_SENSOR_BMI270);
        return false;
    }

    imu_sched_request(IMU_SCHED_SENSOR_BMI270);
    if (s_dma_fail < 5U)
    {
        s_dma_fail++;
        APP_LOG_ERROR("BMI270 DMA start failed rc=%d busy=%d state=0x%lx err=0x%lx", rc,
                      spi_bus_is_busy(), (unsigned long)hspi6.State,
                      (unsigned long)HAL_SPI_GetError(&hspi6));
    }
    return false;
}

bool imu_bmi270_init(void)
{
    memset(&s_bmi, 0, sizeof(s_bmi));
    memset(&s_bmi_spi, 0, sizeof(s_bmi_spi));

    (void)spi_bus_init(&hspi6);
    spi_bus_device_init(&s_bmi_spi,
                        BMI270_CS_GPIO_Port,
                        BMI270_CS_Pin,
                        SPI_POLARITY_HIGH,
                        SPI_PHASE_2EDGE,
                        hspi6.Init.BaudRatePrescaler,
                        hspi6.Init.DataSize);

    s_bmi.intf = BMI2_SPI_INTF;
    s_bmi.read = bmi_spi_read;
    s_bmi.write = bmi_spi_write;
    s_bmi.delay_us = bmi_delay_us;
    s_bmi.intf_ptr = &s_bmi_spi;
    s_bmi.read_write_len = BMI270_CFG_READ_WRITE_LEN;
    s_bmi.config_file_ptr = NULL;

    int8_t rslt = bmi270_init(&s_bmi);
    if (rslt != BMI2_OK)
    {
        APP_LOG_ERROR("BMI270 init failed rc=%d", rslt);
        return false;
    }

    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };
    rslt = bmi270_sensor_enable(sens_list, 2, &s_bmi);
    if (rslt != BMI2_OK)
    {
        APP_LOG_ERROR("BMI270 sensor enable failed rc=%d", rslt);
        return false;
    }

    struct bmi2_sens_config cfg[2] = {0};
    cfg[0].type = BMI2_ACCEL;
    cfg[1].type = BMI2_GYRO;

    rslt = bmi2_get_sensor_config(cfg, 2, &s_bmi);
    if (rslt != BMI2_OK)
    {
        APP_LOG_ERROR("BMI270 get config failed rc=%d", rslt);
        return false;
    }

    cfg[0].cfg.acc.odr = BMI270_CFG_ACC_ODR;
    cfg[0].cfg.acc.range = BMI270_CFG_ACC_RANGE;
    cfg[0].cfg.acc.bwp = BMI270_CFG_ACC_BWP;
    cfg[0].cfg.acc.filter_perf = BMI270_CFG_ACC_FILTER_PERF;

    cfg[1].cfg.gyr.odr = BMI270_CFG_GYR_ODR;
    cfg[1].cfg.gyr.range = BMI270_CFG_GYR_RANGE;
    cfg[1].cfg.gyr.bwp = BMI270_CFG_GYR_BWP;
    cfg[1].cfg.gyr.noise_perf = BMI270_CFG_GYR_NOISE_PERF;
    cfg[1].cfg.gyr.filter_perf = BMI270_CFG_GYR_FILTER_PERF;

    rslt = bmi2_set_sensor_config(cfg, 2, &s_bmi);
    if (rslt != BMI2_OK)
    {
        APP_LOG_ERROR("BMI270 set config failed rc=%d", rslt);
        return false;
    }

    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, &s_bmi);
    if (rslt != BMI2_OK)
    {
        APP_LOG_ERROR("BMI270 map DRDY failed rc=%d", rslt);
        return false;
    }

    struct bmi2_int_pin_config pin_cfg = {0};
    rslt = bmi2_get_int_pin_config(&pin_cfg, &s_bmi);
    if (rslt != BMI2_OK)
    {
        APP_LOG_ERROR("BMI270 get int pin cfg failed rc=%d", rslt);
        return false;
    }

    pin_cfg.pin_type = BMI2_INT1;
    pin_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;
    pin_cfg.pin_cfg[0].lvl = BMI270_CFG_INT_LVL;
    pin_cfg.pin_cfg[0].od = BMI270_CFG_INT_OD;
    pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    pin_cfg.int_latch = BMI270_CFG_INT_LATCH;

    rslt = bmi2_set_int_pin_config(&pin_cfg, &s_bmi);
    if (rslt != BMI2_OK)
    {
        APP_LOG_ERROR("BMI270 set int pin cfg failed rc=%d", rslt);
        return false;
    }

    s_init_ok = 1U;
    s_dma_inflight = 0U;
    s_sample_seq = 0U;
    memset(&s_latest_sample, 0, sizeof(s_latest_sample));
    memset(s_data_tx, 0, sizeof(s_data_tx));
    s_data_tx[0] = (uint8_t)(BMI270_DMA_DATA_START | BMI270_SPI_READ_MASK);
    s_irq_seen = 0U;
    s_irq_logged = 0U;
    s_irq_missing_logged = 0U;
    s_init_ms = HAL_GetTick();
    s_last_irq_ms = 0U;
    s_dma_timestamp_ms = 0U;

    APP_LOG_INFO("BMI270 initialized (SPI6)");
    return true;
}

void imu_bmi270_handle_int1(void)
{
    if (!s_init_ok)
    {
        return;
    }
    s_last_irq_ms = HAL_GetTick();
    s_irq_seen++;
    imu_sched_request(IMU_SCHED_SENSOR_BMI270);
}

void imu_bmi270_poll(void)
{
    if (!s_init_ok)
    {
        return;
    }

    if (imu_bus_is_ready() && s_irq_seen != 0U && s_irq_logged == 0U)
    {
        s_irq_logged = 1U;
        APP_LOG_INFO("BMI270 INT1 seen");
    }
    else if (imu_bus_is_ready() && s_irq_seen == 0U && s_irq_missing_logged == 0U &&
             (HAL_GetTick() - s_init_ms) > 1000U)
    {
        s_irq_missing_logged = 1U;
        APP_LOG_ERROR("BMI270 INT1 not seen");
    }

    /* Diagnostics only; scheduler owns the bus. */
}

bool imu_bmi270_kick(void)
{
    if (!s_init_ok || s_dma_inflight || !imu_bus_is_ready())
    {
        return false;
    }
    return bmi_start_dma_read();
}

bool imu_bmi270_try_get_latest(imu_bmi270_sample_t *out, uint32_t *seq)
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
    imu_bmi270_sample_t sample = s_latest_sample;
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

uint32_t imu_bmi270_get_last_irq_ms(void)
{
    return s_last_irq_ms;
}
