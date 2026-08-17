#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_tim.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_hal_fdcan.h"
#include "main.h"
#include <string.h>

TIM_TypeDef g_tim2_instance = {0};
TIM_HandleTypeDef htim2 = {
    .Instance = &g_tim2_instance,
    .Init = {
        .Prescaler = 0,
        .Period = 0,
    },
};

GPIO_TypeDef g_gpioa = {0};
GPIO_TypeDef g_gpiod = {0};

uint8_t g_gpio_left_upper_state = GPIO_PIN_RESET;
uint8_t g_gpio_left_lower_state = GPIO_PIN_RESET;
uint8_t g_gpio_right_upper_state = GPIO_PIN_RESET;
uint8_t g_gpio_right_lower_state = GPIO_PIN_RESET;

FDCAN_HandleTypeDef hfdcan1 = {.Instance = FDCAN1};

enum { TEST_FDCAN_TX_MAX = 16 };
enum { TEST_FDCAN_RX_MAX = 16 };
uint32_t g_fdcan_tx_count = 0;
FDCAN_TxHeaderTypeDef g_fdcan_tx_headers[TEST_FDCAN_TX_MAX];
uint8_t g_fdcan_tx_data[TEST_FDCAN_TX_MAX][8];

typedef struct {
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];
} test_fdcan_rx_frame_t;

static test_fdcan_rx_frame_t s_fdcan_rx[TEST_FDCAN_RX_MAX];
static uint32_t s_fdcan_rx_head = 0U;
static uint32_t s_fdcan_rx_tail = 0U;

uint32_t g_hal_pclk1_freq = 100000000U;
uint32_t g_hal_hclk_freq = 200000000U;
uint32_t g_hal_apb1_prescaler = LL_RCC_APB1_DIV_1;
uint32_t g_hal_tick = 0U;
uint32_t g_hal_delay_calls = 0U;

volatile uint32_t test_dwt_ctrl = 0;
volatile uint32_t test_dwt_cyccnt = 0;
volatile uint32_t test_coredebug_demcr = 0;

uint32_t HAL_RCC_GetPCLK1Freq(void)
{
    return g_hal_pclk1_freq;
}

uint32_t HAL_RCC_GetHCLKFreq(void)
{
    return g_hal_hclk_freq;
}

uint32_t HAL_GetTick(void)
{
    return g_hal_tick;
}

void HAL_Delay(uint32_t delay_ms)
{
    g_hal_delay_calls++;
    g_hal_tick += delay_ms;
}

uint32_t LL_RCC_GetAPB1Prescaler(void)
{
    return g_hal_apb1_prescaler;
}

void HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim)
{
    if (htim && htim->Instance) {
        htim->Instance->CR1 |= TIM_CR1_CEN;
    }
}

void HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *htim)
{
    if (htim && htim->Instance) {
        htim->Instance->CR1 &= ~TIM_CR1_CEN;
    }
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    if (port == GPIOA && pin == LeftHipUpperLimit_Pin) {
        return g_gpio_left_upper_state ? GPIO_PIN_SET : GPIO_PIN_RESET;
    }
    if (port == GPIOA && pin == LeftHipLowerLimit_Pin) {
        return g_gpio_left_lower_state ? GPIO_PIN_SET : GPIO_PIN_RESET;
    }
    if (port == GPIOD && pin == RightHipUpperLimit_Pin) {
        return g_gpio_right_upper_state ? GPIO_PIN_SET : GPIO_PIN_RESET;
    }
    if (port == GPIOD && pin == RightHipLowerLimit_Pin) {
        return g_gpio_right_lower_state ? GPIO_PIN_SET : GPIO_PIN_RESET;
    }
    return GPIO_PIN_SET;
}

int HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *hfdcan,
                                 FDCAN_TxHeaderTypeDef *pTxHeader,
                                 const uint8_t *pData)
{
    (void)hfdcan;
    if (g_fdcan_tx_count < TEST_FDCAN_TX_MAX) {
        g_fdcan_tx_headers[g_fdcan_tx_count] = *pTxHeader;
        if (pData != NULL) {
            memcpy(g_fdcan_tx_data[g_fdcan_tx_count], pData, 8U);
        } else {
            memset(g_fdcan_tx_data[g_fdcan_tx_count], 0, 8U);
        }
    }
    g_fdcan_tx_count++;
    return HAL_OK;
}

uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
    return 1U;
}

int HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *hfdcan,
                           FDCAN_FilterTypeDef *filter)
{
    (void)hfdcan;
    (void)filter;
    return HAL_OK;
}

int HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *hfdcan,
                                 uint32_t non_matching_std,
                                 uint32_t non_matching_ext,
                                 uint32_t reject_remote_std,
                                 uint32_t reject_remote_ext)
{
    (void)hfdcan;
    (void)non_matching_std;
    (void)non_matching_ext;
    (void)reject_remote_std;
    (void)reject_remote_ext;
    return HAL_OK;
}

int HAL_FDCAN_ConfigInterruptLines(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t active_its, uint32_t line)
{
    (void)hfdcan;
    (void)active_its;
    (void)line;
    return HAL_OK;
}

int HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t active_its,
                                   uint32_t buffer_indexes)
{
    (void)hfdcan;
    (void)active_its;
    (void)buffer_indexes;
    return HAL_OK;
}

int HAL_FDCAN_Start(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
    return HAL_OK;
}

uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *hfdcan,
                                      uint32_t rx_fifo)
{
    (void)hfdcan;
    (void)rx_fifo;
    return s_fdcan_rx_head - s_fdcan_rx_tail;
}

int HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo,
                           FDCAN_RxHeaderTypeDef *header, uint8_t *data)
{
    (void)hfdcan;
    (void)rx_fifo;
    if (header == NULL || data == NULL || s_fdcan_rx_tail == s_fdcan_rx_head) {
        return HAL_ERROR;
    }

    test_fdcan_rx_frame_t *frame =
        &s_fdcan_rx[s_fdcan_rx_tail % TEST_FDCAN_RX_MAX];
    *header = frame->header;
    memcpy(data, frame->data, sizeof(frame->data));
    s_fdcan_rx_tail++;
    return HAL_OK;
}

void hal_fake_fdcan_reset(void)
{
    g_fdcan_tx_count = 0U;
    memset(g_fdcan_tx_headers, 0, sizeof(g_fdcan_tx_headers));
    memset(g_fdcan_tx_data, 0, sizeof(g_fdcan_tx_data));
    memset(s_fdcan_rx, 0, sizeof(s_fdcan_rx));
    s_fdcan_rx_head = 0U;
    s_fdcan_rx_tail = 0U;
}

void hal_fake_fdcan_push_rx(uint32_t identifier, uint32_t data_length,
                            const uint8_t *data)
{
    if ((s_fdcan_rx_head - s_fdcan_rx_tail) >= TEST_FDCAN_RX_MAX) {
        return;
    }

    test_fdcan_rx_frame_t *frame =
        &s_fdcan_rx[s_fdcan_rx_head % TEST_FDCAN_RX_MAX];
    frame->header.Identifier = identifier;
    frame->header.IdType = FDCAN_STANDARD_ID;
    frame->header.RxFrameType = FDCAN_DATA_FRAME;
    frame->header.DataLength = data_length;
    if (data != NULL) {
        memcpy(frame->data, data, sizeof(frame->data));
    } else {
        memset(frame->data, 0, sizeof(frame->data));
    }
    s_fdcan_rx_head++;
}

uint32_t __get_PRIMASK(void)
{
    return 0;
}

void __set_PRIMASK(uint32_t primask)
{
    (void)primask;
}

void __disable_irq(void)
{
}

void __enable_irq(void)
{
}
