#ifndef TESTS_FAKES_STM32H7XX_HAL_FDCAN_H
#define TESTS_FAKES_STM32H7XX_HAL_FDCAN_H

#include <stdint.h>

typedef struct {
    void *Instance;
} FDCAN_HandleTypeDef;

typedef struct {
    uint32_t Identifier;
    uint32_t IdType;
    uint32_t TxFrameType;
    uint32_t DataLength;
    uint32_t ErrorStateIndicator;
    uint32_t BitRateSwitch;
    uint32_t FDFormat;
    uint32_t TxEventFifoControl;
    uint32_t MessageMarker;
} FDCAN_TxHeaderTypeDef;

typedef struct {
    uint32_t Identifier;
    uint32_t IdType;
    uint32_t RxFrameType;
    uint32_t DataLength;
} FDCAN_RxHeaderTypeDef;

typedef struct {
    uint32_t IdType;
    uint32_t FilterIndex;
    uint32_t FilterType;
    uint32_t FilterConfig;
    uint32_t FilterID1;
    uint32_t FilterID2;
} FDCAN_FilterTypeDef;

#define FDCAN1 ((void *)0x1U)
#define FDCAN_STANDARD_ID 0U
#define FDCAN_DATA_FRAME 0U
#define FDCAN_ESI_ACTIVE 0U
#define FDCAN_BRS_OFF 0U
#define FDCAN_CLASSIC_CAN 0U
#define FDCAN_NO_TX_EVENTS 0U
#define FDCAN_FILTER_MASK 0U
#define FDCAN_FILTER_TO_RXFIFO0 0U
#define FDCAN_REJECT 0U
#define FDCAN_REJECT_REMOTE 0U
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE (1U << 0)
#define FDCAN_IT_BUS_OFF (1U << 1)
#define FDCAN_INTERRUPT_LINE0 0U
#define FDCAN_RX_FIFO0 0U

#define FDCAN_DLC_BYTES_0 0U
#define FDCAN_DLC_BYTES_1 1U
#define FDCAN_DLC_BYTES_2 2U
#define FDCAN_DLC_BYTES_3 3U
#define FDCAN_DLC_BYTES_4 4U
#define FDCAN_DLC_BYTES_5 5U
#define FDCAN_DLC_BYTES_6 6U
#define FDCAN_DLC_BYTES_7 7U
#define FDCAN_DLC_BYTES_8 8U

extern FDCAN_HandleTypeDef hfdcan1;

extern uint32_t g_fdcan_tx_count;
extern FDCAN_TxHeaderTypeDef g_fdcan_tx_headers[16];
extern uint8_t g_fdcan_tx_data[16][8];

int HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *hfdcan,
                                 FDCAN_TxHeaderTypeDef *pTxHeader,
                                 const uint8_t *pData);
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *hfdcan);
int HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *hfdcan,
                           FDCAN_FilterTypeDef *filter);
int HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *hfdcan,
                                 uint32_t non_matching_std,
                                 uint32_t non_matching_ext,
                                 uint32_t reject_remote_std,
                                 uint32_t reject_remote_ext);
int HAL_FDCAN_ConfigInterruptLines(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t active_its, uint32_t line);
int HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t active_its,
                                   uint32_t buffer_indexes);
int HAL_FDCAN_Start(FDCAN_HandleTypeDef *hfdcan);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *hfdcan,
                                      uint32_t rx_fifo);
int HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo,
                           FDCAN_RxHeaderTypeDef *header, uint8_t *data);

void hal_fake_fdcan_reset(void);
void hal_fake_fdcan_push_rx(uint32_t identifier, uint32_t data_length,
                            const uint8_t *data);

#endif /* TESTS_FAKES_STM32H7XX_HAL_FDCAN_H */
