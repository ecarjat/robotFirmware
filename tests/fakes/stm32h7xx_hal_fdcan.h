#ifndef TESTS_FAKES_STM32H7XX_HAL_FDCAN_H
#define TESTS_FAKES_STM32H7XX_HAL_FDCAN_H

#include <stdint.h>

typedef struct {
    volatile uint32_t dummy;
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

#define FDCAN_STANDARD_ID 0U
#define FDCAN_DATA_FRAME 0U
#define FDCAN_ESI_ACTIVE 0U
#define FDCAN_BRS_OFF 0U
#define FDCAN_CLASSIC_CAN 0U
#define FDCAN_NO_TX_EVENTS 0U

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

#endif /* TESTS_FAKES_STM32H7XX_HAL_FDCAN_H */
