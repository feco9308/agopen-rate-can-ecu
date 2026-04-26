#ifndef STM32G_NODE_STUB_FDCAN_H
#define STM32G_NODE_STUB_FDCAN_H

#include <stdint.h>

typedef enum
{
    HAL_OK = 0x00U,
    HAL_ERROR = 0x01U
} HAL_StatusTypeDef;

typedef struct
{
    uint32_t Instance;
} FDCAN_HandleTypeDef;

typedef struct
{
    uint32_t Identifier;
    uint32_t IdType;
    uint32_t RxFrameType;
    uint32_t DataLength;
    uint32_t ErrorStateIndicator;
    uint32_t BitRateSwitch;
    uint32_t FDFormat;
    uint32_t FilterIndex;
    uint32_t IsFilterMatchingFrame;
    uint32_t RxTimestamp;
} FDCAN_RxHeaderTypeDef;

typedef struct
{
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

#define FDCAN_STANDARD_ID 0x00000000U
#define FDCAN_DATA_FRAME 0x00000000U
#define FDCAN_DLC_BYTES_8 0x00000008U
#define FDCAN_ESI_ACTIVE 0x00000000U
#define FDCAN_BRS_OFF 0x00000000U
#define FDCAN_CLASSIC_CAN 0x00000000U
#define FDCAN_NO_TX_EVENTS 0x00000000U

HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *hfdcan,
                                                const FDCAN_TxHeaderTypeDef *pTxHeader,
                                                uint8_t *pTxData);

#endif
