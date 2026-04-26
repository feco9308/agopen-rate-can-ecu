/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.h
  * @brief          : Header for USB CDC interface file.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_cdc.h"

#define APP_RX_DATA_SIZE  64U
#define APP_TX_DATA_SIZE  512U

extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);
uint8_t CDC_IsReady_FS(void);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H__ */
