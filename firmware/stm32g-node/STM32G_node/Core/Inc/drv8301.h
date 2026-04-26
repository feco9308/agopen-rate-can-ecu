#ifndef DRV8301_H
#define DRV8301_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  DRV8301_REG_STATUS1 = 0x00u,
  DRV8301_REG_STATUS2 = 0x01u,
  DRV8301_REG_CONTROL1 = 0x02u,
  DRV8301_REG_CONTROL2 = 0x03u
} Drv8301Register;

typedef struct
{
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
  GPIO_TypeDef *enable_port;
  uint16_t enable_pin;
  GPIO_TypeDef *nfault_port;
  uint16_t nfault_pin;
} Drv8301Config;

typedef struct
{
  Drv8301Config cfg;
  uint16_t status1;
  uint16_t status2;
  uint16_t control1;
  uint16_t control2;
  uint16_t control1_expected;
  uint16_t control2_expected;
  uint8_t spi_ok;
  uint8_t gate_enabled;
  uint8_t verify_control1_ok;
  uint8_t verify_ok;
  uint8_t verify_control1_mismatch_count;
  uint8_t verify_mismatch_count;
} Drv8301Device;

HAL_StatusTypeDef Drv8301_Init(Drv8301Device *dev, const Drv8301Config *cfg);
HAL_StatusTypeDef Drv8301_EnableGate(Drv8301Device *dev, uint8_t enabled);
GPIO_PinState Drv8301_ReadFaultPin(const Drv8301Device *dev);
HAL_StatusTypeDef Drv8301_ReadRegister(Drv8301Device *dev, Drv8301Register reg, uint16_t *value);
HAL_StatusTypeDef Drv8301_WriteRegister(Drv8301Device *dev, Drv8301Register reg, uint16_t value);
HAL_StatusTypeDef Drv8301_ReadAll(Drv8301Device *dev);
HAL_StatusTypeDef Drv8301_VerifyControl1(Drv8301Device *dev);
HAL_StatusTypeDef Drv8301_VerifyControl2(Drv8301Device *dev);

#ifdef __cplusplus
}
#endif

#endif
