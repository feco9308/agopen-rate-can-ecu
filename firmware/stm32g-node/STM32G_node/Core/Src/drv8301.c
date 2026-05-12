#include "drv8301.h"

#include <string.h>

enum
{
  DRV8301_CTRL1_DEFAULT = 0x0040u,
  DRV8301_CTRL1_VERIFY_PATTERN = 0x0040u,
  DRV8301_CTRL2_DEFAULT = 0x0000u,
  DRV8301_CTRL2_VERIFY_PATTERN = 0x0080u
};

static HAL_StatusTypeDef drv8301_transfer16(Drv8301Device *dev, uint16_t tx_word, uint16_t *rx_word)
{
  uint8_t tx_buf[2];
  uint8_t rx_buf[2];
  HAL_StatusTypeDef status;

  if ((dev == 0) || (dev->cfg.hspi == 0) || (rx_word == 0))
  {
    return HAL_ERROR;
  }

  tx_buf[0] = (uint8_t)(tx_word >> 8);
  tx_buf[1] = (uint8_t)(tx_word & 0xFFu);

  HAL_GPIO_WritePin(dev->cfg.cs_port, dev->cfg.cs_pin, GPIO_PIN_RESET);
  status = HAL_SPI_TransmitReceive(dev->cfg.hspi, tx_buf, rx_buf, 2u, 20u);
  HAL_GPIO_WritePin(dev->cfg.cs_port, dev->cfg.cs_pin, GPIO_PIN_SET);

  if (status == HAL_OK)
  {
    *rx_word = (uint16_t)(((uint16_t)rx_buf[0] << 8) | rx_buf[1]);
  }

  return status;
}

HAL_StatusTypeDef Drv8301_Init(Drv8301Device *dev, const Drv8301Config *cfg)
{
  if ((dev == 0) || (cfg == 0) || (cfg->hspi == 0) || (cfg->cs_port == 0) || (cfg->enable_port == 0) || (cfg->nfault_port == 0))
  {
    return HAL_ERROR;
  }

  memset(dev, 0, sizeof(*dev));
  dev->cfg = *cfg;

  HAL_GPIO_WritePin(dev->cfg.cs_port, dev->cfg.cs_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(dev->cfg.enable_port, dev->cfg.enable_pin, GPIO_PIN_RESET);
  HAL_Delay(1u);
  dev->control1_expected = DRV8301_CTRL1_VERIFY_PATTERN;
  dev->control2_expected = DRV8301_CTRL2_VERIFY_PATTERN;

  return HAL_OK;
}

HAL_StatusTypeDef Drv8301_EnableGate(Drv8301Device *dev, uint8_t enabled)
{
  if (dev == 0)
  {
    return HAL_ERROR;
  }

  HAL_GPIO_WritePin(dev->cfg.enable_port,
                    dev->cfg.enable_pin,
                    (enabled != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);

  dev->gate_enabled = (enabled != 0u) ? 1u : 0u;
  HAL_Delay(2u);

  return HAL_OK;
}

GPIO_PinState Drv8301_ReadFaultPin(const Drv8301Device *dev)
{
  if (dev == 0)
  {
    return GPIO_PIN_SET;
  }

  return HAL_GPIO_ReadPin(dev->cfg.nfault_port, dev->cfg.nfault_pin);
}

HAL_StatusTypeDef Drv8301_ReadRegister(Drv8301Device *dev, Drv8301Register reg, uint16_t *value)
{
  uint16_t rx_word = 0u;
  uint16_t cmd = (uint16_t)(0x8000u | (((uint16_t)reg & 0x0Fu) << 11));
  HAL_StatusTypeDef status;

  if ((dev == 0) || (value == 0))
  {
    return HAL_ERROR;
  }

  status = drv8301_transfer16(dev, cmd, &rx_word);
  if (status != HAL_OK)
  {
    dev->spi_ok = 0u;
    return status;
  }

  status = drv8301_transfer16(dev, 0u, &rx_word);
  if (status != HAL_OK)
  {
    dev->spi_ok = 0u;
    return status;
  }

  *value = (uint16_t)(rx_word & 0x07FFu);
  dev->spi_ok = 1u;
  return HAL_OK;
}

HAL_StatusTypeDef Drv8301_WriteRegister(Drv8301Device *dev, Drv8301Register reg, uint16_t value)
{
  uint16_t rx_word = 0u;
  uint16_t cmd = (uint16_t)((((uint16_t)reg & 0x0Fu) << 11) | (value & 0x07FFu));
  HAL_StatusTypeDef status;

  if (dev == 0)
  {
    return HAL_ERROR;
  }

  status = drv8301_transfer16(dev, cmd, &rx_word);
  if (status != HAL_OK)
  {
    dev->spi_ok = 0u;
    return status;
  }

  dev->spi_ok = 1u;
  return HAL_OK;
}

HAL_StatusTypeDef Drv8301_ReadAll(Drv8301Device *dev)
{
  HAL_StatusTypeDef status;

  if (dev == 0)
  {
    return HAL_ERROR;
  }

  status = Drv8301_ReadRegister(dev, DRV8301_REG_STATUS1, &dev->status1);
  if (status != HAL_OK)
  {
    return status;
  }

  status = Drv8301_ReadRegister(dev, DRV8301_REG_STATUS2, &dev->status2);
  if (status != HAL_OK)
  {
    return status;
  }

  status = Drv8301_ReadRegister(dev, DRV8301_REG_CONTROL1, &dev->control1);
  if (status != HAL_OK)
  {
    return status;
  }

  status = Drv8301_ReadRegister(dev, DRV8301_REG_CONTROL2, &dev->control2);
  if (status != HAL_OK)
  {
    return status;
  }

  return HAL_OK;
}

HAL_StatusTypeDef Drv8301_VerifyControl1(Drv8301Device *dev)
{
  HAL_StatusTypeDef status;
  uint16_t value = 0u;

  if (dev == 0)
  {
    return HAL_ERROR;
  }

  status = Drv8301_WriteRegister(dev, DRV8301_REG_CONTROL1, dev->control1_expected);
  if (status != HAL_OK)
  {
    dev->verify_control1_ok = 0u;
    return status;
  }

  status = Drv8301_ReadRegister(dev, DRV8301_REG_CONTROL1, &value);
  if (status != HAL_OK)
  {
    dev->verify_control1_ok = 0u;
    return status;
  }

  dev->control1 = value;

  if (value == dev->control1_expected)
  {
    dev->verify_control1_ok = 1u;
    dev->verify_control1_mismatch_count = 0u;
    return HAL_OK;
  }

  dev->verify_control1_ok = 0u;
  if (dev->verify_control1_mismatch_count < 255u)
  {
    dev->verify_control1_mismatch_count++;
  }

  return HAL_ERROR;
}

HAL_StatusTypeDef Drv8301_VerifyControl2(Drv8301Device *dev)
{
  HAL_StatusTypeDef status;
  uint16_t value = 0u;

  if (dev == 0)
  {
    return HAL_ERROR;
  }

  status = Drv8301_WriteRegister(dev, DRV8301_REG_CONTROL2, dev->control2_expected);
  if (status != HAL_OK)
  {
    dev->verify_ok = 0u;
    return status;
  }

  status = Drv8301_ReadRegister(dev, DRV8301_REG_CONTROL2, &value);
  if (status != HAL_OK)
  {
    dev->verify_ok = 0u;
    return status;
  }

  dev->control2 = value;

  if (value == dev->control2_expected)
  {
    dev->verify_ok = 1u;
    dev->verify_mismatch_count = 0u;
    return HAL_OK;
  }

  dev->verify_ok = 0u;
  if (dev->verify_mismatch_count < 255u)
  {
    dev->verify_mismatch_count++;
  }

  return HAL_ERROR;
}
