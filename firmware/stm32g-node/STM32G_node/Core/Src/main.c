/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_debug.h"
#include "app_hall.h"
#include "app_motor_control.h"
#include "app_motor_pwm.h"
#include "drv8301.h"
#include "motor_node_can.h"
#include "stm32g4xx_hal_spi.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN PV */
static AppDebugState g_app_debug;
static AppHall g_hall;
static AppMotorControl g_motor_control;
static AppMotorPwm g_motor_pwm;
static Drv8301Device g_drv8301;
static MotorNodeCan g_motor_node;
static SPI_HandleTypeDef hspi1;
static uint16_t g_as5048a_last_angle_raw;
static uint8_t g_as5048a_sensor_ok;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */
static void APP_SPI1_Init(void);
static void APP_Hall_Init(void);
static void APP_Hall_Poll(void);
static void APP_MotorPwm_Init(void);
static void APP_MotorControl_Init(void);
static void APP_MotorControl_Poll(void);
static void APP_DRV8301_Init(void);
static void APP_DRV8301_Poll(void);
static void APP_DRV8301_SendDebugFrame(void);
static void APP_Debug_Init(void);
static void APP_Debug_LogPeriodic(void);
static void APP_FDCAN_Start(void);
static void APP_MotorNode_Init(void);
static void APP_MotorNode_PollRx(void);
static void APP_MotorNode_Update(void);
static void APP_Heartbeat_Update(void);
static void APP_Fault(AppDebugFault fault);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static HAL_StatusTypeDef APP_FDCAN_SendStdFrame(uint16_t can_id, const uint8_t data[8])
{
  FDCAN_TxHeaderTypeDef tx_header;

  tx_header.Identifier = can_id;
  tx_header.IdType = FDCAN_STANDARD_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.DataLength = FDCAN_DLC_BYTES_8;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_OFF;
  tx_header.FDFormat = FDCAN_CLASSIC_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0u;

  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, (uint8_t *)data);
}

static void APP_SPI1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_SPI1_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_GPIO_WritePin(AS5048A_CS_GPIO_Port, AS5048A_CS_Pin, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = AS5048A_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = 0u;
  HAL_GPIO_Init(AS5048A_CS_GPIO_Port, &GPIO_InitStruct);

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void APP_Hall_Init(void)
{
  if (AppHall_Init(&g_hall) != HAL_OK)
  {
    AppDebug_SetSensorState(&g_app_debug, APP_DEBUG_BLOCK_FAULT);
    AppDebug_SetFault(&g_app_debug, APP_DEBUG_FAULT_SENSOR_INIT);
    return;
  }

  if (AppHall_IsReady(&g_hall) != 0u)
  {
    AppDebug_SetSensorState(&g_app_debug, APP_DEBUG_BLOCK_READY);
  }
  else
  {
    AppDebug_SetSensorState(&g_app_debug, APP_DEBUG_BLOCK_INIT);
  }
}

static void APP_Hall_Poll(void)
{
  if (AppHall_Poll(&g_hall, HAL_GetTick()) != HAL_OK)
  {
    AppDebug_SetSensorState(&g_app_debug, APP_DEBUG_BLOCK_FAULT);
    AppDebug_SetFault(&g_app_debug, APP_DEBUG_FAULT_SENSOR_READ);
    return;
  }

  if (AppHall_IsReady(&g_hall) != 0u)
  {
    AppDebug_SetSensorState(&g_app_debug, APP_DEBUG_BLOCK_READY);
    AppDebug_ClearFault(&g_app_debug, APP_DEBUG_FAULT_SENSOR_INIT);
    AppDebug_ClearFault(&g_app_debug, APP_DEBUG_FAULT_SENSOR_READ);
  }
  else
  {
    AppDebug_SetSensorState(&g_app_debug, APP_DEBUG_BLOCK_INIT);
  }
}

static void APP_MotorPwm_Init(void)
{
  if (AppMotorPwm_Init(&g_motor_pwm) != HAL_OK)
  {
    AppDebug_SetControlState(&g_app_debug, APP_DEBUG_BLOCK_FAULT);
    AppDebug_SetFault(&g_app_debug, APP_DEBUG_FAULT_MOTOR_CONTROL);
    return;
  }

  if (AppMotorPwm_SetDutyPermille(&g_motor_pwm, 500u, 500u, 500u) != HAL_OK)
  {
    AppDebug_SetControlState(&g_app_debug, APP_DEBUG_BLOCK_FAULT);
    AppDebug_SetFault(&g_app_debug, APP_DEBUG_FAULT_MOTOR_CONTROL);
    return;
  }

  AppDebug_SetControlState(&g_app_debug, APP_DEBUG_BLOCK_READY);
}

static void APP_MotorControl_Init(void)
{
  if (AppMotorControl_Init(&g_motor_control, &g_motor_pwm, 7u) != HAL_OK)
  {
    AppDebug_SetControlState(&g_app_debug, APP_DEBUG_BLOCK_FAULT);
    AppDebug_SetFault(&g_app_debug, APP_DEBUG_FAULT_MOTOR_CONTROL);
    return;
  }

  AppDebug_SetControlState(&g_app_debug, APP_DEBUG_BLOCK_READY);
}

static void APP_MotorControl_Poll(void)
{
  if (AppMotorControl_SetCommand(&g_motor_control,
                                 MotorNodeCan_IsRunAllowed(&g_motor_node) ? 1u : 0u,
                                 MotorNodeCan_GetTargetRpm(&g_motor_node)) != HAL_OK)
  {
    AppDebug_SetControlState(&g_app_debug, APP_DEBUG_BLOCK_FAULT);
    AppDebug_SetFault(&g_app_debug, APP_DEBUG_FAULT_MOTOR_CONTROL);
    return;
  }

  if (AppMotorControl_Poll(&g_motor_control, &g_hall, HAL_GetTick()) != HAL_OK)
  {
    AppDebug_SetControlState(&g_app_debug, APP_DEBUG_BLOCK_FAULT);
    AppDebug_SetFault(&g_app_debug, APP_DEBUG_FAULT_MOTOR_CONTROL);
    return;
  }

  AppDebug_SetControlState(&g_app_debug, APP_DEBUG_BLOCK_READY);
  AppDebug_ClearFault(&g_app_debug, APP_DEBUG_FAULT_MOTOR_CONTROL);
}

static void APP_DRV8301_Init(void)
{
  Drv8301Config cfg;

  cfg.hspi = &hspi1;
  cfg.cs_port = DRV8301_CS_GPIO_Port;
  cfg.cs_pin = DRV8301_CS_Pin;
  cfg.enable_port = DRV8301_EN_GATE_GPIO_Port;
  cfg.enable_pin = DRV8301_EN_GATE_Pin;
  cfg.nfault_port = DRV8301_nFAULT_GPIO_Port;
  cfg.nfault_pin = DRV8301_nFAULT_Pin;

  if (Drv8301_Init(&g_drv8301, &cfg) != HAL_OK)
  {
    APP_Fault(APP_DEBUG_FAULT_DRIVER_INIT);
  }

  if (Drv8301_EnableGate(&g_drv8301, 1u) != HAL_OK)
  {
    APP_Fault(APP_DEBUG_FAULT_DRIVER_INIT);
  }

  if (Drv8301_ReadAll(&g_drv8301) != HAL_OK)
  {
    AppDebug_SetDriverState(&g_app_debug, APP_DEBUG_BLOCK_FAULT);
    AppDebug_SetFault(&g_app_debug, APP_DEBUG_FAULT_DRIVER_INIT);
    return;
  }

  if (Drv8301_VerifyControl1(&g_drv8301) != HAL_OK)
  {
    AppDebug_SetDriverState(&g_app_debug, APP_DEBUG_BLOCK_FAULT);
    AppDebug_SetFault(&g_app_debug, APP_DEBUG_FAULT_DRIVER_SPI_VERIFY_CTRL1);
    return;
  }

  if (Drv8301_VerifyControl2(&g_drv8301) != HAL_OK)
  {
    AppDebug_SetDriverState(&g_app_debug, APP_DEBUG_BLOCK_FAULT);
    AppDebug_SetFault(&g_app_debug, APP_DEBUG_FAULT_DRIVER_SPI_VERIFY);
    return;
  }

  AppDebug_SetDriverState(&g_app_debug, APP_DEBUG_BLOCK_READY);
}

static void APP_DRV8301_Poll(void)
{
  static uint32_t last_poll_ms = 0u;
  uint32_t now_ms = HAL_GetTick();

  if ((now_ms - last_poll_ms) < 100u)
  {
    return;
  }

  last_poll_ms = now_ms;

  if (Drv8301_ReadAll(&g_drv8301) != HAL_OK)
  {
    AppDebug_SetDriverState(&g_app_debug, APP_DEBUG_BLOCK_FAULT);
    AppDebug_SetFault(&g_app_debug, APP_DEBUG_FAULT_DRIVER_INIT);
    return;
  }

  if (Drv8301_VerifyControl1(&g_drv8301) != HAL_OK)
  {
    if (g_drv8301.verify_control1_mismatch_count >= 3u)
    {
      AppDebug_SetDriverState(&g_app_debug, APP_DEBUG_BLOCK_FAULT);
      AppDebug_SetFault(&g_app_debug, APP_DEBUG_FAULT_DRIVER_SPI_VERIFY_CTRL1);
    }
    return;
  }

  if (Drv8301_VerifyControl2(&g_drv8301) != HAL_OK)
  {
    if (g_drv8301.verify_mismatch_count >= 3u)
    {
      AppDebug_SetDriverState(&g_app_debug, APP_DEBUG_BLOCK_FAULT);
      AppDebug_SetFault(&g_app_debug, APP_DEBUG_FAULT_DRIVER_SPI_VERIFY);
    }
    return;
  }

  if (Drv8301_ReadFaultPin(&g_drv8301) == GPIO_PIN_RESET)
  {
    AppDebug_SetDriverState(&g_app_debug, APP_DEBUG_BLOCK_FAULT);
    AppDebug_SetFault(&g_app_debug, APP_DEBUG_FAULT_DRIVER_INIT);
    return;
  }

  AppDebug_SetDriverState(&g_app_debug, APP_DEBUG_BLOCK_READY);
  AppDebug_ClearFault(&g_app_debug, APP_DEBUG_FAULT_DRIVER_INIT);
  AppDebug_ClearFault(&g_app_debug, APP_DEBUG_FAULT_DRIVER_SPI_VERIFY);
  AppDebug_ClearFault(&g_app_debug, APP_DEBUG_FAULT_DRIVER_SPI_VERIFY_CTRL1);
}

static void APP_DRV8301_SendDebugFrame(void)
{
  static uint32_t last_debug_tx_ms = 0u;
  uint32_t now_ms = HAL_GetTick();
  const MotorNodeCanIds *ids;
  uint8_t payload[8] = {0};
  GPIO_PinState nfault_pin;

  if ((now_ms - last_debug_tx_ms) < 200u)
  {
    return;
  }

  ids = MotorNodeCan_GetIds(&g_motor_node);
  if (ids == 0)
  {
    return;
  }

  last_debug_tx_ms = now_ms;
  nfault_pin = Drv8301_ReadFaultPin(&g_drv8301);

  payload[0] = (uint8_t)(g_drv8301.status1 & 0x00FFu);
  payload[1] = (uint8_t)(g_drv8301.status2 & 0x00FFu);
  payload[2] = (uint8_t)(g_drv8301.control1 & 0x00FFu);
  payload[3] = (uint8_t)(g_drv8301.control2 & 0x00FFu);
  payload[4] = 0u;
  if (g_drv8301.verify_control1_ok != 0u)
  {
    payload[4] |= 0x01u;
  }
  if (g_drv8301.verify_ok != 0u)
  {
    payload[4] |= 0x02u;
  }
  if (g_drv8301.spi_ok != 0u)
  {
    payload[4] |= 0x04u;
  }
  if (g_drv8301.gate_enabled != 0u)
  {
    payload[4] |= 0x08u;
  }
  if (nfault_pin == GPIO_PIN_RESET)
  {
    payload[4] |= 0x10u;
  }
  payload[5] = (uint8_t)g_app_debug.fault;
  payload[6] = (uint8_t)g_app_debug.stage;
  payload[7] = (uint8_t)g_app_debug.driver;

  if (APP_FDCAN_SendStdFrame(ids->node_cfg_ack_id, payload) != HAL_OK)
  {
    AppDebug_SetFault(&g_app_debug, APP_DEBUG_FAULT_CAN_TX);
    return;
  }

  AppDebug_OnCanTx(&g_app_debug, now_ms);
}

static void APP_Debug_Init(void)
{
  AppDebug_Init(&g_app_debug);
  AppDebug_SetSensorState(&g_app_debug, APP_DEBUG_BLOCK_OFF);
  AppDebug_SetDriverState(&g_app_debug, APP_DEBUG_BLOCK_OFF);
  AppDebug_SetControlState(&g_app_debug, APP_DEBUG_BLOCK_INIT);
}

static void APP_Debug_LogPeriodic(void)
{
  static uint32_t last_log_ms = 0u;
  char line[128];
  int len;
  uint32_t now_ms = HAL_GetTick();

  if ((now_ms - last_log_ms) < 1000u)
  {
    return;
  }

  last_log_ms = now_ms;
  len = snprintf(line,
                 sizeof(line),
                 "t=%lu rpm=%u pos=%u raw=%u flags=0x%02X sensor=%u stg=%u fault=%u tx=%lu rx=%lu\r\n",
                 (unsigned long)now_ms,
                 (unsigned int)g_motor_node.status_fast.actual_rpm,
                 (unsigned int)g_motor_node.status_fast.actual_pos,
                 (unsigned int)g_as5048a_last_angle_raw,
                 (unsigned int)g_motor_node.status_fast.status_flags,
                 (unsigned int)g_as5048a_sensor_ok,
                 (unsigned int)g_app_debug.stage,
                 (unsigned int)g_app_debug.fault,
                 (unsigned long)g_app_debug.can_tx_count,
                 (unsigned long)g_app_debug.can_rx_count);

  if (len > 0)
  {
    (void)len;
  }
}

static void APP_FDCAN_Start(void)
{
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    APP_Fault(APP_DEBUG_FAULT_FDCAN_START);
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    APP_Fault(APP_DEBUG_FAULT_FDCAN_START);
  }

  AppDebug_SetStage(&g_app_debug, APP_DEBUG_STAGE_FDCAN_START_OK);
}

static void APP_MotorNode_Init(void)
{
  MotorNodeCanConfig cfg;

  cfg.hfdcan = &hfdcan1;
  cfg.node_id = 1u;
  cfg.section_index = 0u;
  cfg.profile = MOTOR_NODE_CAN_PROFILE_S0;
  cfg.status_period_ms = 100u;
  cfg.presence_period_ms = 250u;
  cfg.diag_period_ms = 500u;

  MotorNodeCan_Init(&g_motor_node, &cfg);
  MotorNodeCan_SetMeasuredState(&g_motor_node, 1234u, 0x4000u, 0);
  MotorNodeCan_SetDiag(&g_motor_node, 24.2f, 1.5f, 35u, 30u, 0u, 0u);
  MotorNodeCan_SetSeedData(&g_motor_node, 0u, 0u, 0u, 0u, 0u, 100u, 0u);
  AppDebug_SetStage(&g_app_debug, APP_DEBUG_STAGE_NODE_INIT_OK);
}

static void APP_MotorNode_PollRx(void)
{
  FDCAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0u)
  {
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
      AppDebug_SetFault(&g_app_debug, APP_DEBUG_FAULT_CAN_RX);
      break;
    }

    MotorNodeCan_OnRxMessage(&g_motor_node, &rx_header, rx_data);
    AppDebug_OnCanRx(&g_app_debug, HAL_GetTick());
  }
}

static void APP_MotorNode_Update(void)
{
  uint32_t now_ms = HAL_GetTick();
  uint16_t pos = AppMotorControl_GetElectricalPosQ16(&g_motor_control);
  uint16_t measured_rpm = AppMotorControl_GetAppliedRpm(&g_motor_control);
  uint8_t status_flags = 0u;
  uint8_t fault_flags = 0u;
  uint8_t warning_flags = 0u;

  g_as5048a_last_angle_raw = AppHall_GetRawState(&g_hall);
  g_as5048a_sensor_ok = AppHall_IsReady(&g_hall);

  if (g_as5048a_sensor_ok != 0u)
  {
    measured_rpm = AppHall_GetRpm(&g_hall);
    pos = AppHall_GetPosQ16(&g_hall);
  }

  if (MotorNodeCan_IsRunAllowed(&g_motor_node))
  {
    status_flags |= 0x01u;
  }

  if (MotorNodeCan_IsSectionActive(&g_motor_node))
  {
    status_flags |= 0x02u;
  }

  status_flags |= AppDebug_GetStatusFlags(&g_app_debug);
  g_motor_node.status_fast.status_flags = status_flags;
  g_motor_node.status_fast.error_code = AppDebug_GetErrorCode(&g_app_debug);
  MotorNodeCan_SetMeasuredState(&g_motor_node, measured_rpm, pos, 0);
  fault_flags = (uint8_t)(g_drv8301.status1 & 0xFFu);
  warning_flags = AppDebug_GetWarningFlags(&g_app_debug);
  g_motor_node.diag.fault_flags = fault_flags;
  g_motor_node.diag.warning_flags = warning_flags;
  MotorNodeCan_Process(&g_motor_node, now_ms);
  AppDebug_OnCanTx(&g_app_debug, now_ms);
}

static void APP_Heartbeat_Update(void)
{
  static uint32_t last_toggle_ms = 0u;
  static GPIO_PinState led_state = GPIO_PIN_RESET;
  uint32_t now_ms = HAL_GetTick();
  uint32_t period_ms = (g_app_debug.fault == APP_DEBUG_FAULT_NONE) ? 250u : 80u;

  if ((now_ms - last_toggle_ms) >= period_ms)
  {
    last_toggle_ms = now_ms;
    led_state = (led_state == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin, led_state);
  }
}

static void APP_Fault(AppDebugFault fault)
{
  AppDebug_SetFault(&g_app_debug, fault);
  Error_Handler();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  APP_Debug_Init();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  AppDebug_SetStage(&g_app_debug, APP_DEBUG_STAGE_CLOCK_OK);

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  AppDebug_SetStage(&g_app_debug, APP_DEBUG_STAGE_GPIO_OK);
  APP_Hall_Init();
  APP_SPI1_Init();
  APP_MotorPwm_Init();
  APP_MotorControl_Init();
  MX_FDCAN1_Init();
  AppDebug_SetStage(&g_app_debug, APP_DEBUG_STAGE_FDCAN_INIT_OK);
  /* USER CODE BEGIN 2 */
  APP_DRV8301_Init();
  APP_FDCAN_Start();
  APP_MotorNode_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    AppDebug_OnMainLoop(&g_app_debug);
    APP_MotorNode_PollRx();
    APP_Hall_Poll();
    APP_MotorControl_Poll();
    APP_DRV8301_Poll();
    APP_DRV8301_SendDebugFrame();
    APP_MotorNode_Update();
    APP_Debug_LogPeriodic();
    APP_Heartbeat_Update();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    APP_Fault(APP_DEBUG_FAULT_FDCAN_INIT);
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(DRV8301_CS_GPIO_Port, DRV8301_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DRV8301_EN_GATE_GPIO_Port, DRV8301_EN_GATE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = DRV8301_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DRV8301_CS_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = DRV8301_EN_GATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DRV8301_EN_GATE_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = DRV8301_nFAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DRV8301_nFAULT_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = HALL_A_Pin | HALL_B_Pin | HALL_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED_HEARTBEAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_HEARTBEAT_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitStruct.Pin = LED_HEARTBEAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_HEARTBEAT_GPIO_Port, &GPIO_InitStruct);
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin);
    for (volatile uint32_t i = 0u; i < 120000u; ++i)
    {
    }
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
