/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>

void CAN_FilterConfig1(void);
void CAN_FilterConfig2(void);
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  // Not fatal if the bus is not ready yet (e.g. transceiver unpowered): CAN_Service retries
  CAN_Config(&hcan1);
  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 6;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  // Not fatal if the bus is not ready yet (e.g. transceiver unpowered): CAN_Service retries
  CAN_Config(&hcan2);
  /* USER CODE END CAN2_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }
    __HAL_RCC_CAN2_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
    HAL_NVIC_SetPriority(CAN2_SCE_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_SCE_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }
    __HAL_RCC_CAN2_CLK_DISABLE();

    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_SCE_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void CAN_FilterConfig1()
{
  CAN_FilterTypeDef canfilterconfig = {0};

  // Filter for 0x710 (exact match)
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 0;                  // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x710 << 5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x7FF << 5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 18;
  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);


}
void CAN_FilterConfig2()
{

  CAN_FilterTypeDef canfilterconfig = {0};

  // Filter for 0x702 - AMS status (exact match)
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 18;                 // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x702 << 5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x7FF << 5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  // HAL writes this into the shared CAN1->FMR even for CAN2, so it must match CAN_FilterConfig1
  canfilterconfig.SlaveStartFilterBank = 18;

  HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig);

}

CAN_BusStatus can1_status;
CAN_BusStatus can2_status;

static CAN_BusStatus *CAN_GetStatus(CAN_HandleTypeDef *hcan)
{
  return (hcan == &hcan1) ? &can1_status : &can2_status;
}

// Meaning and usual causes, shown in Live Expressions (state_text, last_error_text) and on UART
static const char *const CAN_StateText[] = {
  [CAN_STATE_OK]          = "OK - error active, bus healthy",
  [CAN_STATE_WARNING]     = "WARNING - TEC or REC >= 96: errors happening, still communicating",
  [CAN_STATE_PASSIVE]     = "PASSIVE - TEC or REC >= 128: many errors (REC high = receive side, TEC high = our TX side)",
  [CAN_STATE_BUS_OFF]     = "BUS-OFF - TEC > 255: not sending, hardware recovers after 128 x 11 recessive bits",
  [CAN_STATE_NOT_STARTED] = "NOT STARTED - controller off: start timed out (bus stuck dominant, transceiver unpowered), restarting",
};

static const char *const CAN_ErrorText[] = {
  [CAN_ERR_NONE]          = "none",
  [CAN_ERR_STUFF]         = "STUFF - 6 equal bits: node at another bitrate, noise, missing 120R termination",
  [CAN_ERR_FORM]          = "FORM - fixed bit field wrong: bitrate or sample point mismatch, noise",
  [CAN_ERR_ACK]           = "ACK - nobody acknowledged our frame: no other node powered/connected, CANH/CANL open or swapped, other bitrate",
  [CAN_ERR_BIT_RECESSIVE] = "BIT RECESSIVE - sent 1 read 0: bus held dominant or shorted, faulty node/transceiver",
  [CAN_ERR_BIT_DOMINANT]  = "BIT DOMINANT - sent 0 read 1: our TX not reaching the bus (transceiver off/standby, TX wiring)",
  [CAN_ERR_CRC]           = "CRC - corrupted frame: noise, bad termination or long stubs, bitrate mismatch",
};

#define CAN_RESTART_MS  100   // Min time between restarts, a failing start blocks up to 10 ms
#define CAN_PRINT_MS    1000  // UART report rate limit

// Per bus bookkeeping for CAN_Service
typedef struct {
  uint32_t last_restart_ms;
  uint32_t last_print_ms;
  CAN_BusState printed_state;
  CAN_ErrorCode printed_error;
} CAN_ServiceData;

static CAN_ServiceData can_service[2];

// Filters, start and notifications. Used at boot and after every restart.
HAL_StatusTypeDef CAN_Config(CAN_HandleTypeDef *hcan)
{
  CAN_BusStatus *status = CAN_GetStatus(hcan);

  // Readable texts from boot, before the first CAN_Service
  status->state_text = CAN_StateText[status->state];
  if (status->last_error_text == NULL) {
    status->last_error_text = CAN_ErrorText[CAN_ERR_NONE];
  }

  if (hcan == &hcan1) {
    CAN_FilterConfig1();
  } else {
    CAN_FilterConfig2();
  }

  if (HAL_CAN_Start(hcan) != HAL_OK) {
    return HAL_ERROR;
  }

  // AMS status (0x702) is received on CAN2
  if (hcan == &hcan2) {
    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}

// Re-initialises one controller in place.
// No HAL_CAN_DeInit: CAN1 MspDeInit gates the CAN1 clock, which CAN2 also runs on.
static HAL_StatusTypeDef CAN_Restart(CAN_HandleTypeDef *hcan)
{
  if (HAL_CAN_GetState(hcan) == HAL_CAN_STATE_LISTENING) {
    HAL_CAN_Stop(hcan);
  }

  if (HAL_CAN_Init(hcan) != HAL_OK) {
    return HAL_ERROR;
  }

  return CAN_Config(hcan);
}

// Sends only the latest value: any older copy of the same frame still waiting in a
// mailbox is aborted first, so nothing is buffered. Never blocks, never calls Error_Handler.
HAL_StatusTypeDef CAN_Send(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *header, uint8_t *data)
{
  static const uint32_t tme[3] = { CAN_TSR_TME0, CAN_TSR_TME1, CAN_TSR_TME2 };
  static const uint32_t mailboxes[3] = { CAN_TX_MAILBOX0, CAN_TX_MAILBOX1, CAN_TX_MAILBOX2 };

  CAN_BusStatus *status = CAN_GetStatus(hcan);
  uint32_t tsr = hcan->Instance->TSR;
  uint32_t mailbox;

  if (HAL_CAN_GetState(hcan) != HAL_CAN_STATE_LISTENING) {
    status->tx_dropped++;
    return HAL_ERROR;
  }

  // Abort pending mailboxes that hold an older value of this frame
  for (int i = 0; i < 3; i++) {
    uint32_t tir = hcan->Instance->sTxMailBox[i].TIR;
    uint32_t std_id = (tir & CAN_TI0R_STID) >> CAN_TI0R_STID_Pos;

    if (((tsr & tme[i]) == 0U) && ((tir & CAN_TI0R_IDE) == 0U) && (std_id == header->StdId)) {
      HAL_CAN_AbortTxRequest(hcan, mailboxes[i]);
      status->tx_aborted++;
    }
  }

  // A frame already on the wire cannot be aborted; it finishes and the new value uses another
  // mailbox. Only if all 3 are busy is this value dropped (next period sends a fresh one).
  if (HAL_CAN_AddTxMessage(hcan, header, data, &mailbox) != HAL_OK) {
    status->tx_dropped++;
    return HAL_ERROR;
  }

  status->tx_queued++;
  return HAL_OK;
}

// Call every 10 ms for each bus. Reads the bus state and every error flag into canX_status,
// reports changes on UART (max once per second) and restarts the controller if it is not
// running (max every 100 ms). Bus-off itself is recovered by hardware (AutoBusOff = ENABLE).
void CAN_Service(CAN_HandleTypeDef *hcan)
{
  int bus;

  if (hcan == &hcan1) {
    bus = 1;
  } else if (hcan == &hcan2) {
    bus = 2;
  } else {
    return;
  }

  CAN_TypeDef *can = hcan->Instance;
  CAN_BusStatus *status = CAN_GetStatus(hcan);
  CAN_ServiceData *svc = &can_service[bus - 1];
  CAN_BusState prev_state = status->state;
  uint32_t now = HAL_GetTick();
  uint32_t esr = can->ESR;
  uint32_t tsr = can->TSR;
  CAN_ErrorCode lec = (CAN_ErrorCode)((esr & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos);

  // Error interrupts are not enabled, so every flag is collected by polling the registers.
  // HAL error code first: start/init timeouts, no free TX mailbox (param)...
  CAN_HalError error = { .raw = HAL_CAN_GetError(hcan) };

  // Error counters and state flags
  status->tec = (uint8_t)((esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos);
  status->rec = (uint8_t)((esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos);
  if (esr & CAN_ESR_EWGF) {
    error.bit.ewg = 1;
  }
  if (esr & CAN_ESR_EPVF) {
    error.bit.epv = 1;
  }
  if (esr & CAN_ESR_BOFF) {
    error.bit.bof = 1;
  }

  // Last protocol error since the previous check, then clear it so the next check only sees new ones
  switch (lec) {
    case CAN_ERR_STUFF:         error.bit.stuff = 1;         status->errors.stuff++;         break;
    case CAN_ERR_FORM:          error.bit.form = 1;          status->errors.form++;          break;
    case CAN_ERR_ACK:           error.bit.ack = 1;           status->errors.ack++;           break;
    case CAN_ERR_BIT_RECESSIVE: error.bit.bit_recessive = 1; status->errors.bit_recessive++; break;
    case CAN_ERR_BIT_DOMINANT:  error.bit.bit_dominant = 1;  status->errors.bit_dominant++;  break;
    case CAN_ERR_CRC:           error.bit.crc = 1;           status->errors.crc++;           break;
    default: break;
  }
  if ((lec >= CAN_ERR_STUFF) && (lec <= CAN_ERR_CRC)) {
    status->last_error = lec;
    status->last_error_text = CAN_ErrorText[lec];
    status->last_error_ms = now;
  }
  CLEAR_BIT(can->ESR, CAN_ESR_LEC);

  // Why finished TX requests failed (a frame replaced by a newer value ends as aborted),
  // then clear the request-completed flags (write 1)
  if ((tsr & CAN_TSR_RQCP0) && (tsr & CAN_TSR_ALST0)) { error.bit.tx_arb_lost_mb0 = 1; status->errors.tx_arb_lost++; }
  if ((tsr & CAN_TSR_RQCP0) && (tsr & CAN_TSR_TERR0)) { error.bit.tx_error_mb0 = 1;    status->errors.tx_error++; }
  if ((tsr & CAN_TSR_RQCP1) && (tsr & CAN_TSR_ALST1)) { error.bit.tx_arb_lost_mb1 = 1; status->errors.tx_arb_lost++; }
  if ((tsr & CAN_TSR_RQCP1) && (tsr & CAN_TSR_TERR1)) { error.bit.tx_error_mb1 = 1;    status->errors.tx_error++; }
  if ((tsr & CAN_TSR_RQCP2) && (tsr & CAN_TSR_ALST2)) { error.bit.tx_arb_lost_mb2 = 1; status->errors.tx_arb_lost++; }
  if ((tsr & CAN_TSR_RQCP2) && (tsr & CAN_TSR_TERR2)) { error.bit.tx_error_mb2 = 1;    status->errors.tx_error++; }
  can->TSR = tsr & (CAN_TSR_RQCP0 | CAN_TSR_RQCP1 | CAN_TSR_RQCP2);

  // Bus state, worst first
  if (HAL_CAN_GetState(hcan) != HAL_CAN_STATE_LISTENING) {
    status->state = CAN_STATE_NOT_STARTED;
    error.bit.not_started = 1;
  } else if (esr & CAN_ESR_BOFF) {
    status->state = CAN_STATE_BUS_OFF;
  } else if (esr & CAN_ESR_EPVF) {
    status->state = CAN_STATE_PASSIVE;
  } else if (esr & CAN_ESR_EWGF) {
    status->state = CAN_STATE_WARNING;
  } else {
    status->state = CAN_STATE_OK;
  }
  status->state_text = CAN_StateText[status->state];
  if ((status->state == CAN_STATE_BUS_OFF) && (prev_state != CAN_STATE_BUS_OFF)) {
    status->errors.bus_off++;
  }

  // Error flags: last non-empty set and everything seen since boot
  if (error.raw != 0U) {
    status->error_last = error;
    status->error_seen.raw |= error.raw;
  }
  if ((error.raw != 0U) && !status->fault) {
    status->fault_count++;
  }
  status->fault = (error.raw != 0U) ? 1 : 0;

  // UART report when the state or the last error changes, max once per second
  if (((status->state != svc->printed_state) || (status->last_error != svc->printed_error)) &&
      ((now - svc->last_print_ms) >= CAN_PRINT_MS)) {
    svc->last_print_ms = now;
    svc->printed_state = status->state;
    svc->printed_error = status->last_error;
    printf("CAN%d %s | TEC=%u REC=%u | last error: %s\r\n",
           bus, status->state_text, status->tec, status->rec, status->last_error_text);
  }

  // Controller not running (start timeout, stopped, error state): restart it in place.
  // HAL_CAN_Init clears the HAL error code; if it fails again the next restart is in 100 ms.
  if (status->state == CAN_STATE_NOT_STARTED) {
    if ((now - svc->last_restart_ms) >= CAN_RESTART_MS) {
      svc->last_restart_ms = now;
      status->restarts++;
      CAN_Restart(hcan);
    }
    return;
  }

  HAL_CAN_ResetError(hcan);
}
/* USER CODE END 1 */
