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
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
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

// Filters, start and notifications. Used at boot and after every restart.
HAL_StatusTypeDef CAN_Config(CAN_HandleTypeDef *hcan)
{
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

// Call every 10 ms for each bus. Every 100 ms it reads the bus state, reports errors on
// UART (only when they change) and restarts the controller if it is not running.
// Bus-off itself is recovered by hardware (AutoBusOff = ENABLE).
void CAN_Service(CAN_HandleTypeDef *hcan)
{
  static uint32_t last_can1_check = 0;
  static uint32_t last_can2_check = 0;

  uint32_t now = HAL_GetTick();
  uint32_t *last_check;
  int bus;

  if (hcan == &hcan1) {
    last_check = &last_can1_check;
    bus = 1;
  } else if (hcan == &hcan2) {
    last_check = &last_can2_check;
    bus = 2;
  } else {
    return;
  }

  // Only check every 100 ms
  if ((now - *last_check) < 100) {
    return;
  }
  *last_check = now;

  CAN_BusStatus *status = CAN_GetStatus(hcan);
  uint32_t esr = hcan->Instance->ESR;
  uint32_t error = HAL_CAN_GetError(hcan);

  // Error interrupts are not enabled, so take the bus state straight from ESR
  if (esr & CAN_ESR_EWGF) {
    error |= HAL_CAN_ERROR_EWG;
  }
  if (esr & CAN_ESR_EPVF) {
    error |= HAL_CAN_ERROR_EPV;
  }
  if (esr & CAN_ESR_BOFF) {
    error |= HAL_CAN_ERROR_BOF;
  }
  switch ((esr & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos) {
    case 1: error |= HAL_CAN_ERROR_STF; break;
    case 2: error |= HAL_CAN_ERROR_FOR; break;
    case 3: error |= HAL_CAN_ERROR_ACK; break;
    case 4: error |= HAL_CAN_ERROR_BR;  break;
    case 5: error |= HAL_CAN_ERROR_BD;  break;
    case 6: error |= HAL_CAN_ERROR_CRC; break;
    default: break;
  }
  // Clear the last error code so the next check only sees new errors
  CLEAR_BIT(hcan->Instance->ESR, CAN_ESR_LEC);

  status->tec = (uint8_t)((esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos);
  status->rec = (uint8_t)((esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos);

  if (error == HAL_CAN_ERROR_NONE) {
    if (status->fault) {
      printf("CAN%d OK\r\n", bus);
    }
    status->fault = 0;
    status->last_error = HAL_CAN_ERROR_NONE;
    return;
  }

  // Report only when the error changes, so the UART is not flooded
  if (!status->fault || (error != status->last_error)) {
    printf("CAN%d TEC=%u REC=%u ", bus, status->tec, status->rec);
    CAN_PrintHalError(error);
  }
  status->fault = 1;
  status->last_error = error;

  // Controller not running (start timeout, stopped, error state): restart it in place.
  // If it fails again, the next check sees the new error code and retries in 100 ms.
  if (HAL_CAN_GetState(hcan) != HAL_CAN_STATE_LISTENING) {
    status->restarts++;
    CAN_Restart(hcan);
    return;
  }

  HAL_CAN_ResetError(hcan);
}

void CAN_PrintHalError(uint32_t error)
{
  printf("HAL_CAN error = 0x%08lX\r\n", error);

  if (error & HAL_CAN_ERROR_EWG) {
    printf(" - HAL_CAN_ERROR_EWG: error warning\r\n");
  }
  if (error & HAL_CAN_ERROR_EPV) {
    printf(" - HAL_CAN_ERROR_EPV: error passive\r\n");
  }
  if (error & HAL_CAN_ERROR_BOF) {
    printf(" - HAL_CAN_ERROR_BOF: bus off\r\n");
  }
  if (error & HAL_CAN_ERROR_STF) {
    printf(" - HAL_CAN_ERROR_STF: stuff error\r\n");
  }
  if (error & HAL_CAN_ERROR_FOR) {
    printf(" - HAL_CAN_ERROR_FOR: form error\r\n");
  }
  if (error & HAL_CAN_ERROR_ACK) {
    printf(" - HAL_CAN_ERROR_ACK: no ACK received\r\n");
  }
  if (error & HAL_CAN_ERROR_BR) {
    printf(" - HAL_CAN_ERROR_BR: bit recessive error\r\n");
  }
  if (error & HAL_CAN_ERROR_BD) {
    printf(" - HAL_CAN_ERROR_BD: bit dominant error\r\n");
  }
  if (error & HAL_CAN_ERROR_CRC) {
    printf(" - HAL_CAN_ERROR_CRC: CRC error\r\n");
  }
  if (error & HAL_CAN_ERROR_RX_FOV0) {
    printf(" - HAL_CAN_ERROR_RX_FOV0: RX FIFO0 overrun\r\n");
  }
  if (error & HAL_CAN_ERROR_RX_FOV1) {
    printf(" - HAL_CAN_ERROR_RX_FOV1: RX FIFO1 overrun\r\n");
  }
  if (error & HAL_CAN_ERROR_TX_ALST0) {
    printf(" - HAL_CAN_ERROR_TX_ALST0: arbitration lost mailbox 0\r\n");
  }
  if (error & HAL_CAN_ERROR_TX_TERR0) {
    printf(" - HAL_CAN_ERROR_TX_TERR0: transmit error mailbox 0\r\n");
  }
  if (error & HAL_CAN_ERROR_TX_ALST1) {
    printf(" - HAL_CAN_ERROR_TX_ALST1: arbitration lost mailbox 1\r\n");
  }
  if (error & HAL_CAN_ERROR_TX_TERR1) {
    printf(" - HAL_CAN_ERROR_TX_TERR1: transmit error mailbox 1\r\n");
  }
  if (error & HAL_CAN_ERROR_TX_ALST2) {
    printf(" - HAL_CAN_ERROR_TX_ALST2: arbitration lost mailbox 2\r\n");
  }
  if (error & HAL_CAN_ERROR_TX_TERR2) {
    printf(" - HAL_CAN_ERROR_TX_TERR2: transmit error mailbox 2\r\n");
  }
  if (error & HAL_CAN_ERROR_TIMEOUT) {
    printf(" - HAL_CAN_ERROR_TIMEOUT\r\n");
  }
  if (error & HAL_CAN_ERROR_NOT_INITIALIZED) {
    printf(" - HAL_CAN_ERROR_NOT_INITIALIZED\r\n");
  }
  if (error & HAL_CAN_ERROR_NOT_READY) {
    printf(" - HAL_CAN_ERROR_NOT_READY\r\n");
  }
  if (error & HAL_CAN_ERROR_NOT_STARTED) {
    printf(" - HAL_CAN_ERROR_NOT_STARTED\r\n");
  }
  if (error & HAL_CAN_ERROR_PARAM) {
    printf(" - HAL_CAN_ERROR_PARAM: no free TX mailbox\r\n");
  }
}
/* USER CODE END 1 */
