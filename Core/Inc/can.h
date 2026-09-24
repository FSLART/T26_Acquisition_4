/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */
// CAN bus health, one per bus (watch it in the debugger Live Expressions)
typedef struct {
  uint32_t last_error;  // HAL_CAN_ERROR_* bits seen in the last check
  uint32_t tx_queued;   // frames handed to a TX mailbox
  uint32_t tx_dropped;  // frames that could not be queued
  uint32_t tx_aborted;  // stale frames aborted to free the mailboxes
  uint32_t restarts;    // controller restarts done by CAN_Service
  uint8_t tec;          // transmit error counter
  uint8_t rec;          // receive error counter
  uint8_t fault;        // 1 while the bus reports an error
} CAN_BusStatus;

extern CAN_BusStatus can1_status;
extern CAN_BusStatus can2_status;
/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */
HAL_StatusTypeDef CAN_Config(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef CAN_Send(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *header, uint8_t *data);
void CAN_Service(CAN_HandleTypeDef *hcan);
void CAN_PrintHalError(uint32_t error);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

