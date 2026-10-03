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
// Bus state from the ESR register (Live Expressions shows the enum name)
typedef enum {
  CAN_STATE_OK = 0,         // error active, bus healthy
  CAN_STATE_WARNING,        // TEC or REC >= 96
  CAN_STATE_PASSIVE,        // TEC or REC >= 128
  CAN_STATE_BUS_OFF,        // TEC > 255, node stopped sending, hardware recovering
  CAN_STATE_NOT_STARTED     // controller not running (start failed), CAN_Service restarts it
} CAN_BusState;

// Protocol error, same values as the ESR.LEC field
typedef enum {
  CAN_ERR_NONE = 0,
  CAN_ERR_STUFF,            // 6 equal bits in a row
  CAN_ERR_FORM,             // fixed-format bit field had a wrong value
  CAN_ERR_ACK,              // our frame was not acknowledged by any node
  CAN_ERR_BIT_RECESSIVE,    // we sent recessive (1) but read dominant (0)
  CAN_ERR_BIT_DOMINANT,     // we sent dominant (0) but read recessive (1)
  CAN_ERR_CRC               // received frame with a wrong CRC
} CAN_ErrorCode;

// HAL_CAN_ERROR_* mask with one named bit per flag (same bit order as the HAL defines),
// so Live Expressions lists every error by name
typedef union {
  uint32_t raw;                 // HAL_CAN_ERROR_* value
  struct {
    uint32_t ewg : 1;             // error warning: TEC or REC >= 96
    uint32_t epv : 1;             // error passive: TEC or REC >= 128
    uint32_t bof : 1;             // bus-off: TEC > 255
    uint32_t stuff : 1;           // stuff error
    uint32_t form : 1;            // form error
    uint32_t ack : 1;             // no ACK for our frame
    uint32_t bit_recessive : 1;   // sent 1, read 0
    uint32_t bit_dominant : 1;    // sent 0, read 1
    uint32_t crc : 1;             // CRC error
    uint32_t rx_fifo0_overrun : 1;
    uint32_t rx_fifo1_overrun : 1;
    uint32_t tx_arb_lost_mb0 : 1; // mailbox 0 lost arbitration (higher priority traffic)
    uint32_t tx_error_mb0 : 1;    // mailbox 0 transmit error
    uint32_t tx_arb_lost_mb1 : 1;
    uint32_t tx_error_mb1 : 1;
    uint32_t tx_arb_lost_mb2 : 1;
    uint32_t tx_error_mb2 : 1;
    uint32_t timeout : 1;         // HAL start/stop/init timed out
    uint32_t not_initialized : 1;
    uint32_t not_ready : 1;
    uint32_t not_started : 1;
    uint32_t param : 1;           // HAL_CAN_AddTxMessage: no free TX mailbox
    uint32_t invalid_callback : 1;
    uint32_t internal : 1;
  } bit;
} CAN_HalError;

// CAN bus health, one per bus (watch it in the debugger Live Expressions)
typedef struct {
  CAN_BusState state;           // current state
  const char *state_text;       // what the current state means
  CAN_ErrorCode last_error;     // most recent protocol error, kept until a newer one
  const char *last_error_text;  // what that error means and its usual causes
  uint32_t last_error_ms;       // HAL_GetTick() when last_error was seen
  uint8_t tec;                  // transmit error counter: our own frames failing
  uint8_t rec;                  // receive error counter: frames from other nodes failing
  uint8_t fault;                // 1 while state is not OK or a new error was seen in the last check
  uint32_t fault_count;         // times the bus went from healthy to fault
  CAN_HalError error_last;      // HAL flags of the most recent check that had any, kept until the next one
  CAN_HalError error_seen;      // every HAL flag seen since boot (write 0 to .raw in Live Expressions to clear)
  struct {                      // errors per type (ESR sampled every 10 ms, so these are "at least")
    uint32_t stuff;
    uint32_t form;
    uint32_t ack;
    uint32_t bit_recessive;
    uint32_t bit_dominant;
    uint32_t crc;
    uint32_t bus_off;           // times it entered bus-off
    uint32_t tx_arb_lost;       // frame gave up (replaced by a newer value) after losing arbitration: bus busy with lower IDs
    uint32_t tx_error;          // frame gave up (replaced by a newer value) after an error on the wire
  } errors;
  uint32_t tx_queued;           // frames handed to a TX mailbox
  uint32_t tx_dropped;          // frames that could not be queued
  uint32_t tx_aborted;          // older pending value replaced by a newer one (frame was slow to go out)
  uint32_t restarts;            // controller restarts done by CAN_Service
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

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

