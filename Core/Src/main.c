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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

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

/* USER CODE BEGIN PV */
volatile uint32_t time_ms = 0;

// Digital inputs
uint8_t inertia;    // PC8
uint8_t emergency;  // PC7

// Analog sensors
float ST_ANGLE;     // Steering angle (deg)
float SUSP1;        // Left suspension position (mm)
float SUSP2;        // Right suspension position (mm)
uint16_t adcSusp1, adcSusp2;
uint16_t adcSt_Angle;
uint16_t ADC_VALUE[3];  // DMA target: [0] PA7 steering, [1] PB0 susp1, [2] PB1 susp2

// Digital inputs
uint8_t ignition, r2d;  // PC9, PA8

// ADC moving average
#define ADC_BUFFER_SIZE 10
uint16_t adc_buffers[3][ADC_BUFFER_SIZE];
uint8_t adc_buffer_index = 0;
uint16_t adc_filtered[3];

// CAN
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];

CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

// AMS status received on CAN2 (ID 0x702, byte 1)
uint8_t AMS_Status;
volatile uint32_t ams_last_rx_ms = 0;  // time_ms of the last 0x702 frame

// AMS status is only trusted while 0x702 keeps arriving (set to >= 3x the AMS period)
#define AMS_TIMEOUT_MS 500

// Debug snapshot of the whole board: add "acq4" to STM32CubeIDE Live Expressions
typedef struct {
	uint32_t time_ms;           // uptime (ms)

	struct {
		uint16_t raw[3];        // DMA samples: [0] PA7 steering, [1] PB0 susp left, [2] PB1 susp right
		uint16_t filtered[3];   // moving average of ADC_BUFFER_SIZE samples
		float volts[3];         // filtered value at the MCU pin (V)
	} adc;

	struct {
		float steering_deg;
		float susp_left_mm;
		float susp_right_mm;
	} sensors;

	struct {
		uint8_t ignition;       // PC9
		uint8_t r2d;            // PA8
		uint8_t inertia;        // PC8, 1 = OK
		uint8_t emergency;      // PC7, 1 = OK
		uint8_t ts_on_led;      // PD2 output
	} io;

	struct {
		uint8_t status;         // 0x702 byte 1 (precharge_state), LED on at 0x10
		uint8_t alive;          // 1 while 0x702 arrives within AMS_TIMEOUT_MS
		uint32_t age_ms;        // time since the last 0x702
		uint32_t rx_count;      // 0x702 frames received
	} ams;

	struct {
		uint8_t tx_740[7];      // last payload queued on CAN1 (0x740)
		uint8_t tx_060[1];      // last payload queued on CAN2 (0x060)
		CAN_BusStatus can1;
		CAN_BusStatus can2;
	} can;
} Acq4_Debug;

Acq4_Debug acq4;

// Tasks Prototypes
void execute_immediate_tasks(void);
void execute_10ms_tasks(void);
void execute_50ms_tasks(void);
void execute_100ms_tasks(void);
void Acq4_DebugUpdate(void);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ADC_UpdateMovingAverage(void);
float MeasureSuspensionPosition(uint16_t bits);
float MeasureSteeringAngle(uint16_t bits);

int _write(int file, char *data, int len) {
	HAL_UART_Transmit(&huart1, (uint8_t*) data, len, HAL_MAX_DELAY);
	return len;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM7) {
		time_ms++;
	}
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  MX_IWDG_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_VALUE, 3);

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < ADC_BUFFER_SIZE; j++) {
			adc_buffers[i][j] = 0;
		}
		adc_filtered[i] = 0;
	}

	// CAN filters, start and RX notification are done in MX_CANx_Init (CAN_Config)

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		// Execute immediate tasks
		execute_immediate_tasks();

		static uint32_t previus_tick_10ms = 0;
		static uint32_t previus_tick_50ms = 0;
		static uint32_t previus_tick_100ms = 0;

		// Execute 10ms Tasks
		if (time_ms - previus_tick_10ms >= 10) {
			execute_10ms_tasks();
			previus_tick_10ms = time_ms;
		}

		// Execute 50ms Tasks
		if (time_ms - previus_tick_50ms >= 50) {
			execute_50ms_tasks();
			previus_tick_50ms = time_ms;
		}

		// Execute 100ms Tasks
		if (time_ms - previus_tick_100ms >= 100) {
			execute_100ms_tasks();
			previus_tick_100ms = time_ms;
		}
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void execute_immediate_tasks() {

}

void execute_10ms_tasks() {
	HAL_IWDG_Refresh(&hiwdg);

	// Latest filtered ADC samples
	adcSt_Angle = adc_filtered[0];
	adcSusp1 = adc_filtered[1];
	adcSusp2 = adc_filtered[2];

	// Digital inputs
	ignition = HAL_GPIO_ReadPin(IGN_GPIO_Port, IGN_Pin);
	r2d = HAL_GPIO_ReadPin(R2D_GPIO_Port, R2D_Pin);
	inertia = HAL_GPIO_ReadPin(INERTIA_GPIO_Port, INERTIA_Pin);
	emergency = HAL_GPIO_ReadPin(EMERGENCY_GPIO_Port, EMERGENCY_Pin);

	// CAN health check / recovery (runs every 100 ms internally)
	CAN_Service(&hcan1);
	CAN_Service(&hcan2);

	Acq4_DebugUpdate();
}

void execute_50ms_tasks() {
	ADC_UpdateMovingAverage();
	ST_ANGLE = MeasureSteeringAngle(adcSt_Angle);
	SUSP1 = MeasureSuspensionPosition(adcSusp1);
	SUSP2 = MeasureSuspensionPosition(adcSusp2);

	// Scale by 10 for 1-decimal precision (e.g. 12.3 -> 123)
	int16_t st_angle = (int16_t) (ST_ANGLE * 10.0f);
	uint16_t susp1 = (uint16_t) (SUSP1 * 10.0f);
	uint16_t susp2 = (uint16_t) (SUSP2 * 10.0f);

	// CAN1 - 0x740: steering angle, suspensions, inertia/emergency
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = 0x740;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 7;

	TxData[0] = st_angle & 0xFF;          // Steering angle LSB
	TxData[1] = (st_angle >> 8) & 0xFF;   // Steering angle MSB
	TxData[2] = susp1 & 0xFF;             // Suspension 1 LSB
	TxData[3] = (susp1 >> 8) & 0xFF;      // Suspension 1 MSB
	TxData[4] = susp2 & 0xFF;             // Suspension 2 LSB
	TxData[5] = (susp2 >> 8) & 0xFF;      // Suspension 2 MSB
	TxData[6] = (inertia & 0x01) | ((emergency << 1) & 0x02);

	// A failed send is counted in can1_status and handled by CAN_Service, never fatal
	memcpy(acq4.can.tx_740, TxData, sizeof(acq4.can.tx_740));
	CAN_Send(&hcan1, &TxHeader, TxData);

	// CAN2 - 0x060: ignition / R2D
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = 0x60;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 1;

	TxData[0] = (ignition & 0x01) | ((r2d << 1) & 0x02);
	memcpy(acq4.can.tx_060, TxData, sizeof(acq4.can.tx_060));
	CAN_Send(&hcan2, &TxHeader, TxData);
}

void execute_100ms_tasks() {
	HAL_GPIO_TogglePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin); // HEARTBEAT

	printf("O Steering Angle é %.2f\n", ST_ANGLE);
	printf("A suspensão da esquerda é %.2f\n", SUSP1);
	printf("A suspensão da direita é %.2f\n", SUSP2);
	printf("O Ignition está a %d\n", ignition);
	printf("O R2D está a %d\n", r2d);
	if (inertia == 1) {
		printf("O Inertia está bom\n");
	} else {
		printf("O Inertia está ativado\n");
	}
	if (emergency == 1) {
		printf("O Emergency button está bom\n");
	} else {
		printf("O Emergency button está ativado\n");
	}

	// TS ON LED driven by AMS status, forced off if the AMS frame stopped arriving
	uint8_t ams_alive = (time_ms - ams_last_rx_ms) <= AMS_TIMEOUT_MS;
	if (ams_alive && (AMS_Status == 0x10)) {
		HAL_GPIO_WritePin(TS_ON_LED_GPIO_Port, TS_ON_LED_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(TS_ON_LED_GPIO_Port, TS_ON_LED_Pin, GPIO_PIN_RESET);
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
		return;
	}

	if ((RxHeader.StdId == 0x702) && (RxHeader.IDE == CAN_ID_STD)) {
		AMS_Status = RxData[1];
		ams_last_rx_ms = time_ms;
		acq4.ams.rx_count++;
	}
}

// Copies everything into acq4 for Live Expressions (called every 10 ms)
void Acq4_DebugUpdate(void) {
	uint32_t now = time_ms;

	acq4.time_ms = now;

	for (int i = 0; i < 3; i++) {
		acq4.adc.raw[i] = ADC_VALUE[i];
		acq4.adc.filtered[i] = adc_filtered[i];
		acq4.adc.volts[i] = adc_filtered[i] * 3.3f / 4095.0f;
	}

	acq4.sensors.steering_deg = ST_ANGLE;
	acq4.sensors.susp_left_mm = SUSP1;
	acq4.sensors.susp_right_mm = SUSP2;

	acq4.io.ignition = ignition;
	acq4.io.r2d = r2d;
	acq4.io.inertia = inertia;
	acq4.io.emergency = emergency;
	acq4.io.ts_on_led = HAL_GPIO_ReadPin(TS_ON_LED_GPIO_Port, TS_ON_LED_Pin);

	acq4.ams.status = AMS_Status;
	acq4.ams.age_ms = now - ams_last_rx_ms;
	acq4.ams.alive = acq4.ams.age_ms <= AMS_TIMEOUT_MS;

	acq4.can.can1 = can1_status;
	acq4.can.can2 = can2_status;
}

// Measurements

// Moving average over the last ADC_BUFFER_SIZE samples of each ADC channel
void ADC_UpdateMovingAverage(void) {
	for (int channel = 0; channel < 3; channel++) {
		adc_buffers[channel][adc_buffer_index] = ADC_VALUE[channel];

		uint32_t sum = 0;
		for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
			sum += adc_buffers[channel][i];
		}

		adc_filtered[channel] = sum / ADC_BUFFER_SIZE;
	}

	adc_buffer_index = (adc_buffer_index + 1) % ADC_BUFFER_SIZE;
}

// Converts ADC counts to suspension position (mm) - 75 mm linear potentiometer
float MeasureSuspensionPosition(uint16_t bits) {
	float V_SUSP;
	float sensor_voltage = 5.0f;
	float MCU_voltage = 3.3f;
	float Electrical_stroke = 75.0f;
	float SUSPENSION_POSITION;
	float Conversion_Factor = MCU_voltage / sensor_voltage;
	float volts;

	V_SUSP = (bits * MCU_voltage) / 4095.0f;
	volts = V_SUSP / Conversion_Factor;

	SUSPENSION_POSITION = (Electrical_stroke * volts) / sensor_voltage;

	return SUSPENSION_POSITION;
}

// Converts ADC counts to steering angle (deg) - 0.5..4.5 V sensor behind a 2/3 divider
float MeasureSteeringAngle(uint16_t bits) {
	const float ADC_MAX = 4095.0f;
	const float MCU_VREF = 3.3f;
	const float SENSOR_VREF_Max = 4.5f;
	const float SENSOR_VREF_Min = 0.5f;
	const float Resolution = 180.0f;
	const float OFFSET = -73.8f;

	// Sensor range as seen by the MCU (after the 2/3 divider)
	float max_v = SENSOR_VREF_Max * (2.0f / 3.0f);
	float min_v = SENSOR_VREF_Min * (2.0f / 3.0f);
	float inclination = 360.0f / (max_v - min_v);

	float V_STA = (bits * MCU_VREF) / ADC_MAX;
	float ST_Angle = inclination * V_STA - 45.0f - Resolution;

	// Mechanical zero calibration
	ST_Angle = ST_Angle + OFFSET;
	return ST_Angle;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
