/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "status.h"
#include "string.h"
#include "usbd_cdc_if.h"
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
const uint16_t led_pins[3] = { LED_GREEN_Pin, LED_RED_Pin, LED_YELLOW_Pin };
uint8_t led_pin_count;
volatile uint32_t led_tick;

TDK_Status current_status, prev_status;
uint8_t connected_flag, connected_flag_prev;

char rxBuf[RX_BUFFER_SIZE];
uint32_t rxLen;
uint32_t last_rx_time;

uint16_t beep_buffer[10];
uint8_t beep_index;
uint8_t beep_length;
volatile uint32_t beep_tick;

volatile uint32_t btn_green_tick;
volatile uint32_t btn_yellow_tick;
uint8_t btn_green_triggered_flag;
uint8_t btn_yellow_triggered_flag;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void ShortBeep(int count);
void LongBeep(int count);
void ShortLongBeep(int short_count, int long_count);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_TIM1_Init();
	MX_USB_DEVICE_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);

	current_status = IDLE;
	prev_status = IDLE;
	connected_flag = 0;
	connected_flag_prev = 0;
	led_pin_count = 0;
	rxLen = 0;
	last_rx_time = 0;
	beep_index = 0;
	beep_length = 0;
	beep_tick = 0;
	btn_green_tick = 0;
	btn_yellow_tick = 0;
	btn_green_triggered_flag = 0;
	btn_yellow_triggered_flag = 0;

	ShortBeep(3);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		uint32_t now = HAL_GetTick();

		if (rxLen > 0 && rxBuf[rxLen - 1] == '\n') {
			rxBuf[rxLen - 1] = '\0';
			connected_flag = 1;
			last_rx_time = now;
			if (strcmp("idle", rxBuf) == 0) {
				current_status = IDLE;
			} else if (strcmp("running A", rxBuf) == 0) {
				current_status = RUNNING_A;
			} else if (strcmp("running B", rxBuf) == 0) {
				current_status = RUNNING_B;
			} else if (strcmp("stopped", rxBuf) == 0) {
				current_status = STOPPED;
			} else if (strcmp("error", rxBuf) == 0) {
				current_status = ERR;
			} else {
				last_rx_time = 0;
			}
			rxLen = 0;
		}

		connected_flag =
				(now - last_rx_time) > DISCONNECTED_TIMEOUT ?
						0 : connected_flag;
		if (connected_flag && !connected_flag_prev)
			ShortBeep(2);
		if (!connected_flag && connected_flag_prev)
			LongBeep(3);
		connected_flag_prev = connected_flag;

		if (HAL_GPIO_ReadPin(GPIOB, BTN_GREEN_Pin)) {
			btn_green_tick = 0;
			btn_green_triggered_flag = 0;
		}
		if (HAL_GPIO_ReadPin(GPIOB, BTN_YELLOW_Pin)) {
			btn_yellow_tick = 0;
			btn_yellow_triggered_flag = 0;
		}

		if (!HAL_GPIO_ReadPin(GPIOB, BTN_GREEN_Pin)
				&& !HAL_GPIO_ReadPin(GPIOB, BTN_YELLOW_Pin)
				&& btn_green_tick > 100 && btn_yellow_tick > 100
				&& !btn_green_triggered_flag && !btn_yellow_triggered_flag) {
			btn_green_triggered_flag = 1;
			btn_yellow_triggered_flag = 1;
			LongBeep(2);
			usb_printf("stop\n");
		} else if (!HAL_GPIO_ReadPin(GPIOB, BTN_GREEN_Pin)
				&& btn_green_tick > 500 && !btn_green_triggered_flag) {
			btn_green_triggered_flag = 1;
			ShortLongBeep(1, 1);
			usb_printf("start A\n");
		} else if (!HAL_GPIO_ReadPin(GPIOB, BTN_YELLOW_Pin)
				&& btn_yellow_tick > 500 && !btn_yellow_triggered_flag) {
			btn_yellow_triggered_flag = 1;
			ShortLongBeep(2, 1);
			usb_printf("start B\n");
		}
//
//		HAL_GPIO_WritePin(GPIOB, LED_YELLOW_Pin,
//				!HAL_GPIO_ReadPin(GPIOB, BTN_YELLOW_Pin));
//		HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin,
//				!HAL_GPIO_ReadPin(GPIOB, BTN_GREEN_Pin));
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 83;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 2000;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 83;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(ONBOARD_LED_GPIO_Port, ONBOARD_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LED_YELLOW_Pin | LED_GREEN_Pin | LED_RED_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : ONBOARD_LED_Pin */
	GPIO_InitStruct.Pin = ONBOARD_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ONBOARD_LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : BTN_GREEN_Pin BTN_YELLOW_Pin */
	GPIO_InitStruct.Pin = BTN_GREEN_Pin | BTN_YELLOW_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : BUZZER_Pin */
	GPIO_InitStruct.Pin = BUZZER_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LED_YELLOW_Pin LED_GREEN_Pin LED_RED_Pin */
	GPIO_InitStruct.Pin = LED_YELLOW_Pin | LED_GREEN_Pin | LED_RED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void ShortBeep(int count) {
	int i = 0;
	for (i = 0; i < count; i++) {
		beep_buffer[i << 1] = 100;
		beep_buffer[(i << 1) + 1] = 30;
	}
	beep_length = count * 2;
	beep_index = 0;
	beep_tick = 0;
}

void LongBeep(int count) {
	int i = 0;
	for (i = 0; i < count; i++) {
		beep_buffer[i << 1] = 300;
		beep_buffer[(i << 1) + 1] = 30;
	}
	beep_length = count * 2;
	beep_index = 0;
	beep_tick = 0;
}

void ShortLongBeep(int short_count, int long_count) {
	int i = 0;
	int count = short_count + long_count;
	for (i = 0; i < count; i++) {
		beep_buffer[i << 1] = i < short_count ? 100 : 300;
		beep_buffer[(i << 1) + 1] = 30;
	}
	beep_length = count * 2;
	beep_index = 0;
	beep_tick = 0;
}

void TIMER2_ISR() {
	beep_tick++;

	if (beep_index < beep_length) {
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,
				~(beep_index & 0x01) & 0x01);
		if (beep_tick > beep_buffer[beep_index]) {
			beep_index++;
			beep_tick = 0;
		}
	}

	led_tick++;
	if (!connected_flag) {
		if ((led_tick) > 100) {
			HAL_GPIO_TogglePin(GPIOB, led_pins[led_pin_count++]);
			if (led_pin_count >= 3)
				led_pin_count = 0;
			led_tick = 0;
		}
	} else {
		switch (current_status) {
		case IDLE:
			if ((led_tick) > 500) {
				HAL_GPIO_WritePin(GPIOB, LED_YELLOW_Pin | LED_GREEN_Pin,
						GPIO_PIN_RESET);
				HAL_GPIO_TogglePin(GPIOB, LED_RED_Pin);
				led_tick = 0;
			}
			break;
		case RUNNING_A:
			if ((led_tick) > 100) {
				HAL_GPIO_WritePin(GPIOB, LED_RED_Pin | LED_YELLOW_Pin,
						GPIO_PIN_RESET);
				HAL_GPIO_TogglePin(GPIOB, LED_GREEN_Pin);
				led_tick = 0;
			}
			break;
		case RUNNING_B:
			if ((led_tick) > 100) {
				HAL_GPIO_WritePin(GPIOB, LED_RED_Pin | LED_GREEN_Pin,
						GPIO_PIN_RESET);
				HAL_GPIO_TogglePin(GPIOB, LED_YELLOW_Pin);
				led_tick = 0;
			}
			break;
		case STOPPED:
			HAL_GPIO_WritePin(GPIOB, LED_RED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin | LED_YELLOW_Pin,
					GPIO_PIN_RESET);
			break;
		case ERR:
			if ((led_tick) > 100) {
				HAL_GPIO_WritePin(GPIOB, LED_YELLOW_Pin | LED_GREEN_Pin,
						GPIO_PIN_RESET);
				HAL_GPIO_TogglePin(GPIOB, LED_RED_Pin);
				HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
				led_tick = 0;
			}
			break;
		default:
			break;
		}
	}

	if (!HAL_GPIO_ReadPin(GPIOB, BTN_GREEN_Pin)) {
		btn_green_tick++;
	}

	if (!HAL_GPIO_ReadPin(GPIOB, BTN_YELLOW_Pin)) {
		btn_yellow_tick++;
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
		HAL_GPIO_WritePin(ONBOARD_LED_GPIO_Port, ONBOARD_LED_Pin, GPIO_PIN_SET);
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
