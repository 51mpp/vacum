/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_LEN   1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
char state[100] = "stop";
char ch1;
char str_vr4[100];
float ena_1 = 1.0;
float enb_1 = 1.0;
float ena_2 = 1.0;
float enb_2 = 1.0;
uint8_t mode = 0;
uint8_t rxData;
uint8_t avDis;
uint8_t light = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance = 0;
uint8_t pwm;
uint8_t value = 200;

#define TRIG_PIN GPIO_PIN_8
#define TRIG_PORT GPIOE

void delay(uint16_t microsec) {
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER(&htim1) < microsec)
		;

}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // if the interrupt source is channel1
			{
		if (Is_First_Captured == 0) // if the first value is not captured
				{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
					TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured == 1)   // if the first is already captured
				{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1) {
				Difference = IC_Val2 - IC_Val1;
			}

			else if (IC_Val1 > IC_Val2) {
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034 / 2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
					TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		}
	}
}
void HCSR04_Read(void) {
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET); // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET); // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
}
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
	MX_USART3_UART_Init();
	MX_USB_OTG_FS_PCD_Init();
	MX_USART1_UART_Init();
	MX_TIM2_Init();
	MX_TIM1_Init();
	MX_I2C1_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
//	HAL_UART_Receive_IT(&huart1, (uint16_t*) &state, 64);
	HAL_UART_Receive_IT(&huart1, &rxData, 1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	char strdistance[100];
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	int average(int x);
	float red = 0.0;
	float green = 0.0;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		htim2.Instance->CCR1 = (1000 - 1) * ena_1;
		htim2.Instance->CCR2 = (1000 - 1) * enb_1;
		htim2.Instance->CCR3 = (1000 - 1) * ena_2;
		htim2.Instance->CCR4 = (1000 - 1) * enb_2;
		htim3.Instance->CCR1 = (1000 - 1) * red;
		htim3.Instance->CCR2 = (1000 - 1) * green;
		HCSR04_Read();
		delay(10000);
		avDis = average(Distance);
		sprintf(strdistance, "%d\n\r", avDis);

		HAL_UART_Transmit(&huart3, (uint8_t*) strdistance, strlen(strdistance),1000);
		if (mode == 0) {
			if (avDis <= 15) {
				light = 0;
				HAL_UART_Transmit(&huart1,  &light, 1, 1000);

				//					HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
				//					HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
				//					pwm = (GPIOA->IDR & GPIO_PIN_6) >> 10;
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
				//					htim3.Instance->CCR1 = value;
				red = 1.0;
				green = 0.0;

			} else {
				light =1;
				HAL_UART_Transmit(&huart1,  &light, 1, 1000);

				//					HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
				//					HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
				//					pwm = (GPIOA->IDR & GPIO_PIN_7) >> 10;
				red = 0.0;
				green = 1.0;
			}
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
		} else if (mode == 1) {
			light=0;
			HAL_UART_Transmit(&huart1,  &light, 1, 1000);
			ena_1 =1.0;
			enb_1 =1.0;
			ena_2 = 1.0;
			enb_2 = 1.0;
			if (avDis <= 15) {
				//					HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
				//					HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
				//					pwm = (GPIOA->IDR & GPIO_PIN_6) >> 10;
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
				//					htim3.Instance->CCR1 = value;
				HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, 0);
				HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, 1);
				//
				HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, 1);
				HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, 0);
				//
				HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, 0);
				HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, 1);
				//
				HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, 1);
				HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, 0);
				HAL_Delay(1000);



				ena_1 =1.0;
							enb_1 =1.0;
							ena_2 = 0.3;
							enb_2 = 0.8;
				red = 1.0;
				green = 0.0;
								HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, 1);
								HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, 0);
								//
								HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, 0);
								HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, 1);
								//
								HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, 0);
								HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, 1);
								//
								HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, 1);
								HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, 0);
				HAL_Delay(900);
//				ena_1 = 1.0;
//				enb_1 = 1.0;
//
//				HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, 1);		//ขวาบน
//				HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, 0);
//				//
//				HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, 0);
//				HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, 1);
//				//
//				HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, 1);	//ซ้ายล่าง
//				HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, 0);
//				//
//				HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, 0);		//ซ้ายบน
//				HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, 1);
//				HAL_Delay(200);
//				ena_1 = 0.5;
//				enb_1 = 0.5;
//				red = 1.0;
//				green = 0.0;
//				HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, 1);
//				HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, 0);
//				//
//				HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, 0);
//				HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, 1);
//				//
//				HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, 0);
//				HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, 1);
//				//
//				HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, 1);
//				HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, 0);
//				HAL_Delay(300);

			} else {
				light=1;
				HAL_UART_Transmit(&huart1,  &light, 1, 1000);

				//					HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
				//					HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
				//					pwm = (GPIOA->IDR & GPIO_PIN_7) >> 10;
				red = 0.0;
				green = 1.0;
				HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, 1);		//ขวาบน
				HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, 0);
				//
				HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, 0);
				HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, 1);
				//
				HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, 1);	//ซ้ายล่าง
				HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, 0);
				//
				HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, 0);		//ซ้ายบน
				HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, 1);
			}
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
		}
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

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 96;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x20303E5D;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 96 - 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535 - 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_IC_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
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
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 84 - 1;
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
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 96 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1000 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_FS_PCD_Init(void) {

	/* USER CODE BEGIN USB_OTG_FS_Init 0 */

	/* USER CODE END USB_OTG_FS_Init 0 */

	/* USER CODE BEGIN USB_OTG_FS_Init 1 */

	/* USER CODE END USB_OTG_FS_Init 1 */
	hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
	hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
	hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
	hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
	hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
	hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
	hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
	if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USB_OTG_FS_Init 2 */

	/* USER CODE END USB_OTG_FS_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOF, IN3_1_Pin | IN4_1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, IN1_1_Pin | IN2_1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin | LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, IN4_2_Pin | IN3_2_Pin | IN2_2_Pin | IN1_2_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : USER_Btn_Pin */
	GPIO_InitStruct.Pin = USER_Btn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : IN3_1_Pin IN4_1_Pin */
	GPIO_InitStruct.Pin = IN3_1_Pin | IN4_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pins : IN1_1_Pin IN2_1_Pin */
	GPIO_InitStruct.Pin = IN1_1_Pin | IN2_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
	GPIO_InitStruct.Pin = RMII_MDC_Pin | RMII_RXD0_Pin | RMII_RXD1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin */
	GPIO_InitStruct.Pin = RMII_REF_CLK_Pin | RMII_MDIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
	GPIO_InitStruct.Pin = LD1_Pin | LD3_Pin | LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PE8 */
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : RMII_TXD1_Pin */
	GPIO_InitStruct.Pin = RMII_TXD1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_OverCurrent_Pin */
	GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : IN4_2_Pin IN3_2_Pin IN2_2_Pin IN1_2_Pin */
	GPIO_InitStruct.Pin = IN4_2_Pin | IN3_2_Pin | IN2_2_Pin | IN1_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
	GPIO_InitStruct.Pin = RMII_TX_EN_Pin | RMII_TXD0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	/* Prevent unused argument(s) compilation warning */
	UNUSED(huart);

	if (huart->Instance == huart1.Instance) {
		ena_1 = 1.0;
		enb_1 = 1.0;
		ena_2 = 1.0;
		enb_2 = 1.0;
		if (mode == 0) {
			if (rxData == 'B') {
				//
				HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, 0);
				HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, 0);
				//
				HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, 0);
				HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, 0);
				//
				HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, 0);
				HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, 0);
				//
				HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, 0);
				HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, 0);
			} else if (rxData == 'W') {
				HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, 1);		//ขวาบน
				HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, 0);
				//
				HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, 0);
				HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, 1);
				//
				HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, 1);	//ซ้ายล่าง
				HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, 0);
				//
				HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, 0);		//ซ้ายบน
				HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, 1);
			} else if (rxData == 'S') {
				HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, 0);
				HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, 1);
				//
				HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, 1);
				HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, 0);
				//
				HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, 0);
				HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, 1);
				//
				HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, 1);
				HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, 0);
				ena_2 = 0.5;
				enb_2 = 0.5;
			} else if (rxData == 'A') {
				HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, 1);
				HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, 0);
				//
				HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, 0);
				HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, 1);
				//
				HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, 0);
				HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, 1);
				//
				ena_1 = 0.5;
				enb_1 = 0.5;
				HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, 1);
				HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, 0);
			} else if (rxData == 'D') {
				HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, 0);
				HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, 1);
				//
				HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, 1);
				HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, 0);
				//
				HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, 1);
				HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, 0);
				//
				HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, 0);
				HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, 1);
			} else if (rxData == 'M') {
				mode = 1;
				rxData = 66;
			}

		} else if (mode == 1) {
			if (rxData == 'N') {
				mode = 0;
			}
		}
		char temp[50];
		sprintf(temp, "%d", mode);
		HAL_UART_Transmit(&huart3, &rxData, 1, 1000);

		HAL_UART_Receive_IT(&huart1, &rxData, 1);

	}

}
int average(int x) {
	static int samples[16];
	static int i = 0;
	static int total = 0;
	total += x - samples[i];
	samples[i] = x;
	i = (i == 15 ? 0 : i + 1);
	return total / 16;
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
