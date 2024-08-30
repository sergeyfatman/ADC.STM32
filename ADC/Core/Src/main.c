/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "string.h"
#include "usbd_hid.h"


#define QTY_Joysticks 2
#define Mapping_PWM_Value_Min 800
#define Mapping_PWM_Value_Max 1500
#define Not_Initialized -1
#define Dead_Zone_Percents 5

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint32_t ADC_XY_Joystick[QTY_Joysticks * 2];
struct min_max_struct {
	int32_t Min_X;
	int32_t Max_X;
	int32_t Min_Y;
	int32_t Max_Y;
} ADC_MIN_MAX_XY_Joystick[QTY_Joysticks];
struct pwm_multiplier_struct {
	float K_X;
	float L_X;
	int32_t Dead_Min_X;
	int32_t Dead_Max_X;
	float K_Y;
	float L_Y;
	int32_t Dead_Min_Y;
	int32_t Dead_Max_Y;
} PWM_XY_Joystick_Multiplier[QTY_Joysticks];
struct pwm_xy_struct {
	uint32_t X;
	uint32_t Y;
} PWM_XY_Joystick[QTY_Joysticks];

bool Calibration_Mode = false;
bool Begin_Change_Mode = false;
uint32_t Button_Calibration_Time[4] = { 0, 0, 0, 0 };
bool Button_Calibration_Status[4] = { false, false, false, false };

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void Joystick_Data_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern USBD_HandleTypeDef hUsbDeviceFS;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		HAL_ADC_Start_DMA(&hadc1, ADC_XY_Joystick, QTY_Joysticks * 2);
	}
}

void setStatusLED(void) {
	if (Calibration_Mode) {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
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
	void Calibrate_MIN_MAX_ADC_XY_Joystick(void) {
		uint8_t Joystick_Number;
		uint32_t Joystick_X, Joystick_Y;
		int32_t Length_L1 = Mapping_PWM_Value_Max - Mapping_PWM_Value_Min,
				Length_L2 = 0;
		float K = 0;

		for (int index = 0; index < QTY_Joysticks; ++index) {
			Joystick_Number = index * 2;
			Joystick_X = ADC_XY_Joystick[Joystick_Number];
			Joystick_Y = ADC_XY_Joystick[Joystick_Number + 1];

			if (ADC_MIN_MAX_XY_Joystick[index].Min_X
					> Joystick_X|| ADC_MIN_MAX_XY_Joystick[index].Min_X == Not_Initialized) {
				ADC_MIN_MAX_XY_Joystick[index].Min_X = Joystick_X;
			}
			if (ADC_MIN_MAX_XY_Joystick[index].Max_X
					< Joystick_X|| ADC_MIN_MAX_XY_Joystick[index].Max_X == Not_Initialized) {
				ADC_MIN_MAX_XY_Joystick[index].Max_X = Joystick_X;
			}
			Length_L2 = ADC_MIN_MAX_XY_Joystick[index].Max_X
					- ADC_MIN_MAX_XY_Joystick[index].Min_X;
			if (Length_L2 != 0) {
				K = (float) (Length_L1 / Length_L2);
				PWM_XY_Joystick_Multiplier[index].K_X = K;
				PWM_XY_Joystick_Multiplier[index].L_X =
						(float) (Mapping_PWM_Value_Min
								- ADC_MIN_MAX_XY_Joystick[index].Min_X * K);
			}

			if (ADC_MIN_MAX_XY_Joystick[index].Min_Y
					> Joystick_Y|| ADC_MIN_MAX_XY_Joystick[index].Min_Y == Not_Initialized) {
				ADC_MIN_MAX_XY_Joystick[index].Min_Y = Joystick_Y;
			}
			if (ADC_MIN_MAX_XY_Joystick[index].Max_Y
					< Joystick_Y|| ADC_MIN_MAX_XY_Joystick[index].Max_Y == Not_Initialized) {
				ADC_MIN_MAX_XY_Joystick[index].Max_Y = Joystick_Y;
			}
			Length_L2 = ADC_MIN_MAX_XY_Joystick[index].Max_Y
					- ADC_MIN_MAX_XY_Joystick[index].Min_Y;
			if (Length_L2 != 0) {
				K = (float) (Length_L1 / Length_L2);
				PWM_XY_Joystick_Multiplier[index].K_Y = K;
				PWM_XY_Joystick_Multiplier[index].L_Y =
						(float) (Mapping_PWM_Value_Min
								- ADC_MIN_MAX_XY_Joystick[index].Min_Y * K);
			}
		}
	}

	void Count_PWM_XY_Joystick(void) {
		uint8_t Joystick_Number;
		uint32_t Joystick_X, Joystick_Y;

		for (int index = 0; index < QTY_Joysticks; ++index) {
			Joystick_Number = index * 2;
			Joystick_X = ADC_XY_Joystick[Joystick_Number];
			Joystick_Y = ADC_XY_Joystick[Joystick_Number + 1];

			if (PWM_XY_Joystick_Multiplier[index].K_X != Not_Initialized
					&& PWM_XY_Joystick_Multiplier[index].L_X != Not_Initialized) {
				PWM_XY_Joystick[index].X = Joystick_X
						* PWM_XY_Joystick_Multiplier[index].K_X
						+ PWM_XY_Joystick_Multiplier[index].L_X;
			}
			if (PWM_XY_Joystick_Multiplier[index].K_Y != Not_Initialized
					&& PWM_XY_Joystick_Multiplier[index].L_Y != Not_Initialized) {
				PWM_XY_Joystick[index].Y = Joystick_Y
						* PWM_XY_Joystick_Multiplier[index].K_Y
						+ PWM_XY_Joystick_Multiplier[index].L_Y;
			}
		}
	}

	void Check_Calibration_Buttons(void) {
		bool Button_Status[2] = { false, false };
		Button_Status[0] = !HAL_GPIO_ReadPin(GPIOA, ButtonC1_Pin);
		Button_Status[1] = !HAL_GPIO_ReadPin(GPIOA, ButtonC2_Pin);

		for (int index = 0; index < 2; ++index) {
			if (Button_Status[index]) {
				if (Button_Calibration_Time[index] > 0
						&& HAL_GetTick() - Button_Calibration_Time[index]
								> 20) {
					Button_Calibration_Status[index] = true;
					if (HAL_GetTick() - Button_Calibration_Time[index + 2]
							> 5000) {
						Button_Calibration_Status[index + 2] = true;

					} else {
						Button_Calibration_Status[index + 2] = false;
					}
				} else {
					Button_Calibration_Status[index] = false;
					Button_Calibration_Status[index + 2] = false;
					if (Button_Calibration_Time[index] == 0) {
						Begin_Change_Mode = true;
						Button_Calibration_Time[index] = HAL_GetTick();
						Button_Calibration_Time[index + 2] = HAL_GetTick();
					}
				}
			} else {
				Button_Calibration_Time[index] = 0;
				Button_Calibration_Status[index] = false;
				Button_Calibration_Time[index + 2] = 0;
				Button_Calibration_Status[index + 2] = false;
				Begin_Change_Mode = false;
			}
		}

		if (Begin_Change_Mode && Button_Calibration_Status[2]
				&& Button_Calibration_Status[3]) {
			Begin_Change_Mode = false;
			Calibration_Mode = !Calibration_Mode;
		}
		setStatusLED();
	}

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	Joystick_Data_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, ADC_XY_Joystick, QTY_Joysticks * 2);
	HAL_ADC_Start_IT(&hadc1);
	setStatusLED();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Check_Calibration_Buttons();
		if (Calibration_Mode) {
			Calibrate_MIN_MAX_ADC_XY_Joystick();
		} else {
			Count_PWM_XY_Joystick();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ButtonC1_Pin ButtonC2_Pin */
  GPIO_InitStruct.Pin = ButtonC1_Pin|ButtonC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Joystick_Data_Init(void) {
	for (int index = 0; index < QTY_Joysticks; ++index) {
		ADC_MIN_MAX_XY_Joystick[index].Min_X = Not_Initialized;
		ADC_MIN_MAX_XY_Joystick[index].Max_X = Not_Initialized;
		ADC_MIN_MAX_XY_Joystick[index].Min_Y = Not_Initialized;
		ADC_MIN_MAX_XY_Joystick[index].Max_Y = Not_Initialized;

		PWM_XY_Joystick_Multiplier[index].K_X = Not_Initialized;
		PWM_XY_Joystick_Multiplier[index].L_X = Not_Initialized;
		PWM_XY_Joystick_Multiplier[index].Dead_Min_X = Not_Initialized;
		PWM_XY_Joystick_Multiplier[index].Dead_Max_X = Not_Initialized;
		PWM_XY_Joystick_Multiplier[index].K_Y = Not_Initialized;
		PWM_XY_Joystick_Multiplier[index].L_Y = Not_Initialized;
		PWM_XY_Joystick_Multiplier[index].Dead_Min_Y = Not_Initialized;
		PWM_XY_Joystick_Multiplier[index].Dead_Max_Y = Not_Initialized;

		PWM_XY_Joystick[index].X = Not_Initialized;
		PWM_XY_Joystick[index].Y = Not_Initialized;
	}
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
