/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stdlib.h"
#include "string.h"
#include "ctype.h"
#include "math.h"

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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
int isDangerButtonPressed = 0;
int onOffTime;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
//handlers
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
//buzzer
void PWM_Play();
void PWM_Change_Tone(uint16_t pwm_freq, uint16_t volume);
void PWM_Pause();
void Change_Melody(const Tone* melody , const uint32_t wholenote,uint16_t melody_len);
void Update_Melody();
//elevator admin commands
char* Procces_Command(char* string);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_PCD_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 23;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD10 PD11 PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD1 PD2 PD3 PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_10){

	}else if(GPIO_Pin == GPIO_PIN_11){

	}else if(GPIO_Pin == GPIO_PIN_12){

	}else if(GPIO_Pin == GPIO_PIN_13){

	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM4){
		if(ssEnableDigit == 1){
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, 0);
			intToBcd(inputFloor);
			ssEnableDigit = 4;
		  }else if(ssEnableDigit == 4){
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_1, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0);
			intToBcd(currentFloor);
			ssEnableDigit = 1;
		}
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART3){
		if(data != 0x0A){
			uart_buffer[uart_buffer_pos] = data;
			uart_buffer_pos += 1;
		}else{
			uart_buffer[uart_buffer_pos] = '\0';
			/*
			 * ADMIN#{1234}
			 *
			 */
			int str_len = uart_buffer_pos+2; //if user enter {aa} str_len is 4 because {a + a + 0x0A + \0}
			char input[str_len];
			for(int i=0;i<str_len;i++){
				input[i] = 0;
			}
			for (int i = 0; i < str_len; i++) {
				input[i] = toupper(buffer[i]); //for example if user enter ADMIN it changes to ADMIN for next processes
			}
			char* string = input;
			if(isAdminLogin == 0){
				//if admin is not login
				if(strcmp(string,"ADMIN#{1234}") == 0){
					char* response = "You Login Successfully !!!";
					HAL_UART_Transmit(&huart3, response, sizeof(*response), 1000);
					isAdminLogin = 1;
				}else{
					char* response = "You Should Login First !!!";
					HAL_UART_Transmit(&huart3, response, sizeof(*response), 1000);
				}
			}else{
				if(strcmp(string,"ADMIN#{1234}") == 0){
					char* response = "You Are Already Logged In !!!";
					HAL_UART_Transmit(&huart3, response, sizeof(*response), 1000);
					isAdminLogin = 1;
				}else if(strcmp(string,"SET MAX LEVEL []") == 0){

				}
			}

			if(strcmp(string,"ADMIN#{1234}") == 0){
				isAdminLogin = 1;
			}

		}
		HAL_UART_Receive_IT(&huart3, &uart_data, sizeof(data));
	}
}
void PWM_Play(){
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

void PWM_Change_Tone(uint16_t pwm_freq, uint16_t volume) // pwm_freq (1 - 20000), volume (0 - 1000)
{
    if (pwm_freq == 0 || pwm_freq > 20000)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    }
    else
    {
        const uint32_t internal_clock_freq = HAL_RCC_GetSysClockFreq();
        const uint16_t prescaler = 1 + internal_clock_freq / pwm_freq / 60000;
        const uint32_t timer_clock = internal_clock_freq / prescaler;
        const uint32_t period_cycles = timer_clock / pwm_freq;
        const uint32_t pulse_width = (volume * 50) * period_cycles / 1000 / 2;
        TIM_HandleTypeDef* pwm = &htim2;
        pwm->Instance->PSC = prescaler - 1;
        pwm->Instance->ARR = period_cycles - 1;
        pwm->Instance->EGR = TIM_EGR_UG;
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_width); // pwm_timer->Instance->CCR2 = pulse_width;
    }
}

void PWM_Pause(){
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}

void Update_Melody(){
	if(HAL_GetTick()-onOffTime>200){
		if(isDangerButtonPressed == 0){
			isDangerButtonPressed = 1;
			PWM_Change_Tone(0, 50);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8 | GPIO_PIN_9| GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, 0);
		}else{
			isDangerButtonPressed = 0;
			PWM_Change_Tone(3951, 50);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8 | GPIO_PIN_9| GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, 1);
		}
	}
}

void intToBcd(int num){
  x1 = num & 1; //PA4 --> A  0001  kam arzesh
  x2 = num & 2; //PA3 --> B
  x3 = num & 4; //PA2 --> C
  x4 = num & 8; //PA1 --> D 1000 ba arzesh
  if(x1>0) x1=1;
  if(x2>0) x2=1;
  if(x3>0) x3=1;
  if(x4>0) x4=1;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, x4);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, x3);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, x2);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, x1);
}



/*
 * ERR Codes:
 * -1 = wrong command or parameter
 * -2 = wrong command
 * -3 = wrong parameter
 */
char* Process_Command(char* string){
	if(strstr(string,"SET MAX LEVEL") != NULL){
		int* p = Get_Parameter_In_Command_SML(string);

	}else if(strstr(string,"SET WAIT") != NULL){
		char* p = Get_Parameter_In_Command(string," ");
		char* r;
		int wait_time = strtol(p, &r, 10);
	}else if(strstr(string,"SET LED") != NULL){
		char* p = Get_Parameter_In_Command(string," ");
		if(strcmp(p,"ON") == 0){

		}else if(strcmp(p,"OFF") == 0){

		}
	}else if(strstr(string,"SET LEVEL") != NULL){
		char* p = Get_Parameter_In_Command(string," ");
		char* r;
		int current_floor = strtol(p, &r, 10);
	}else if(strstr(string,"TEST#") != NULL){
		char* p = Get_Parameter_In_Command(string,"#");

	}else if(strstr(string,"START") != NULL){

	}else if(strstr(string,"ADMIN#") != NULL){
		char* p = Get_Parameter_In_Command(string,"#");
	}else{

	}
}

int* Get_Parameter_In_Command_START(char* string,int* res_out){
	//START
	int res[1] = {0};
	if(strcmp(string,"START") == 0 ){
		res[0] = 0;
	}else{
		res[0] = -1;
	}
	return res;
}
int* Get_Parameter_In_Command_AP(char* string,int* res_out){
	//ADMIN PASS => ADMIN#{1234}
	char copy_string[strlen(string)];
	strcpy(copy_string,string);
	int queu[5] = {-1};
	int* queue = queu;
	char* command_string_parts[2] = {"",""};
	int i = 0;
	char* res = strtok(copy_string,"#");
	while(res != NULL && i < 2){
		command_string_parts[i]=res;
		i += 1;
		res = strtok(NULL,"#");
	}
	if(
			res != NULL ||
			(
				strcmp(command_string_parts[0],"ADMIN") !=0
			)
	){
		queue[0]=-2;
		return queue;
	}
	if(strcmp(command_string_parts[1],"1234") !=0){
		queue[0]=-3;
		return queue;
	}
	queue[0] = 0;
	return queue;

}

void Get_Parameter_In_Command_SML(char* string,int* res_out){
	//SML => SET MAX LEVEL
	char copy_string[strlen(string)];
	strcpy(copy_string,string);
	int res_temp[1] = {-1};
	int* max_level = res_temp;
	char* command_string_parts[4] = {"","","",""};
	int i = 0;
	char* res = strtok(copy_string," ");
	while(res != NULL && i < 4){
		//Set Max Level 2 => if we split string with space we have 3 substrings.
		command_string_parts[i] = res;
		i += 1;
		res = strtok(NULL," ");
	}
	if(
			res != NULL ||
			(
				strcmp(command_string_parts[0],"SET") !=0
				|| strcmp(command_string_parts[1],"MAX") !=0
				|| strcmp(command_string_parts[2],"LEVEL") !=0
			)
	){
		max_level[0] = -2;
		return max_level;
	}
	int string_ml_len = strlen(command_string_parts[3]);
	if(string_ml_len != 1){
		max_level[0] = -3;
		return max_level;
	}
	char string_ml[1];
	int number_wt[1];
	strcpy(string_ml,command_string_parts[3]);

	int ascii_char = (int) (string_ml[0]);
	if(ascii_char < 48 || ascii_char > 57){
		max_level[0] = -1;
		return max_level;
	}else{
		number_wt[j] = ascii_char - 48;
	}
	return number_wt;

}
void Get_Parameter_In_Command_SLED(char* string,int* res_out){
	//SLED => SET LED
	char copy_string[strlen(string)];
	strcpy(copy_string,string);
	int res_temp[1] = {-1};
	int* level = res_temp;
	char* command_string_parts[3] = {"","",""};
	int i = 0;
	char* res = strtok(copy_string," ");
	while(res != NULL && i < 3){
		//Set Max Level 2 => if we split string with space we have 3 substrings.
		command_string_parts[i] = res;
		i += 1;
		res = strtok(NULL,copy_string);
	}
	if(
			res != NULL ||
			(
				strcmp(command_string_parts[0],"SET") !=0
				|| strcmp(command_string_parts[1],"LED") !=0
			)
	){
		level[0] = -2;
		return level;
	}
	int number_nf[1];
	if(strcmp(command_string_parts[2],"ON") == 0){
		number_nf[0] = 1;
	}else if(strcmp(command_string_parts[2],"OFF") == 0){
		number_nf[0] = 0;
	}else{
		level[0] = -3;
		return level;
	}
	return number_nf;

}

int* Get_Parameter_In_Command_SL(char* string){
	//SL => SET LEVEL
	char copy_string[strlen(string)];
	strcpy(copy_string,string);
	int res_temp[1] = {-1};
	int* level = res_temp;
	char* command_string_parts[3] = {"","",""};
	int i = 0;
	char* res = strtok(copy_string," ");
	while(res != NULL && i < 3){
		//Set Max Level 2 => if we split string with space we have 3 substrings.
		command_string_parts[i] = res;
		i += 1;
		res = strtok(NULL,copy_string);
	}
	if(
			res != NULL ||
			(
				strcmp(command_string_parts[0],"SET") !=0
				|| strcmp(command_string_parts[1],"LEVEL") !=0
			)
	){
		level[0] = -2;
		return level;
	}
	int string_level_len = strlen(command_string_parts[2]);
	if(string_level_len != 1){
		level[0] = -3;
		return level;
	}
	char string_level[1];
	int number_level[1];
	strcpy(string_ml,command_string_parts[2]);
	int ascii_char = (int) (string_level[0]);
	if(ascii_char < 48 || ascii_char > 57){
		max_level[0] = -1;
		return max_level;
	}else{
		number_level[j] = ascii_char - 48;
	}
	return number_level;

}

int* Get_Parameter_In_Command_SWT(char* string){
	//SML => SET Wait
	char copy_string[strlen(string)];
	strcpy(copy_string,string);
	int res_temp[1] = {-1};
	int* wait_time = res_temp;
	char* command_string_parts[3] = {"","",""};
	int i = 0;
	char* res = strtok(copy_string," ");
	while(res != NULL && i < 3){
		//Set Wait 200 => if we split string with space we have 3 substrings.
		command_string_parts[i] = res;
		i += 1;
		res = strtok(NULL,copy_string);
	}
	if(
			res != NULL ||
			(
				strcmp(command_string_parts[0],"SET") !=0
				|| strcmp(command_string_parts[1],"WAIT") !=0
			)
	){
		wait_time[0] = -2;
		return wait_time;
	}
	int string_wt_len = strlen(command_string_parts[2]);
	char string_wt[string_wt_len];
	int number_wt[string_wt_len];
	strcpy(string_wt,command_string_parts[2]);
	for(int j=0;j<string_wt_len;j++){
		int ascii_char = (int) (string_wt[j]);
		if(ascii_char < 48 || ascii_char > 57){
			wait_time[0] = -1;
			return wait_time;
		}else{
			number_wt[j] = ascii_char - 48;
		}
	}
	if(number_wt[string_wt_len-1] !=0 || number_wt[string_wt_len-2] !=0){
		wait_time[0] = -3;
		return wait_time;
	}
	int wt[]={0};
	for(int j=0;j<string_wt_len;j++){
		int tavan = string_wt_len - 1 - j;
		wt[0] += pow(10,tavan);
	}
	return wt;
}
int* Get_Parameter_In_Command_Test(char* string){
	char copy_string[strlen(string)];
	strcpy(copy_string,string);
	int queu[5] = {-1};
	int* queue = queu;
	char* command_string_parts[2] = {"",""};
	int i = 0;
	char* res = strtok(copy_string,"#");
	while(res != NULL && i < 2){
		command_string_parts[i]=res;
		i += 1;
		res = strtok(NULL,copy_string);
	}
	if(
			res != NULL ||
			(
				strcmp(command_string_parts[0],"TEST") !=0
			)
	){
		queue[0]=-2;
		return queue;
	}
	int string_queue_len = strlen(command_string_parts[1]);
	char string_queue[string_queue_len];
	strcpy(string_queue,command_string_parts[1]);
	if(string_queue[0] == '\n' || string_queue[0] == ' '){
		return queue;
	}
	i = 0;
	while(string_queue[i] != '\n' && i < string_queue_len){
		if(isdigit(string_queue[i])){
    		queue[i] = (int) (string_queue[i]) - 48;
		}else{
		    queue[0] = -3;
		}
		i++;
	}
	return queue;
}

char* Get_Parameter_In_Command(char* string,char* delim){
	char* res =strtok(string, delim);
	while(res != NULL){
		res = strtok(NULL,delim);
	}
	return strtok(res," ");
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
  while (1)
  {
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
