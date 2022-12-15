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
/*
 *  1. Set Max Level
 *  2. Set Level
 *  3. Set Wait
 *  4. Set LED
 *  5. Test
 *  6. Admin
 *  7. Start
 *
 */
typedef struct ProcessCommandResult {
	int command_number;
	char message[50];
	int parameters[10];
} ProcessCommandResult;

typedef struct Queue{
	int floors[10];
	int length; //count of floors in queue
} Queue;

/*
 * 1 => Empty State
 * 2 => Admin State
 * 3 => Move State
 * 4 => Waiting State
 */
typedef int State;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ES 1  //Empty State
#define AS 2  //Admin State
#define MS 3   //Move State
#define WS 4	//Wait State
#define MOVE_TIME 1000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
unsigned char uart_buffer[20];
unsigned char uart_data;
int uart_buffer_pos = 0;

int max_level = 9;
int level = 0; //current floor
int next_floor = -1;
int choice_floor = 0;
Queue floors_queue;
int wait_time = 2000;
int wait_counter = 1;
int move_counter = 1;
State state = ES;

int on_off_danger_time = 0;
int is_danger_button_active = 0;
int led_status;
int should_led_on_in_alarm = 0;
int is_user_can_active_alarm = 1;

int gpio_external_button_time =0;

int ssEnableDigit = 1;

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
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
//handlers
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
//buzzer
void PWM_Play();
void PWM_Change_Tone(uint16_t pwm_freq, uint16_t volume);
void PWM_Pause();
void Update_Melody();
//elevator admin commands

ProcessCommandResult Process_Command(char* string);

ProcessCommandResult Get_Parameter_In_Command_START(char* string);
ProcessCommandResult Get_Parameter_In_Command_AP(char* string);
ProcessCommandResult Get_Parameter_In_Command_SML(char* string);
ProcessCommandResult Get_Parameter_In_Command_SL(char* string);
ProcessCommandResult Get_Parameter_In_Command_SLED(char* string);
ProcessCommandResult Get_Parameter_In_Command_SWT(char* string);
ProcessCommandResult Get_Parameter_In_Command_TEST(char* string);

//int to bcd
void intToBcd(int x);

//alarm (de)active functions
void Active_Danger_Alarm();
void DeActive_Danger_Alarm();
//floors queue
int Is_Floors_Queue_Empty(Queue* q);
int getFloorInQueue(Queue* q);
void setFloorInQueue(Queue* q,int new_floor);
void Init_Floors_In_Queue_With_Empty(Queue* q);
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  Init_Floors_In_Queue_With_Empty(&floors_queue);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_UART_Receive_IT(&huart3, &uart_data, sizeof(uart_data));
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
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 2399;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
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

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT2_Pin;
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

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void Init_Floors_In_Queue_With_Empty(Queue* q){
	for (int i=0;i<10;i++){
		q->floors[i] = -1;
	}
	q->length=0;
	return;
}
int Is_Floors_Queue_Empty(Queue* q){
	if(q->floors[0] == -1){
		return 1;
	}else{
		return 0;
	}
}

int getFloorInQueue(Queue* q){

	int temp = q->floors[0];
	for(int i=0;i <max_level-1;i++){
		q->floors[i] = q->floors[i+1];
	}
	q->floors[max_level - 1] = -1;
	q->length--;
	return temp;
}

void setFloorInQueue(Queue* q,int new_floor){
	if(new_floor==next_floor){
		return;
	}
	for(int i=0 ; i < max_level ; i++){
		if(q->floors[i] == new_floor){
			return;
		}
	}

	q->floors[q->length] = new_floor;
	if(new_floor >= 0){
		q->length++;
	}
	return;
}

void Active_Danger_Alarm(){
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	  is_danger_button_active =1;
}
void DeActive_Danger_Alarm(){
	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
   	  is_danger_button_active =0;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(HAL_GetTick() - gpio_external_button_time <200){
		return;
	}

	if(GPIO_Pin == GPIO_PIN_10){
		if(choice_floor == max_level){
			choice_floor = max_level;
		}else{
			choice_floor += 1;
		}
	}else if(GPIO_Pin == GPIO_PIN_11){
		if(choice_floor == 0){
			choice_floor = 0;
		}else{
			choice_floor -= 1;
		}
	}else if(GPIO_Pin == GPIO_PIN_12){
		if(state == ES){
			setFloorInQueue(&floors_queue, choice_floor);
			next_floor = getFloorInQueue(&floors_queue);
			state = MS;
		}else if(state == MS || state == WS){
			setFloorInQueue(&floors_queue, choice_floor);
		}
	}else if(GPIO_Pin == GPIO_PIN_0){
		if(is_user_can_active_alarm == 1){
			if(is_danger_button_active == 0){
				Active_Danger_Alarm();
			}else{
				DeActive_Danger_Alarm();
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8 | GPIO_PIN_9| GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, 0);
			}
		}
	}
}
void intToBcd(int num){
  int x1 = num & 1; //PA4 --> A  0001  kam arzesh
  int x2 = num & 2; //PA3 --> B
  int x3 = num & 4; //PA2 --> C
  int x4 = num & 8; //PA1 --> D 1000 ba arzesh
  if(x1>0) x1=1;
  if(x2>0) x2=1;
  if(x3>0) x3=1;
  if(x4>0) x4=1;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, x4);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, x3);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, x2);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, x1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM4){
		//Period = 3 MS
		if(ssEnableDigit == 1){
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, 0);
			intToBcd(choice_floor %10 );
			ssEnableDigit = 4;
		}else if(ssEnableDigit == 4){
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_1, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0);
			intToBcd(level %10);
			ssEnableDigit = 1;
		}
	}else if(htim->Instance == TIM1){

		//Enable or Disable Elevator when state become AS

		//Period = 50 MS
		if(state == ES || state == AS){
			return;
		}else if(state == MS){
			if(move_counter * 50 < MOVE_TIME){
				move_counter += 1;
			}else{
				//End Of Moving Time For A Floor
				//When move time of the elevator for a floor is over,
				//we have 2 state:
				//state1: Elevator reached to desired floor:
				//state must changes to (ES) OR (WS (And Then Move For Next Floor In Queue))
				//in this state function enter to IF
				//state2: Elevator does'nt reached to desired floor:
				//state must changes to WS
				move_counter = 1;
				wait_counter = 1;

				if(level < next_floor){
					level++;
				}else if(level > next_floor){
					level--;
				}else{
					int is_floors_queue_empty = Is_Floors_Queue_Empty(&floors_queue);
					if(is_floors_queue_empty == 1){
						state = ES;
						next_floor = -1;
					}else{
						state = WS;
					}
				}

			}
		}else if (state == WS){
			if(wait_counter * 50 < wait_time){
				wait_counter += 1;
			}else{
				//End Of Waiting Time ::
				//When stop time of the elevator on a floor is over,
				//the elevator goes to the movement state (MS).
				//then the movement counter must be reset
				//to count the number of movement interruptions.
				move_counter = 1;
				state = MS;
				int is_floors_queue_empty = Is_Floors_Queue_Empty(&floors_queue);
				if(level == next_floor && is_floors_queue_empty != 1){
					next_floor = getFloorInQueue(&floors_queue);
				}
			}
		}
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART3){
		if(uart_data != 0x0A){
			uart_buffer[uart_buffer_pos] = uart_data;
			uart_buffer_pos += 1;
		}else{
			uart_buffer[uart_buffer_pos] = '\0';
			int str_len = uart_buffer_pos + 2; //if user enter {aa} str_len is 4 because {a + a + 0x0A + \0}
			char input[str_len];
			for (int i = 0; i < str_len; i++) {
				input[i] = toupper(uart_buffer[i]); //for example if user enter adMiN it changes to ADMIN for next processes
			}
			uart_buffer_pos = 0 ;

			ProcessCommandResult pcr = Process_Command(input);

			if(pcr.command_number == 1){
				//SET MAX LEVEL
				max_level = pcr.parameters[0];
				level = 0;
				Init_Floors_In_Queue_With_Empty(&floors_queue);

			}else if(pcr.command_number == 2){
				//SET LEVEL
				level = pcr.parameters[0];
				Init_Floors_In_Queue_With_Empty(&floors_queue);

			}else if(pcr.command_number == 3){
				//SET WAIT TIME
				wait_time = pcr.parameters[0];

			}else if(pcr.command_number == 4){
				//SET LED
				is_user_can_active_alarm = pcr.parameters[0];

			}else if(pcr.command_number == 5){
				//TEST
				Init_Floors_In_Queue_With_Empty(&floors_queue);
				for(int i=0;i < 10 && pcr.parameters[i] != -1;i++){
					setFloorInQueue(&floors_queue, pcr.parameters[i]);
				}
			}else if(pcr.command_number == 6){
				//Admin Login
				Init_Floors_In_Queue_With_Empty(&floors_queue);
				state = AS;
			}else if(pcr.command_number == 7){
				//Start
				choice_floor = 0;
				if(floors_queue.length>0){
					next_floor = getFloorInQueue(&floors_queue);
					state = MS;
				}else{
					state = ES;
				}
			}

			HAL_UART_Transmit(&huart3, pcr.message, strlen(pcr.message), 1000);
		}
		HAL_UART_Receive_IT(&huart3, &uart_data, sizeof(uart_data));
	}
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
        const uint32_t pulse_width = volume * period_cycles / 1000 / 2;
        TIM_HandleTypeDef* pwm = &htim2;
        pwm->Instance->PSC = prescaler - 1;
        pwm->Instance->ARR = period_cycles - 1;
        pwm->Instance->EGR = TIM_EGR_UG;
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_width); // pwm_timer->Instance->CCR2 = pulse_width;
    }
}

void Update_Melody(){

	if(is_danger_button_active == 1 && HAL_GetTick() - on_off_danger_time > 250){
		on_off_danger_time = HAL_GetTick();
		if(should_led_on_in_alarm == 0){
			should_led_on_in_alarm = 1;
			PWM_Change_Tone(0, 50);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8 | GPIO_PIN_9| GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, 0);
		}else{
			should_led_on_in_alarm = 0;
			PWM_Change_Tone(2637, 50);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8 | GPIO_PIN_9| GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, 1);
		}
	}
}


ProcessCommandResult Process_Command(char* string){
	ProcessCommandResult pcr = {
			-1,"There is not a command with this name!!!\n", {}
	};
	if(strstr(string,"SET MAX LEVEL") != NULL){
		pcr = Get_Parameter_In_Command_SML(string);
	}else if(strstr(string,"SET WAIT") != NULL){
		pcr = Get_Parameter_In_Command_SWT(string);
	}else if(strstr(string,"SET LED") != NULL){
		pcr = Get_Parameter_In_Command_SLED(string);
	}else if(strstr(string,"SET LEVEL") != NULL){
		pcr = Get_Parameter_In_Command_SL(string);
	}else if(strstr(string,"TEST#") != NULL){
		pcr = Get_Parameter_In_Command_TEST(string);
	}else if(strstr(string,"START") != NULL){
		pcr = Get_Parameter_In_Command_START(string);
	}else if(strstr(string,"ADMIN#") != NULL){
		pcr= Get_Parameter_In_Command_AP(string);
	}
	return pcr;
}

ProcessCommandResult Get_Parameter_In_Command_START(char* string){
	//START
	ProcessCommandResult pcr = {
		7,"Successful Process",{}
	};
	for(int i=0; i < sizeof(pcr.parameters)/sizeof(pcr.parameters[0]) ; i++){
		pcr.parameters[i] = -1;
	}

	if(state != AS){
		pcr.command_number = -1;
		strcpy(pcr.message,"Admin is not logged\n");
		return pcr;
	}

	if(strcmp(string,"START") != 0 ){
		pcr.command_number = -1;
		strcpy(pcr.message,"command is wrong!!!\n");
	}else{
		strcpy(pcr.message,"program is exited from admin mode!!!\n");
	}
	return pcr;
}
ProcessCommandResult Get_Parameter_In_Command_AP(char* string){
	//ADMIN PASS => ADMIN#{1234}
	ProcessCommandResult pcr = {
		6,"Error",{}
	};
	for(int i=0; i < sizeof(pcr.parameters)/sizeof(pcr.parameters[0]) ; i++){
		pcr.parameters[i] = -1;
	}
	if(state == AS){
		pcr.command_number = -1;
		strcpy(pcr.message,"already you logged in.\n");
		return pcr;
	}
	if(state != ES){
		pcr.command_number = -1;
		strcpy(pcr.message,"for login, queue of elevator should be empty!!!\n");
		return pcr;
	}
	char copy_string[strlen(string)];
	strcpy(copy_string,string);
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
		pcr.command_number = -1;
		strcpy(pcr.message,"command is wrong!!!\n");
		return pcr;
	}
	if(strcmp(command_string_parts[1],"1234") !=0){
		pcr.command_number = -1;
		strcpy(pcr.message , "password is wrong!!!\n");
		return pcr;
	}else{
		strcpy(pcr.message , "admin is logged successfully!!!\n");
	}
	return pcr;

}

ProcessCommandResult Get_Parameter_In_Command_SML(char* string){
	//SML => SET MAX LEVEL
	ProcessCommandResult pcr = {
		1,"Error",{}
	};
	for(int i=0; i < sizeof(pcr.parameters)/sizeof(pcr.parameters[0]) ; i++){
		pcr.parameters[i] = -1;
	}

	if(state != AS){
		pcr.command_number = -1;
		strcpy(pcr.message,"Admin is not logged\n");
		return pcr;
	}

	char copy_string[strlen(string)];
	strcpy(copy_string,string);
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
		strcpy(pcr.message , "command is wrong!!!\n");
		pcr.command_number = -1;
		return pcr;
	}
	int string_ml_len = strlen(command_string_parts[3]);
	if(string_ml_len != 1){
		strcpy(pcr.message , "command parameter is wrong . range is (0-9)!!!\n");
		pcr.command_number = -1;
		return pcr;
	}

	char string_ml[1];
	strcpy(string_ml,command_string_parts[3]);

	int ascii_char = (int) (string_ml[0]);
	if(ascii_char < 48 || ascii_char > 57){
		strcpy(pcr.message,"command parameter is wrong . range is (0-9)!!!\n");
		pcr.command_number = -1;
		return pcr;
	}else{
		char msg[50];
		sprintf(msg,"max level is changed to %d\n",ascii_char - 48);
		strcpy(pcr.message,msg);
		pcr.parameters[0] = ascii_char - 48;
	}
	return pcr;

}
ProcessCommandResult Get_Parameter_In_Command_SLED(char* string){
	//SLED => SET LED
	ProcessCommandResult pcr = {
		4,"Error",{}
	};
	for(int i=0; i < sizeof(pcr.parameters)/sizeof(pcr.parameters[0]) ; i++){
		pcr.parameters[i] = -1;
	}

	if(state != AS){
		pcr.command_number = -1;
		strcpy(pcr.message,"Admin is not logged\n");
		return pcr;
	}

	char copy_string[strlen(string)];
	strcpy(copy_string,string);
	char* command_string_parts[3] = {"","",""};
	int i = 0;
	char* res = strtok(copy_string," ");
	while(res != NULL && i < 3){
		//Set Max Level 2 => if we split string with space we have 3 substrings.
		command_string_parts[i] = res;
		i += 1;
		res = strtok(NULL, " ");
	}
	if(
			res != NULL ||
			(
				strcmp(command_string_parts[0],"SET") !=0
				|| strcmp(command_string_parts[1],"LED") !=0
			)
	){
		strcpy(pcr.message , "command is wrong!!!\n");
		pcr.command_number = -1;
		return pcr;
	}
	if(strcmp(command_string_parts[2],"ON") == 0){
		char msg[50] = "led is changed to on\n";
		strcpy(pcr.message,msg);
		pcr.parameters[0] = 1;
	}else if(strcmp(command_string_parts[2],"OFF") == 0){
		char msg[50] = "led is changed to off\n";
		strcpy(pcr.message,msg);
		pcr.parameters[0] = 0;
	}else{
		strcpy(pcr.message , "command parameter is wrong . parameter is {off} or {on}!!!\n");
		pcr.command_number = -1;
		return pcr;
	}
	return pcr;

}

ProcessCommandResult Get_Parameter_In_Command_SL(char* string){
	//SL => SET LEVEL
	ProcessCommandResult pcr = {
		2,"Error",{}
	};
	for(int i=0; i < sizeof(pcr.parameters)/sizeof(pcr.parameters[0]) ; i++){
		pcr.parameters[i] = -1;
	}

	if(state != AS){
		pcr.command_number = -1;
		strcpy(pcr.message,"Admin is not logged\n");
		return pcr;
	}

	char copy_string[strlen(string)];
	strcpy(copy_string,string);
	char* command_string_parts[3] = {"","",""};
	int i = 0;
	char* res = strtok(copy_string," ");
	while(res != NULL && i < 3){
		//Set Max Level 2 => if we split string with space we have 3 substrings.
		command_string_parts[i] = res;
		i += 1;
		res = strtok(NULL, " ");
	}
	if(
			res != NULL ||
			(
				strcmp(command_string_parts[0],"SET") !=0
				|| strcmp(command_string_parts[1],"LEVEL") !=0
			)
	){
		strcpy(pcr.message, "command is wrong!!!\n");
		pcr.command_number = -1;
		return pcr;
	}
	int string_level_len = strlen(command_string_parts[2]);
	if(string_level_len != 1){
		strcpy(pcr.message, "command parameter is wrong . parameter range is (0-9)!!!\n");
		pcr.command_number = -1;
		return pcr;
	}
	char string_level[1];
	strcpy(string_level,command_string_parts[2]);
	int ascii_char = (int) (string_level[0]);
	int max_level_ascii = 48 + max_level;
	if(ascii_char < 48 || ascii_char > max_level_ascii){
		char err[50];
		sprintf(err,"command parameter is wrong . parameter range is (0-%d)!!!\n",max_level_ascii-48);
		strcpy(pcr.message , err);
		pcr.command_number = -1;
		return pcr;
	}else{
		char msg[50];
		sprintf(msg,"level is changed to %d\n",ascii_char -48);
		strcpy(pcr.message,msg);
		pcr.parameters[0] = ascii_char -48;
	}
	return pcr;

}

ProcessCommandResult Get_Parameter_In_Command_SWT(char* string){
	//SML => SET Wait
	ProcessCommandResult pcr = {
		3,"Error",{}
	};
	for(int i=0; i < sizeof(pcr.parameters)/sizeof(pcr.parameters[0]) ; i++){
		pcr.parameters[i] = -1;
	}

	if(state != AS){
		pcr.command_number = -1;
		strcpy(pcr.message,"Admin is not logged\n");
		return pcr;
	}

	char copy_string[strlen(string)];
	strcpy(copy_string,string);
	char* command_string_parts[3] = {"","",""};
	int i = 0;
	char* res = strtok(copy_string," ");
	while(res != NULL && i < 3){
		//Set Wait 200 => if we split string with space we have 3 substrings.
		command_string_parts[i] = res;
		i += 1;
		res = strtok(NULL, " ");
	}
	if(
			res != NULL ||
			(
				strcmp(command_string_parts[0],"SET") !=0
				|| strcmp(command_string_parts[1],"WAIT") !=0
			)
	){
		pcr.command_number = -1;
		strcpy(pcr.message , "Command is Wrong!!!\n");
		return pcr;
	}
	int string_wt_len = strlen(command_string_parts[2]);
	char string_wt[string_wt_len];
	int number_wt[string_wt_len];
	strcpy(string_wt,command_string_parts[2]);
	for(int j=0;j<string_wt_len;j++){
		int ascii_char = (int) (string_wt[j]);
		if(ascii_char < 48 || ascii_char > 57){
			pcr.command_number = -1;
			strcpy(pcr.message , "Command parameter is Wrong!!!\n");
			return pcr;
		}else{
			number_wt[j] = ascii_char - 48;
		}
	}
	int sum = 0;
	for(int j=0;j<string_wt_len;j++){
		int tavan = string_wt_len - 1 - j;
		sum += pow(10,tavan) * number_wt[j];
	}
	if(number_wt[string_wt_len-1] !=0 || number_wt[string_wt_len-2] !=0){
		pcr.command_number = -1;
		char msg[50];
		sprintf(msg,"Command parameter should be multiple of 100, while time is %d\n",sum);
		strcpy(pcr.message,msg);
		return pcr;
	}
	if(sum < 500 || sum > 5000){
		pcr.command_number = -1;
		char msg[50];
		sprintf(msg,"Command parameter should be in range [500-5000], while time is %d\n",sum);
		strcpy(pcr.message,msg);
		return pcr;
	}
	char msg[50];
	sprintf(msg,"wait time changed to %d\n",sum);
	strcpy(pcr.message,msg);
	pcr.parameters[0] = sum;
	return pcr;
}
ProcessCommandResult Get_Parameter_In_Command_TEST(char* string){

	ProcessCommandResult pcr = {
		5,"Error",{}
	};
	for(int i=0; i < sizeof(pcr.parameters)/sizeof(pcr.parameters[0]) ; i++){
		pcr.parameters[i] = -1;
	}

	if(state != AS){
		pcr.command_number = -1;
		strcpy(pcr.message,"Admin is not logged\n");
		return pcr;
	}
	char copy_string[strlen(string)];
	strcpy(copy_string,string);
	char* command_string_parts[2] = {"",""};
	int i = 0;
	char* res = strtok(copy_string,"#");
	while(res != NULL && i < 2){
		command_string_parts[i]=res;
		i += 1;
		res = strtok(NULL, " ");
	}
	if(
			res != NULL ||
			(
				strcmp(command_string_parts[0],"TEST") !=0
			)

	){
		strcpy(pcr.message , "Command is Wrong!!!");
		pcr.command_number=-1;
		return pcr;
	}
	int string_queue_len = strlen(command_string_parts[1]);
	if(string_queue_len == 0){
		strcpy(pcr.message , "Command hasn't queue for test!!!\n");
		pcr.command_number=-1;
		return pcr;
	}
	char string_queue[string_queue_len];
	strcpy(string_queue,command_string_parts[1]);
	if(string_queue[0] == '\n' || string_queue[0] == ' '){
		strcpy(pcr.message , "Command hasn't queue for test!!!\n");
		pcr.command_number=-1;
		return pcr;
	}
	i = 0;
	while(string_queue[i] != '\n' && i < string_queue_len){
		if(isdigit(string_queue[i])){
			int test_floor = (int) (string_queue[i]) - 48;
			if(test_floor <= max_level){
				pcr.parameters[i] = (int) (string_queue[i]) - 48;
				if(test_floor == level){
					char msg[50];
					sprintf(msg,"You can't enter current floor{%d} in queue test\n",test_floor);
					strcpy(pcr.message , msg);
					pcr.command_number=-1;
					return pcr;
				}
			}else{
				char msg[50];
				sprintf(msg,"level should be in range 0 ,%d but floor:%d is not\n",max_level,test_floor);
				strcpy(pcr.message , msg);
				pcr.command_number=-1;
				return pcr;
			}
		}else{
			strcpy(pcr.message , "All floors in test queue should be a number\n");
			pcr.command_number=-1;
			return pcr;
		}
		i++;
	}

	int is_reapeted_element = 0;
	int reapeted_level = -1;
	for(int k=0;k<string_queue_len;k++){
		for(int j=0;j<string_queue_len;j++){
			if(k != j && pcr.parameters[k] == pcr.parameters[j]){
				is_reapeted_element = 1;
				reapeted_level = pcr.parameters[k];
			}
		}
	}
	if(is_reapeted_element == 1){
		char msg[50];
		sprintf(msg,"floor: %d already exists in queue!!!\n",reapeted_level);
		strcpy(pcr.message , msg);
		pcr.command_number=-1;
		return pcr;
	}
	strcpy(pcr.message,"Test queue filled successfully!!!\n");
	return pcr;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of message occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL message return state */
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
  * @param  line: assert_param message line source number
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
