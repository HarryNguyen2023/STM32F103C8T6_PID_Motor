#include "main.h"
#include <stdio.h>
#include <string.h>

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);

// Define command to control the motor speed and position
#define Dir '0'
#define Speed1 '1'
#define Speed2 '2'
#define Speed3 '3'
#define Angle45 '4'
#define Angle60 '5'
#define Angle90 '6'
#define Angle180 '7'
#define Angle270 '8'
#define Angle360 '9'

// Define pins connect to the motor
#define M_PORT GPIOB
#define DC1 GPIO_PIN_13
#define DC2 GPIO_PIN_14

// Global variables
#define MAX_DC 999				// Maximum duty cycle value can be input into the PWM
#define PPR 374					// Number of pulse per revolution of the motor encoder
#define PI 3.1416
#define RES (2 * PI) / PPR		// Resolution in rad of each pulse
#define RAD PI / 180

char rcvMsg[25] = {'\0'};
uint8_t command;
uint8_t count = 0;

// PID control parameters
float Kp = 0;
float Ki = 0;
float Kd = 0;
volatile float intError = 0;
volatile float devError = 0;
float targetPos = 0;
float currentPos = 0;
float nowError = 0;
float prevError = 0;
const float deltaTime = 0.01;

// Function to control the speed of the motor
void SpeedCtrl(float dutyCycle)
{
	if(dutyCycle <= 1)
	{
		uint32_t speed = (uint32_t)(MAX_DC * dutyCycle);
		TIM4->CCR1 = speed;
	}
}

// Function to stop the motor immediately
void MotorStop(void)
{
	HAL_GPIO_WritePin(M_PORT, DC1, 0);
	HAL_GPIO_WritePin(M_PORT, DC2, 0);
	// End PWM pulse
	SpeedCtrl(0);
}

// Function to start the motor
void MotorStart(void)
{
	HAL_GPIO_WritePin(M_PORT, DC1, 1);
    HAL_GPIO_WritePin(M_PORT, DC2, 0);
}

// Function to handle command for position control of the motor
void PositionCommand(uint8_t comm)
{
	// Stop the motor
	MotorStop();
	// Reset the current encoder position
	currentPos = (TIM3->CNT) * RAD;
	// Execute the command
	if(comm == Angle45)
	{
		targetPos = currentPos + 45 * RAD;
		// Input PID parameters
		Kp = 0.4;
		Ki = 0;
		Kd = 0;
	}
	else if(comm == Angle60)
	{
		targetPos = currentPos + 60 * RAD;
		// Input PID parameters
		Kp = 0.35;
		Ki = 0;
		Kd = 0;
	}
	else if(comm == Angle90)
	{
		targetPos = currentPos + 90 * RAD;
		// Input PID parameters
		Kp = 0.2;
		Ki = 0.005;
		Kd = 0;
	}
	else if(comm == Angle180)
	{
		targetPos = currentPos + 180 * RAD;
		// Input PID parameters
		Kp = 0.1;
		Ki = 0.005;
		Kd = 0;
	}
	else if(comm == Angle270)
	{
		targetPos = currentPos + 270 * RAD;
		// Input PID parameters
		Kp = 0.055;
		Ki = 0.008;
		Kd = 0;
	}
	else if(comm == Angle360)
	{
		targetPos = currentPos + 360 * RAD;
		// Input PID parameters
		Kp = 0.035;
		Ki = 0.008;
		Kd = 0;
	}
	// Start the TIMER 2 timer mode
	HAL_TIM_Base_Start_IT(&htim2);
}

// Function to get the control signal of PID controller
float PIDControl()
{
	float controlSignal = 0;
	currentPos = (TIM3->CNT) * RAD;
	// Get the error value
	nowError = targetPos - currentPos;
	// Get the integral of error using trapezoidal rule
	intError += deltaTime * ((nowError + prevError)/2.0);
	// Get the derivative of error
	devError = (nowError - prevError)/deltaTime;
	// Calculate the PID control value
	controlSignal = (Kp*nowError) + (Ki*intError) + (Kd*devError);
	// Assign the new error
	prevError = nowError;

	return controlSignal;
}

// Function to control the position of the motor shaft
void PositionCtrl()
{
	// Get the PID control signal
	float ctrlSig = PIDControl();
	// Check if control signal is valid
	if(ctrlSig > 1)
	{
		ctrlSig = 1;
	}
	else if(ctrlSig < 0.15 && ctrlSig > 0)
	{
		ctrlSig = 0.15;
	}else if(ctrlSig == 0)
	{
		// Stop the motor
		MotorStop();
		// Stop the timer
		HAL_TIM_Base_Stop(&htim2);
		return;
	}
	// Drive the motor
	HAL_GPIO_WritePin(M_PORT, DC1, 1);
	HAL_GPIO_WritePin(M_PORT, DC2, 0);
	SpeedCtrl(ctrlSig);
}


// Function to execute the command sent by UART
void Command_Handling(uint8_t com)
{
	if(com == Dir)
	{
		HAL_GPIO_TogglePin(M_PORT, DC1);
		HAL_GPIO_TogglePin(M_PORT, DC2);
		// Send feedback
		sprintf(rcvMsg, "Direction changed\r\n");

	}
	else if(com == Speed1)
	{
		MotorStart();
		SpeedCtrl(0.5);
		// Send feedback
		sprintf(rcvMsg, "Speed 1 running\r\n");
	}
	else if(com == Speed2)
	{
		MotorStart();
		SpeedCtrl(0.75);
		// Send feedback
		sprintf(rcvMsg, "Speed 2 running\r\n");
	}
	else if(com == Speed3)
	{
		MotorStart();
		SpeedCtrl(1);
		// Send feedback
		sprintf(rcvMsg, "Speed 3 running\r\n");
	}
	else if(com == Angle45 || com == Angle60 || com == Angle90 || com == Angle180 || com == Angle270 || com == Angle360)
	{
		sprintf(rcvMsg, "Position control mode\r\n");
		PositionCommand(command);
	}
	else
	{
		sprintf(rcvMsg, "Invalid command\r\n");
	}
	HAL_UART_Transmit(&huart1, rcvMsg, strlen(rcvMsg), 100);
}

// Call back function when receive message on UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// Receive the data and clear the flag bit
	HAL_UART_Receive_IT(huart, &command, 1);
	// Check the received command
	sprintf(rcvMsg,"Received : %c\r\n", command);
	HAL_UART_Transmit(&huart1, rcvMsg, strlen(rcvMsg), 100);
	// Handling the command
	Command_Handling(command);
}

int main(void)
{
  // System initialization
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();

  // Configure the UART reception location
  HAL_UART_Receive_IT(&huart1, &command, 1);
  // Configure the PWM mode on TIMER 4
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  // Configure the Encoder mode on TIMER 3
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  // Initiate the motor
  MotorStart();
  SpeedCtrl(0.3);

  while (1)
  {
  }
  return 0;
}

// Function to handle callback of timer interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	PositionCtrl();
	if(++count == 100)
	{
		count = 0;
		sprintf(rcvMsg,"Pulse : %d\r\n", TIM3->CNT);
	    HAL_UART_Transmit(&huart1, rcvMsg, strlen(rcvMsg), 100);
	}
}

// Function to handle external interrupt service
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// Immediately stop the motor
	if(GPIO_Pin == GPIO_PIN_15)
	{
		MotorStop();
		// Send feedback
		sprintf(rcvMsg, "Motor stopped\r\n");
		HAL_UART_Transmit(&huart1, rcvMsg, strlen(rcvMsg), 100);
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 720;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 35;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
