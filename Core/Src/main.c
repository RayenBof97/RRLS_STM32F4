/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Testing Robost Recursive Least Square with DSP CMSIS Library
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define ARM_MATH_CM4
#include <arm_math.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* Kalman Filter Configuration */
#define DT          0.001f     		// Time step dt = 1(ms)
#define LAMBDA_MAX 	0.98f			// Maximum Forgetting Value
#define LAMBDA_MIN 	0.90f			// Minimum Forgetting Value
#define DELTA 		0.01f			// Sensitivity tuning parameter

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim10;

/* USER CODE BEGIN PV */


// RRLS Algorithm specific
static arm_matrix_instance_f32 mat_x; // Regression vector (2x1)
static arm_matrix_instance_f32 mat_xt; //Transoise of the regression vector (2x1)
static arm_matrix_instance_f32 mat_theta; // Coefficients vector (2x1)
static arm_matrix_instance_f32 mat_K;	// Gain Vector (2x1)
static arm_matrix_instance_f32 mat_P;	// Covariance Matrix (2x2)
static arm_matrix_instance_f32 mat_ypred; // y^ Prediction (Scalar (1x1))
static arm_matrix_instance_f32 mat_I; // I matrix (2x2)


//Temp Matrix structures
static arm_matrix_instance_f32 mat_temp1_2x1;
static arm_matrix_instance_f32 mat_temp2_2x1;
static arm_matrix_instance_f32 mat_temp1_2x2;
static arm_matrix_instance_f32 mat_temp2_2x2;
static arm_matrix_instance_f32 mat_temp_1x1;


//Don't forget to declare a static arm matrix here for temp after using them
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM10_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
static void RRLS_init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Essential buffers only - we can reuse these for multiple operations
static float32_t temp1_2x1[2];
static float32_t temp2_2x1[2];
static float32_t temp1_2x2[4];
static float32_t temp2_2x2[4];
static float32_t temp_1x1[1];
static float32_t xt[2];

// Regression vector X (2x1)
float32_t x_state[2] = {0.0f, 0.0f};

// Coefficients  Vector Theta (2x1)
float32_t theta_f32[2] = {
    0.9f, 0.1f
};

// Covariance Matrix P
float32_t P_f32[4] = {
    1000.0f, 0.0f,
	0.0f,    1000.0f
};
// I Matrix
float32_t I_f32[4] = {
		1.0f , 0.0f,
		0.0f , 1.0f
};

// Gain Vector (K)
float32_t K_gain[2];           // Gain K (2x1)
float32_t ypred_f32[1] = {0};
float32_t epsilon_f32[1] ={0};



//Hardware Measurement (Input / Output)
uint8_t u = 0 ; // Input
float32_t y_measure = 0; //Measurement
float32_t lambda_t = 0;

//Timer specifics
uint32_t tim10_freq =10000;
uint32_t ms_counter = 0;
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
  MX_TIM10_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  RRLS_init();
  TIM10->ARR = (DT*(float32_t)tim10_freq)-1 ;
  HAL_TIM_Base_Start_IT(&htim10);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 8400-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void RRLS_init(void)
{

  /* scratch matrices (To be changed depending on the used ones  */
  arm_mat_init_f32(&mat_x, 2,1, x_state);
  arm_mat_init_f32(&mat_xt, 1,2, xt);
  arm_mat_init_f32(&mat_theta, 2,1,theta_f32);
  arm_mat_init_f32(&mat_P, 2,2,P_f32);
  arm_mat_init_f32(&mat_I, 2,2,I_f32);
  arm_mat_init_f32(&mat_K, 2,1,K_gain);
  arm_mat_init_f32(&mat_ypred, 1,1,ypred_f32);
  arm_mat_init_f32(&mat_temp1_2x1, 2,1,temp1_2x1);
  arm_mat_init_f32(&mat_temp2_2x1, 2,1,temp2_2x1);
  arm_mat_init_f32(&mat_temp1_2x2, 2,2,temp1_2x2);
  arm_mat_init_f32(&mat_temp2_2x2, 2,2,temp2_2x2);
  arm_mat_init_f32(&mat_temp_1x1, 1,1,temp_1x1);

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	float32_t temp;
	//Increment MS counter (Increment Every ~1ms)
	ms_counter++;

	//Set the regressin vector X [ [y(k-1)] , [u(k-1)] ]
	 x_state[0] = y_measure;
	 x_state[1] = (float)u*5.00f;

	//Read the output
	 HAL_ADC_Start(&hadc1);
	 HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	 y_measure= (float)HAL_ADC_GetValue(&hadc1) * (3.3f / 4095.0f); // 12-bit to volts
	//Check for input
	if(ms_counter == 5){
		// Toggle Input to the circuit
		ms_counter = 0;
	 	u ^= 1; //Toggle U from 0 to 1
	 	//GPIO Toggle an output please
	 	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
	 }

	//RRLS Predict
	arm_mat_trans_f32(&mat_x,&mat_xt); // Transpose of X (Xt)
	arm_mat_mult_f32(&mat_xt, &mat_theta, &mat_ypred); // ypred = x.theta
	epsilon_f32[0] = y_measure - ypred_f32[0]; //Epsilon = y - y^

	//Forgetting Factor Update
	if (DELTA > 0.0f && fabsf(epsilon_f32[0]) > 1e-6f) {
	    temp = (epsilon_f32[0] * epsilon_f32[0]) / DELTA;
	    lambda_t = LAMBDA_MIN + (LAMBDA_MAX - LAMBDA_MIN) * expf(-temp);
	} else {
	    lambda_t = LAMBDA_MAX; // Default to maximum forgetting
	}

	//Update RRLS (Update K Gain Vector , Parameter Vector, and Covariance Matrix
	arm_mat_mult_f32(&mat_P, &mat_x, &mat_temp1_2x1); //temp1_2x1 = P @ x
	arm_mat_mult_f32(&mat_xt,&mat_temp1_2x1,&mat_temp_1x1); // temp1x1 = x.T @ P @ x
	temp = 1.0f/(lambda_t + temp_1x1[0]); // Temp = 1 /(lambda + x.T @ P @ x)
	arm_mat_scale_f32(&mat_temp1_2x1, temp, &mat_K); // K = P @ x / temp
	arm_mat_scale_f32(&mat_K, epsilon_f32[0], &mat_temp1_2x1); // temp1_2x1 = K * epsilon
	arm_mat_add_f32(&mat_theta, &mat_temp1_2x1, &mat_temp2_2x1); //temp2_2x1 = theta + K * epsilon

	//Affecting Results to theta vector
	theta_f32[0] = temp2_2x1[0];
	theta_f32[1] = temp2_2x1[1];

	arm_mat_mult_f32(&mat_K,&mat_xt,&mat_temp1_2x2); //temp1_2x2 = K * x.T
	arm_mat_sub_f32(&mat_I, &mat_temp1_2x2, &mat_temp2_2x2); // temp2_2x2 = I - K * x.T
	arm_mat_mult_f32(&mat_temp2_2x2, &mat_P, &mat_temp1_2x2); // (I - K * x.T) @ P
	temp = 1.0f/lambda_t;
	arm_mat_scale_f32(&mat_temp1_2x2, temp, &mat_P); // mat_P = (I- K * x.T) @ P
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
