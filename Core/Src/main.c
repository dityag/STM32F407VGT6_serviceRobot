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
#include "cmsis_os.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include <control_motor.h>
#include "mainpp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//==================CONTROL MOTOR================//
float KP_motor = 3 ;
float KI_motor = 0 ;
float KD_motor = 0 ;
float PID_dt = 1 ;

float vel_x;
float vel_y;
float vel_th;

//================CONTROL MOTOR MODE RC================//
int encoder[3];
short int motor_velo[3];
short int motor_SetPoint[3];
float proportional_motor[3], integral_motor[3], derivative_motor[3];
float prev_enc[3], error_velo_motor[3], previous_error_velo_motor[3];
float outputPWM[3];

//=====================JOYSTICK RC==================//
uint8_t joystick_buf[13];
uint8_t joystick_x_buf, joystick_y_buf, joystick_z_buf;
int8_t joystick_x, joystick_y, joystick_z;
int joystick_bt_timeout = 300, joystick_bt_counter = 0;
int joystick_mode, mode;
int joystick_increase_speed, joystick_decrease_speed, speed, lock_increase_speed, lock_decrease_speed;

//====================IMU BNO055=====================//
char imu_buf[32];
float euler_x;
float euler_y;
float euler_z;
float quat_w;
float quat_x;
float quat_y;
float quat_z;

//====================ODOMETRY=========================//
int odometry[2] = {32768, 32768};
float x_buffer_position = 0, y_buffer_position = 0;
float x_offset_position = 0, y_offset_position = 0;
float x_position = 0, y_position = 0;

float gyro_buffer, gyro_offset = 90;
float gyro_angle = 90, gyro_radian = 1.5707963268; // 1/2*phi

float x_velocity;
float y_velocity;
float angular_velocity;

//================COMMMUNICATION ROSSERIAL================//
unsigned long int t0_ros;
unsigned long int t1_ros;

uint32_t tick;
uint32_t test;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		encoder[0] = __HAL_TIM_GET_COUNTER(&htim2);
	}
	if(htim->Instance == TIM3){
		encoder[1] = __HAL_TIM_GET_COUNTER(&htim3);
	}
	if(htim->Instance == TIM4){
		encoder[2] = __HAL_TIM_GET_COUNTER(&htim4);
	}

	if(htim->Instance == TIM1){
		odometry[0] = __HAL_TIM_GET_COUNTER(&htim1);
	}
	if(htim->Instance == TIM8){
		odometry[1] = __HAL_TIM_GET_COUNTER(&htim8);
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
///  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM9_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_TIM12_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  MX_TIM13_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim13);

  //============ACTUATOR MOTOR============
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);//M1
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);//M2
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);//M3

  //=============EXTERNAL PWM=============
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);//EXTERNAL PWM

  //=============ENCODER MOTOR============
  HAL_TIM_Encoder_Start_IT(&htim2,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim3,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim4,TIM_CHANNEL_ALL);

  HAL_TIM_Encoder_Start_IT(&htim1,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim8,TIM_CHANNEL_ALL);
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM7){
		motor_VeloControl();
	}
	if (htim->Instance == TIM13){
//		motor_VectorKinematic(joystick_x, joystick_y, joystick_z);
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
