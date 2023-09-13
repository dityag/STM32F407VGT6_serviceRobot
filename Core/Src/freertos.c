/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "string.h"
#include <control_motor.h>
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
/* USER CODE BEGIN Variables */
//==================CONTROL MOTOR================//
//extern int encoder[3];
//extern short int motor_velo[3];
//extern short int motor_SetPoint[3];
//extern float proportional_motor[3], integral_motor[3], derivative_motor[3];
//extern float prev_enc[3], error_velo_motor[3], previous_error_velo_motor[3];
extern float outputPWM[3];
extern short int motor_SetPoint[3];

//=====================JOYSTICK RC==================//
extern uint8_t joystick_buf[14];
extern uint8_t joystick_x_buf, joystick_y_buf, joystick_z_buf;
extern int8_t joystick_x, joystick_y, joystick_z;
//extern int i;
extern int  joystick_bt_timeout, joystick_bt_counter;

//================CONTROL MOTOR MODE RC================//
extern int dummy;
//extern int8_t kecepatan_x;
//extern int8_t kecepatan_y;
//extern int8_t kecepatan_z;

/* USER CODE END Variables */
osThreadId Task01Handle;
osThreadId Task02Handle;
osThreadId Task03Handle;
osThreadId Task04Handle;
osThreadId Task05Handle;
osMutexId Mutex01Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTask01(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);
void StartTask05(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of Mutex01 */
  osMutexDef(Mutex01);
  Mutex01Handle = osMutexCreate(osMutex(Mutex01));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Task01 */
  osThreadDef(Task01, StartTask01, osPriorityNormal, 0, 1024);
  Task01Handle = osThreadCreate(osThread(Task01), NULL);

  /* definition and creation of Task02 */
  osThreadDef(Task02, StartTask02, osPriorityNormal, 0, 128);
  Task02Handle = osThreadCreate(osThread(Task02), NULL);

  /* definition and creation of Task03 */
  osThreadDef(Task03, StartTask03, osPriorityLow, 0, 128);
  Task03Handle = osThreadCreate(osThread(Task03), NULL);

  /* definition and creation of Task04 */
  osThreadDef(Task04, StartTask04, osPriorityLow, 0, 128);
  Task04Handle = osThreadCreate(osThread(Task04), NULL);

  /* definition and creation of Task05 */
  osThreadDef(Task05, StartTask05, osPriorityLow, 0, 128);
  Task05Handle = osThreadCreate(osThread(Task05), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartTask01 */
/**
  * @brief  Function implementing the Task01 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask01 */
void StartTask01(void const * argument)
{
  /* USER CODE BEGIN StartTask01 */
  /* Infinite loop */
  for(;;)
  {
	  if(joystick_bt_counter < joystick_bt_timeout){
		  if(HAL_UART_Receive_DMA(&huart2, joystick_buf, sizeof(joystick_buf)) != HAL_OK){
			  joystick_bt_counter++;
		  }
		  else{
			  joystick_bt_counter = 0;
		  }
	  }

	  if(joystick_bt_counter >= 299){
		  joystick_bt_counter = 299;
	  }

	  if(joystick_buf[0] == 'E' && joystick_buf[1] == 'L' && joystick_buf[2] == 'K' && joystick_buf[3] == 'A'){
		  HAL_UART_Receive_DMA(&huart2, joystick_buf, sizeof(joystick_buf));

		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);

		  memcpy(&joystick_y_buf, joystick_buf + 4, 1);
		  memcpy(&joystick_x_buf, joystick_buf + 5, 1);
		  memcpy(&joystick_z_buf, joystick_buf + 7, 1);

		  joystick_x_buf = constrain(joystick_x_buf, 0, 246);
		  joystick_y_buf = constrain(joystick_y_buf, 0, 246);
		  joystick_z_buf = constrain(joystick_z_buf, 0, 246);

		  joystick_x = map(joystick_x_buf, 0, 246, -123, 123);
		  joystick_y = map(joystick_y_buf, 0, 246, -123, 123);
		  joystick_z = map(joystick_z_buf, 0, 246, -123, 123);
	  }
	  else{
		  HAL_UART_Receive_DMA(&huart2, joystick_buf, sizeof(joystick_buf));

		  joystick_x = 0;
		  joystick_y = 0;
		  joystick_z = 0;
		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	  }

	  if(joystick_bt_counter == 299){
		  joystick_x = 0;
		  joystick_y = 0;
		  joystick_z = 0;
		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	  }
    osDelay(1);
  }
  /* USER CODE END StartTask01 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Task02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
	  //==============================================================================
	  if (outputPWM[1] < 0){
		  dummy = 0;
		  HAL_GPIO_WritePin(MOTOR1A_GPIO_Port, MOTOR1A_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MOTOR1B_GPIO_Port, MOTOR1B_Pin, GPIO_PIN_RESET);
	  }
	  else if (outputPWM[1] > 0){
		  dummy = 1;
		  HAL_GPIO_WritePin(MOTOR1A_GPIO_Port, MOTOR1A_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MOTOR1B_GPIO_Port, MOTOR1B_Pin, GPIO_PIN_SET);
	  }
	  //==============================================================================
	  if (outputPWM[2] < 0){
		  HAL_GPIO_WritePin(MOTOR2A_GPIO_Port, MOTOR2A_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MOTOR2B_GPIO_Port, MOTOR2B_Pin, GPIO_PIN_RESET);
	  }
	  else if (outputPWM[2] > 0){
		  HAL_GPIO_WritePin(MOTOR2A_GPIO_Port, MOTOR2A_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MOTOR2B_GPIO_Port, MOTOR2B_Pin, GPIO_PIN_SET);
	  }
	  //==============================================================================
	  if (outputPWM[0] < 0){
		  HAL_GPIO_WritePin(MOTOR3A_GPIO_Port, MOTOR3A_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MOTOR3B_GPIO_Port, MOTOR3B_Pin, GPIO_PIN_RESET);
	  }
	  else if (outputPWM[0] > 0)
	  {
		  HAL_GPIO_WritePin(MOTOR3A_GPIO_Port, MOTOR3A_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MOTOR3B_GPIO_Port, MOTOR3B_Pin, GPIO_PIN_SET);
	  }

	  if(joystick_bt_counter == 299){
		  outputPWM[0] = 0;
		  outputPWM[1] = 0;
		  outputPWM[2] = 0;
	  }

	  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, abs(motor_SetPoint[1]));
	  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, abs(motor_SetPoint[2]));
	  __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, abs(motor_SetPoint[0]));
	  osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the Task03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
	osDelay(1000);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
	osDelay(1000);
    osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the Task04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void const * argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the Task05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void const * argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
	  osDelay(100);
	  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
	  osDelay(500);
    osDelay(1);
  }
  /* USER CODE END StartTask05 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
