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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "string.h"
#include "mainpp.h"
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
extern float vel_x;
extern float vel_y;
extern float vel_th;
extern float outputPWM[3];

//=====================JOYSTICK RC==================//
extern uint8_t joystick_buf[13];
extern uint8_t joystick_x_buf, joystick_y_buf, joystick_z_buf;
extern int8_t joystick_x, joystick_y, joystick_z;
extern int  joystick_bt_timeout, joystick_bt_counter;
extern int joystick_mode, mode;
extern int joystick_increase_speed, joystick_decrease_speed, speed, lock_increase_speed, lock_decrease_speed;

//====================IMU BNO055=====================//
extern char imu_buf[32];
extern float euler_x;
extern float euler_y;
extern float euler_z;
extern float quat_w;
extern float quat_x;
extern float quat_y;
extern float quat_z;

extern float gyro_angle_w, gyro_offset_w;
extern float gyro_angle_z, gyro_offset_z;

//===============ODOMETRY================//
extern float x_velocity;
extern float y_velocity;
extern float angular_velocity;

//================COMMMUNICATION ROSSERIAL================//
extern uint32_t tick;

/* USER CODE END Variables */
osThreadId joys_imuTaskHandle;
osThreadId rosserialTaskHandle;
osThreadId out_motorTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Startjoys_imuTask(void const * argument);
void StartrosserialTask(void const * argument);
void Startout_motorTask(void const * argument);

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
  /* definition and creation of joys_imuTask */
  osThreadDef(joys_imuTask, Startjoys_imuTask, osPriorityNormal, 0, 128);
  joys_imuTaskHandle = osThreadCreate(osThread(joys_imuTask), NULL);

  /* definition and creation of rosserialTask */
  osThreadDef(rosserialTask, StartrosserialTask, osPriorityNormal, 0, 2048);
  rosserialTaskHandle = osThreadCreate(osThread(rosserialTask), NULL);

  /* definition and creation of out_motorTask */
  osThreadDef(out_motorTask, Startout_motorTask, osPriorityNormal, 0, 128);
  out_motorTaskHandle = osThreadCreate(osThread(out_motorTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Startjoys_imuTask */
/**
  * @brief  Function implementing the joy_imuTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Startjoys_imuTask */
void Startjoys_imuTask(void const * argument)
{
  /* USER CODE BEGIN Startjoys_imuTask */
  /* Infinite loop */
  for(;;)
  {	  //JOYSTICK BLUETOOTH
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

	  if(joystick_buf[0] == 'i' && joystick_buf[1] == 't' && joystick_buf[2] == 's'){
		  HAL_UART_Receive_DMA(&huart2, joystick_buf, sizeof(joystick_buf));

		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);

		  memcpy(&joystick_y_buf, joystick_buf + 3, 1);
		  memcpy(&joystick_x_buf, joystick_buf + 4, 1);
		  memcpy(&joystick_z_buf, joystick_buf + 6, 1);

		  joystick_x_buf = constrain(joystick_x_buf, 0, 246);
		  joystick_y_buf = constrain(joystick_y_buf, 0, 246);
		  joystick_z_buf = constrain(joystick_z_buf, 0, 246);

		  joystick_x = map(joystick_x_buf, 0, 246, -123, 123);
		  joystick_y = map(joystick_y_buf, 0, 246, -123, 123);
		  joystick_z = map(joystick_z_buf, 0, 246, -123, 123);

		  joystick_mode = joystick_buf[7];
		  mode = joystick_buf[8];
		  joystick_increase_speed = joystick_buf[9];
		  joystick_decrease_speed = joystick_buf[10];
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

	  //GYRO BNO055
	  HAL_UART_Receive_DMA(&huart4, imu_buf, sizeof(imu_buf));
	  if(imu_buf[0] == 'i' && imu_buf[1] == 't' && imu_buf[2] == 's'){
		  memcpy(&euler_x, imu_buf + 3, 4);
		  memcpy(&euler_y, imu_buf + 7, 4);
		  memcpy(&euler_z, imu_buf + 11, 4);
		  memcpy(&quat_w, imu_buf + 15, 4);
		  memcpy(&quat_x, imu_buf + 19, 4);
		  memcpy(&quat_y, imu_buf + 23, 4);
		  memcpy(&quat_z, imu_buf + 27, 4);
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);

		  gyro_angle_w = (gyro_offset_w - euler_x);

		  while (gyro_angle_w > 180)
			  gyro_angle_w -= 360;
		  while (gyro_angle_w < -180)
			  gyro_angle_w += 360;

		  gyro_angle_z = (gyro_offset_z - euler_x);
		  while (gyro_angle_z > 180)
			  gyro_angle_z -= 360;
		  while (gyro_angle_z < -180)
			  gyro_angle_z += 360;

		  gyro_angle_w = gyro_angle_w * 0.01745329252;
		  gyro_angle_z = gyro_angle_z * 0.01745329252;

	  }else{
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
	  }

	  //CONTROL PC
	  x_velocity 		= map(vel_x, -1, 1, -123, 123);
	  y_velocity 		= map(vel_y, -1, 1, -123, 123);
	  angular_velocity 	= map(vel_th, -1, 1, -123, 123);

    osDelay(1);
  }
  /* USER CODE END Startjoys_imuTask */
}

/* USER CODE BEGIN Header_StartrosserialTask */
/**
* @brief Function implementing the rosserialTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartrosserialTask */
void StartrosserialTask(void const * argument)
{
  /* USER CODE BEGIN StartrosserialTask */
	setup();
  /* Infinite loop */
  for(;;)
  {
//	  calculate_odometry();
//	  loop();
//    osDelay(20);
	  tick++;
	  loop();
	  osDelay(20);
  }
  /* USER CODE END StartrosserialTask */
}

/* USER CODE BEGIN Header_Startout_motorTask */
/**
* @brief Function implementing the out_motorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Startout_motorTask */
void Startout_motorTask(void const * argument)
{
  /* USER CODE BEGIN Startout_motorTask */
  /* Infinite loop */
  for(;;)
  {
	  //=================CONTROL SPEED==================//
	  	  if(joystick_increase_speed == 1 && lock_increase_speed == 0){
	  		  speed++;
	  		  lock_increase_speed = 1;
	  	  }
	  	  if(joystick_increase_speed == 0 && lock_increase_speed == 1){
	  		  lock_increase_speed = 0;
	  	  }

	  	  if(joystick_decrease_speed == 1 && lock_decrease_speed == 0){
	  		  speed--;
	  		  lock_decrease_speed = 1;
	  	  }
	  	  if(joystick_decrease_speed == 0 && lock_decrease_speed == 1){
	  		  lock_decrease_speed = 0;
	    	  }

	  	  if(speed > 2){
	  		  speed = 0;
	  	  }
	  	  if(speed < 0){
	  		  speed = 2;
	  	  }

	  	  if(speed == 0){
	  		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	  	  }else if(speed == 1){
	  		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	  	  }else if(speed == 2){
	  		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	  	  }

	  	  //=====================SELECT MODE COMM/STM=================//
	  	  if(mode == 1){
	  		motor_VectorKinematic(angular_velocity, y_velocity, x_velocity);
	  	  }
	  	  else if(mode == 0){
	  		motor_VectorKinematic(joystick_z, joystick_y, joystick_x);
	  	  }
	  	  //========================OUTPUT PWM===========================//
	  	  if (outputPWM[1] < 0){
	  		  HAL_GPIO_WritePin(MOTOR1A_GPIO_Port, MOTOR1A_Pin, GPIO_PIN_SET);
	  		  HAL_GPIO_WritePin(MOTOR1B_GPIO_Port, MOTOR1B_Pin, GPIO_PIN_RESET);
	  	 	  }
	  	  else if (outputPWM[1] > 0){
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

	   	  //========================SAFETY CONTROL PWM=====================//
	   	  if(joystick_bt_counter == 299 || joystick_mode == 0){
	   		  outputPWM[0] = 0;
	   		  outputPWM[1] = 0;
	   		  outputPWM[2] = 0;
	   	  }

	   	  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, abs(outputPWM[1]));
	   	  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, abs(outputPWM[2]));
	   	  __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, abs(outputPWM[0]));

	   	  osDelay(1);
  }
  /* USER CODE END Startout_motorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
