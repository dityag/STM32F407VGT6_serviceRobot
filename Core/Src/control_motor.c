/*
 * control_motor.c
 *
 *  Created on: Apr 8, 2023
 *      Author: HP GAMING
 */


#include <control_motor.h>

//================CONTROL MOTOR MODE RC================//
//extern int8_t kecepatan_x;
//extern int8_t kecepatan_y;
//extern int8_t kecepatan_z;

extern float KP_motor;
extern float KI_motor;
extern float KD_motor;
extern float PID_dt;

extern int encoder[3];
extern short int motor_velo[3];
extern short int motor_SetPoint[3];
extern float proportional_motor[3], integral_motor[3], derivative_motor[3];
extern float prev_enc[3], error_velo_motor[3], previous_error_velo_motor[3];
extern float outputPWM[3];

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void motor_VectorKinematic(short int vx, short int vy, short int vsudut)
{
	motor_SetPoint[0] = (short int) (((vx*-1) * cosf(0 * M_PI/180)) + ((vy*1.1) * sinf(0 * M_PI/180)) + (vsudut*-0.72));
	motor_SetPoint[1] = (short int) (((vx*-1) * cosf(240 * M_PI/180)) + ((vy*1.1) * sinf(240 * M_PI/180))*1.2 + (vsudut*-0.72));
	motor_SetPoint[2] = (short int) (((vx*-1) * cosf(120 * M_PI/180)) + ((vy*1.1) * sinf(120 * M_PI/180))*1.2 + (vsudut*-0.72));
	motor_SetPoint[0] = constrain(motor_SetPoint[0], -150, 150);
	motor_SetPoint[1] = constrain(motor_SetPoint[1], -150, 150);
	motor_SetPoint[2] = constrain(motor_SetPoint[2], -150, 150);
}

void motor_VeloControl(void)
{
	for(int i = 0; i < 3; i++){
		motor_velo[i] = (encoder[i] - prev_enc[i]);
		error_velo_motor[i] = motor_SetPoint[i] - motor_velo[i];

		proportional_motor[i] = KP_motor * error_velo_motor[i];
		integral_motor[i] += error_velo_motor[i];
		derivative_motor[i] = KD_motor * (error_velo_motor[i] - previous_error_velo_motor[i]) / PID_dt;

		integral_motor[i] = KI_motor * (integral_motor[i] + error_velo_motor[i] * PID_dt);

		if(integral_motor[i] > 30) integral_motor[i] = 30;
		else if(integral_motor[i] < -30) integral_motor[i] = -30;

		outputPWM[i] = proportional_motor[i] + integral_motor[i] + derivative_motor[i];

		if(outputPWM[i] > 150) outputPWM[i] = 150;
		else if(outputPWM[i] < -150) outputPWM[i] = -150;
		prev_enc[i] = encoder[i];
		previous_error_velo_motor[i] = error_velo_motor[i];
	}

	//==================

//	sum_error_velo_motor[0]=0;
//	sum_error_velo_motor[1]=0;
//	sum_error_velo_motor[2]=0;
//	motor_velo[0] = Encoder0;
//	motor_velo[1] = Encoder1;
//	motor_velo[2] = Encoder2;
//	Encoder0=Encoder1=Encoder2=0;
//
//	for(int i = 0; i < 3; i++)
//	{
//		error_velo_motor[i] = motor_SetPoint[i] - motor_velo[i];
//		sum_error_velo_motor[i] += error_velo_motor[i];
//		sum_error_velo_motor[i] = constrain(sum_error_velo_motor[i], sum_error_velo_motor_max, sum_error_velo_motor_min);
//
//		// if(integral_motor[i] > integral_output_max) integral_motor[i] = integral_output_max;
//		// else if(integral_motor[i] < integral_output_min) integral_motor[i] = integral_output_min;
//		//
//		// if(derivative_motor[i] > derivative_output_max) derivative_motor[i] = derivative_output_max;
//		// else if(derivative_motor[i] < derivative_output_min) derivative_motor[i] = derivative_output_min;
//
//		integral_motor[i] = constrain(integral_motor[i], integral_output_max, integral_output_min);
//		derivative_motor[i] = constrain(derivative_motor[i], derivative_output_max, derivative_output_min);
//		proportional_motor[i] = KP_motor * error_velo_motor[i];
//		integral_motor[i] = KI_motor * sum_error_velo_motor[i];
//		derivative_motor[i] = KD_motor * (error_velo_motor[i] - previous_error_velo_motor[i]);
//
//		// if(derivative_motor[i] > derivative_output_max) derivative_motor[i] = derivative_output_max-1;
//		// else if(derivative_motor[i] < derivative_output_min) derivative_motor[i] = derivative_output_min+1;
//
//		previous_error_velo_motor[i] = error_velo_motor[i];
//		outputPWM[i] = (short int) proportional_motor[i] + integral_motor[i] + derivative_motor[i];
//		if(outputPWM[i] > output_max_velo_motor) outputPWM[i] = output_max_velo_motor;
//		else if(outputPWM[i] < output_min_velo_motor) outputPWM[i] = output_min_velo_motor;
//	}
////==============================================================================
//	if (outputPWM[0] < 0)
//	{
//		HAL_GPIO_WritePin(MOTOR1A_GPIO_Port, MOTOR1A_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(MOTOR1B_GPIO_Port, MOTOR1B_Pin, GPIO_PIN_RESET);
//	}
//	else if (outputPWM[0] > 0)
//	{
//		HAL_GPIO_WritePin(MOTOR1A_GPIO_Port, MOTOR1A_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(MOTOR1B_GPIO_Port, MOTOR1B_Pin, GPIO_PIN_SET);
//	}
////==============================================================================
//	if (outputPWM[1] < 0)
//	{
//		HAL_GPIO_WritePin(MOTOR2A_GPIO_Port, MOTOR2A_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(MOTOR2B_GPIO_Port, MOTOR2B_Pin, GPIO_PIN_RESET);
//	}
//	else if (outputPWM[1] > 0)
//	{
//		HAL_GPIO_WritePin(MOTOR2A_GPIO_Port, MOTOR2A_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(MOTOR2B_GPIO_Port, MOTOR2B_Pin, GPIO_PIN_SET);
//	}
////==============================================================================
//	if (outputPWM[2] < 0)
//	{
//		HAL_GPIO_WritePin(MOTOR3A_GPIO_Port, MOTOR3A_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(MOTOR3B_GPIO_Port, MOTOR3B_Pin, GPIO_PIN_RESET);
//	}
//	else if (outputPWM[2] > 0)
//	{
//		HAL_GPIO_WritePin(MOTOR3A_GPIO_Port, MOTOR3A_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(MOTOR3B_GPIO_Port, MOTOR3B_Pin, GPIO_PIN_SET);
//	}
//==============================================================================================
//	PWM0 = abs(outputPWM[0]);
//	PWM1 = abs(outputPWM[1]);
//	PWM2 = abs(outputPWM[2]);
//=========================
//	for(int i = 0; i < 3; i++){
//
//		motor_velo[i] = encoder[i] - prev_enc[i];
//		prev_enc[i] = motor_velo[i];
//
//		error_velo_motor[i] = motor_SetPoint[i] - motor_velo[i];
//
//		integral_motor[i] += error_velo_motor[i];
//		derivative_motor[i] = (error_velo_motor[i] - previous_error_velo_motor[i]) / 0.001;
//
//		integral_motor[i] = integral_motor[i] + error_velo_motor[i] * 0.001;
//
//		if(integral_motor[i] > 50) integral_motor[i] = 50;
//		else if(integral_motor[i] < -50) integral_motor[i] = -50;
//
//		outputPWM[i] = KP_motor*error_velo_motor[i] + KI_motor*integral_motor[i] + KD_motor*derivative_motor[i];
//
//		if(outputPWM[i] > 120) outputPWM[i] = 120;
//		else if(outputPWM[i] < -120) outputPWM[i] = -120;
//
//		previous_error_velo_motor[i] = error_velo_motor[i];
//
//	}
}


