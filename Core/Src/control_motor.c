/*
 * control_motor.c
 *
 *  Created on: Apr 8, 2023
 *      Author: HP GAMING
 */


#include <control_motor.h>

//================CONTROL MOTOR MODE RC================//
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
	motor_SetPoint[0] = (short int) (((vx * 0.8) * cosf(0 * M_PI/180)) + (vsudut));
	motor_SetPoint[1] = (short int) (((vx) * cosf(240 * M_PI/180)) + ((vy * 1.21) * sinf(240 * M_PI/180)) + (vsudut));
	motor_SetPoint[2] = (short int) (((vx) * cosf(120 * M_PI/180)) + ((vy * 1.20) * sinf(120 * M_PI/180)) + (vsudut));
}

void motor_VeloControl(void)
{

	motor_velo[0] = (encoder[0] - prev_enc[0]);
	prev_enc[0] = encoder[0];
	motor_velo[1] = (encoder[1] - prev_enc[1]);
	prev_enc[1] = encoder[1];
	motor_velo[2] = (encoder[2] - prev_enc[2]);
	prev_enc[2] = encoder[2];

	for(int i = 0; i < 3; i++){
//		motor_velo[i] = (encoder[i] - prev_enc[i]);
		error_velo_motor[i] = motor_SetPoint[i] - motor_velo[i];
		proportional_motor[i] = KP_motor * error_velo_motor[i];
		integral_motor[i] += error_velo_motor[i];
//		derivative_motor[i] = KD_motor * (error_velo_motor[i] - previous_error_velo_motor[i]) / PID_dt;
		derivative_motor[i] = KD_motor * (error_velo_motor[i] - previous_error_velo_motor[i]);

//		integral_motor[i] = KI_motor * (integral_motor[i] + error_velo_motor[i] * PID_dt);
		previous_error_velo_motor[i] = error_velo_motor[i];

		if(integral_motor[i] > 999) integral_motor[i] = 999;
		else if(integral_motor[i] < -999) integral_motor[i] = -999;

//		outputPWM[i] = (proportional_motor[i] + integral_motor[i] + derivative_motor[i]) * 0.9;
		outputPWM[i] = proportional_motor[i] * 0.3;

		if(outputPWM[i] > 999) outputPWM[i] = 999;
		else if(outputPWM[i] < -999) outputPWM[i] = -999;
//		prev_enc[i] = encoder[i];
//		previous_error_velo_motor[i] = error_velo_motor[i];
	}
}


