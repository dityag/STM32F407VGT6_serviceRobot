/*
 * control_motor.h
 *
 *  Created on: Apr 8, 2023
 *      Author: HP GAMING
 */

#ifndef INC_CONTROL_MOTOR_H_
#define INC_CONTROL_MOTOR_H_

#include "main.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"

//#define PWM1 TIM9->CCR1
//#define PWM2 TIM9->CCR2
//#define PWM0 TIM12->CCR2
//#define encoder[0] TIM2->CNT
//#define encoder[1] TIM3->CNT
//#define encoder[2] TIM4->CNT
//#define KP_motor 1
//#define KI_motor 0.145
//#define KD_motor 20
//#define PID_dt 1

#define PG45_GEAR_REDUCTION 27.2
#define PG45_ENCODER_PPR 200
//#define output_max_velo_motor 999
//#define output_min_velo_motor -999
//#define integral_output_max 999
//#define integral_output_min -999
//#define derivative_output_max 999
//#define derivative_output_min -999
//#define sum_error_velo_motor_max 999
//#define sum_error_velo_motor_min -999

void motor_VectorKinematic(short int vx, short int vy, short int vsudut);
void motor_VeloControl(void);
short calculatePID(short setpoint, float actual_value, float kp, float ki, float kd, short maxval, short minval,float errornow, float errorbfr, float max_err, float min_err, float dt);

float map(float x, float in_min, float in_max, float out_min, float out_max);

//void motor_SetPWM(short int outputPWM[3]);

#ifdef constrain
#undef constrain
#endif

#ifndef constrain
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

#endif /* INC_CONTROL_MOTOR_H_ */
