/*
 * odometry.c
 *
 *  Created on: Mar 5, 2024
 *      Author: HP GAMING
 */

#include "odometry.h"

//===============ODOMETRY================//
extern int odometry[2];
extern float x_buffer_position, y_buffer_position;
extern float x_offset_position, y_offset_position;
extern float x_position, y_position;

extern float gyro_buffer, gyro_offset;
extern float gyro_angle, gyro_radian;

//====================IMU BNO055=====================//
extern float euler_x;

void calculate_odometry(void)
{
	//gyro offset dan x y offset didapat dari callback ros

	short int odometry0_speed = odometry[0];
	odometry[0] = 0;
	short int odometry1_speed = odometry[1];
	odometry[1] = 0;

	float buffer_x[2];
	float buffer_y[2];

	gyro_buffer = euler_x;

	gyro_angle = (gyro_offset - gyro_buffer);
	gyro_radian = (gyro_offset - gyro_buffer) * 0.01745329252;

	// 1/2 * phi
	while (gyro_angle > 180){
		gyro_angle -= 360;
	}
	while (gyro_angle < -180){
		gyro_angle += 360;
	}

	while (gyro_radian > 3.14159265359){
		gyro_radian -= 6.28318530718;
	}
	while (gyro_radian < -3.14159265359){
		gyro_radian += 6.28318530718;
	}

	buffer_x[0] = odometry0_speed * cosf(gyro_radian + 0.785398);
	buffer_x[1] = odometry1_speed * cosf(gyro_radian + 2.356190);

	buffer_y[0] = odometry0_speed * sinf(gyro_radian + 0.785398);
	buffer_y[1] = odometry1_speed * sinf(gyro_radian + 2.356190);

	x_buffer_position += (buffer_x[0] + buffer_x[1]) * odometry_to_cm;
	y_buffer_position -= (buffer_y[0] + buffer_y[1]) * odometry_to_cm;

	x_position = x_buffer_position - x_offset_position;
	y_position = y_buffer_position - y_offset_position;
}
