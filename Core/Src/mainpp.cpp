/*
 * mainpp.cpp
 *
 *  Created on: Jan 6, 2024
 *      Author: HP GAMING
 */

#include <mainpp.h>
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

ros::NodeHandle nh;

std_msgs::Int32MultiArray str_msg;

ros::Publisher stm("stm", &str_msg);

extern int mode;
extern short int motor_SetPoint[3];
extern int encoder[3];
extern int encoder_external[2];

//================COMMMUNICATION ROSSERIAL================//
extern int32_t stm_to_comm[50];
extern int32_t comm_to_stm[50];

extern float outputPWM_comm[3];
extern float outputPWM_stm[3];

void messageCb(const std_msgs::Int32MultiArray& incoming_msg )
{
	if (incoming_msg.data){
		for(int i = 0; i <= 50; i++){
			comm_to_stm[i] = incoming_msg.data[i];
		}

		if(comm_to_stm[0] == 'E' && comm_to_stm[1] == 'L' && comm_to_stm[2] == 'K' && comm_to_stm[3] == 'A'){
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_0);
			outputPWM_comm[0] = comm_to_stm[4];
			outputPWM_comm[1] = comm_to_stm[5];
			outputPWM_comm[2] = comm_to_stm[6];
		}
	}
}

ros::Subscriber<std_msgs::Int32MultiArray> sub("/service_robot/comm", &messageCb );

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void setup(void)
{
  nh.initNode();
  nh.advertise(stm);
  nh.subscribe(sub);
}

void loop(void)
{
  str_msg.data = stm_to_comm;
  str_msg.data[0] = 'E';
  str_msg.data[1] = 'L';
  str_msg.data[2] = 'K';
  str_msg.data[3] = 'A';

  str_msg.data[4] = int(outputPWM_stm[0]);
  str_msg.data[5] = int(outputPWM_stm[1]);
  str_msg.data[6] = int(outputPWM_stm[2]);

  str_msg.data[7] = mode;

  str_msg.data[8] = 65535;
  str_msg.data[9] = encoder[1];
  str_msg.data[10] = encoder[2];

  str_msg.data[11] = encoder_external[0];
  str_msg.data[12] = encoder_external[1];

//  memcpy(str_msg.data + 7, &encoder[0], 4);
//  memcpy(str_msg.data + 11, &encoder[1], 4);
//  memcpy(str_msg.data + 15, &encoder[2], 4);
//  memcpy(str_msg.data + 19, &encoder_external[0], 4);
//  memcpy(str_msg.data + 23, &encoder_external[1], 4);
//  memcpy(str_msg.data + 27, &mode, 1);

  str_msg.data_length = 13;

  stm.publish(&str_msg);
  nh.spinOnce();
}
