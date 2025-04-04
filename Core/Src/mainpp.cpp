/*
 * mainpp.cpp
 *
 *  Created on: Jan 6, 2024
 *      Author: HP GAMING
 */

#include <main.h>
#include <mainpp.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

#include "odom.h"
#include "pid.h"

ros::NodeHandle  nh;

#define ODO_PERIOD 10  // Millis between /tf and /odom publication
#define PID_PERIOD  10  // Millis between each PID calculation

// TODO:  meterPerTick needs to be computed from parameters in setup()
// meter per encoder tick is wheel circumfrence / encoder ticks per wheel revoution
//const float meterPerTick = (0.13 * 3.1415) / (75.0 * 64.0); // Thumper

extern uint32_t tick;

// These global variables are used by the PID library.
// TODO Kp, Ki and Kd should be parameters
double leftSetpoint = 0.0;
double leftInput,  leftOutput;
double rightSetpoint = 0.0;
double rightInput, rightOutput;

extern float vel_x;
extern float vel_y;
extern float vel_th;

const float meterPerTick = (0.058 * M_PI) / 1000;  // Woodie
const float base_width   = 0.44;         // Woodie

long encoderLeftLastValue  = 0L;
long encoderRightLastValue = 0L;

Odometer odo(meterPerTick, base_width);

//double Kp = 60, Ki = 100, Kd = 1;  //  Usable but not respncive enough
double Kp = 40.0, Ki = 0.0, Kd = 0.0;

// Create one PID object for each motor.  The Input and output units
// will be "meters"
PID leftWheelPID( &leftInput,  &leftOutput,  &leftSetpoint,  Kp, Ki, Kd, DIRECT);
PID rightWheelPID(&rightInput, &rightOutput, &rightSetpoint, Kp, Ki, Kd, DIRECT);

// Functions declaratons (silly C-language requirement)
void cmd_velCallback( const geometry_msgs::Twist& toggle_msg);
void broadcastTf();
// DEBUG FOLLOWS
void MotorTest();

// Subscribe to Twist messages on cmd_vel.
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", &cmd_velCallback);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

// This is an Arduino convention.  Place everything that needs to run just
// once in the setup() funtion.  The environment will call setup()

void setup(void) {

  leftWheelPID.SetSampleTime(PID_PERIOD);
  rightWheelPID.SetSampleTime(PID_PERIOD);
  leftWheelPID.SetOutputLimits(-255, 255);
  rightWheelPID.SetOutputLimits(-255, 255);
  leftWheelPID.SetMode(AUTOMATIC);
  rightWheelPID.SetMode(AUTOMATIC);

  // Connect to ROS computer and wait for connection
  nh.initNode();

  // Advertize odometry and transform
  odo.setupPubs(nh);

  // Subscribe to cmd_vel
  nh.subscribe(sub_cmd_vel);

  nh.loginfo("starting...");
}

// These are all used in the loop() below
static long encoderLeftLastValuePid  = 0;
static unsigned long millis_last_LEFT = 0;

static long encoderRightLastValuePid  = 0;
static unsigned long millis_last_RIGHT = 0;

static unsigned long NextPubMillis   = 0;
static long encoderLeftLastValueOdo  = 0;
static long encoderRightLastValueOdo = 0;
static long timeLastOdo              = 0;

// This loop() funtion is an arduino convention.  It is called by the environment
// inside a tight loop and runs forever or until the CPU is reset or powered off.
//

void loop(void) {
  extern int odometry[2];
  float seconds_from_last;
  long  millis_from_last;
  float distLeft;
  float distRight;

  // Three things run here all on their own schedule
  //  1.  The Left Wheel PID controler
  //  2.  The Right Wheel PID controler
  //  3.  The Odometry and TF publisher
  // Each wheel has it's own PID control and might do it's computation at
  // different times.

  // Get encoder values
  long encLeft  = odometry[0];
  long encRight = odometry[1];
  long curMillis = tick;  // capture time when encoders are sampled


  //=========> LEFT PID Controler
  //
  // Figure out how far we have gone in meters from last PID computation
  distLeft  = meterPerTick * float(encLeft  - encoderLeftLastValuePid);

  //figure out how fast the LEFT wheel went, in meters per second
  millis_from_last  = curMillis - millis_last_LEFT;
  seconds_from_last = float(millis_from_last) / 1000.0;
  leftInput  = distLeft  / seconds_from_last;

  // The PID.Compute() method will look at the millis() clock and determine if it is
  // time to calculate new output.   If so it returns true and then we update the
  // motor speed.  Note the motor update speed update rate is independent of the /tf
  // and /odom publication rate.

  if (leftWheelPID.Compute()) {
      encoderLeftLastValuePid = encLeft;
      millis_last_LEFT = curMillis;
    }


  //==========> RIGHT PID Controler
  //
  // Figure out how far we have gone in meters from last PID computation
  distRight = meterPerTick * float(encRight - encoderRightLastValuePid);

  //figure out how fast the RIGHT wheel went, in meters per second
  millis_from_last  = curMillis - millis_last_RIGHT;
  seconds_from_last = float(millis_from_last) / 1000.0;
  rightInput  = distRight  / seconds_from_last;

  // The PID.Compute() method will look at the millis() clock and determine if it is
  // time to calculate new output.   If so it returns true and then we update the
  // motor speed.  Note the motor update speed update rate is independent of the /tf
  // and /odom publication rate.

  if (rightWheelPID.Compute()) {
      encoderRightLastValuePid = encRight;
      millis_last_RIGHT = curMillis;
    }

  //==========> OdometryPublsher
  //
  // Check if it is time to publish /odom and /tf
  if (curMillis >= NextPubMillis) {
    NextPubMillis = curMillis + ODO_PERIOD;

    // Figure out how far we have gone in meters from last PID computation
    distLeft  = meterPerTick * float(encLeft  - encoderLeftLastValueOdo);
    distRight = meterPerTick * float(encRight - encoderRightLastValueOdo);

    // Publish odometry
    float odoInterval = float(curMillis - timeLastOdo) / 1000.0;
    odo.update_publish(nh.now(), odoInterval, distLeft, distRight);

    encoderLeftLastValueOdo  = encLeft;
    encoderRightLastValueOdo = encRight;
    timeLastOdo = curMillis;
    }


  // handle any data movements across the serial interface
  nh.spinOnce();

}

// This funtion is called every time we receive a Twist message.
// We do not send the commanded speed to the wheels.  Rather we set
// thePID loops set point to the commanded sprrd.
void cmd_velCallback(const geometry_msgs::Twist& twist_msg) {

  // We only use two numbers from the Twist message.
  //    linear.x  is the forward speed in meters per second.
  //              (The "x" axis points forward.)
  //    angular.y is the rotation about the z or vertical
  //              axis in radians per second.
  //

  vel_x   = twist_msg.linear.x;
  vel_y   = twist_msg.linear.y;
  vel_th  = twist_msg.angular.z;

  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  // This is a "hack".  It turns ou the motors have a minimum
  // speed because of internal friction.   If the commanded speed is
  // below a threshold we replace the commanded speed with zero.
  // TODO:  Find a better threshold, make it a parameter
//  if (fabs(vel_x)  < 0.001) vel_x  = 0.0;
//  if (fabs(vel_th) < 0.001) vel_th = 0.0;

  // Compute the wheel speeds in meters per second.
//  float left_vel  =  vel_x - (vel_th * base_width / 2.0);
//  float right_vel =  vel_x + (vel_th * base_width / 2.0);

  //char buff[40];
  //snprintf(buff, 100, "CMD_VEL %f, %f", left_vel, right_vel);
  //nh.loginfo(buff);

  // Show the Twist message on the LCD.
  //displayStatus(&vel_x, &vel_th);

  // The PID works in units of meters per second, so no
  // conversion is needed.
//  leftSetpoint  = left_vel;
//  rightSetpoint = right_vel;
}
