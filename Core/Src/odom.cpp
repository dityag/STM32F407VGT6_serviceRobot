/*
 * odom.c
 *
 *  Created on: Mar 6, 2024
 *      Author: HP GAMING
 */

#include "odom.h"
#include <math.h>

// Frame names.
char base_link[] = "base_link";
char odom[]      = "odom";

extern float dummy;

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

nav_msgs::Odometry odomMsg;
ros::Publisher     odom_pub(odom, &odomMsg);

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster tfBroadcaster;


Odometer::Odometer(const float metersPerTick, const float base_width) {
  _metersPerTick = metersPerTick;
  _base_width    = base_width;
  _cur_x         = 0.0;
  _cur_y         = 0.0;
  _cur_theta     = 0.0;
}

void Odometer::setupPubs(ros::NodeHandle &nh) {
  nh.advertise(odom_pub);
  tfBroadcaster.init(nh);
}


void Odometer::update_publish(ros::Time current_time, const float odoInterval,
                              const float distLeft, const float distRight) {

  float vel_x;
  float vel_y;
  float vel_theta;

  update_odom(odoInterval, distLeft, distRight, vel_x, vel_theta);
  publish_odom(current_time, vel_x, vel_theta);
  broadcastTf(current_time);
}

void Odometer::update_odom(const float odoInterval, const float distLeft,const float distRight,
                          float& vel_x, float& vel_theta) {

  float dist;
  float d_theta;

  dist = (distRight + distLeft) / 2.0;

  // Check for the special case of driving in a straight line
  // then compute current loation relative to previous location

  // If the difference in distance is under 2mm/Second we call it a straight line
  if (abs(distRight - distLeft) < (0.002 * odoInterval)) {

    // drove in sraight line
    d_theta = 0.0;
    _cur_x += dist * cos(_cur_theta);
    _cur_y += dist * sin(_cur_theta);
  }
  else {

    // The path was along an arc
    d_theta = (distRight - distLeft) / _base_width;
    float r = dist / d_theta;

    _cur_x += r * (sin(d_theta + _cur_theta) - sin(_cur_theta));
    _cur_y -= r * (cos(d_theta + _cur_theta) - cos(_cur_theta));
//    _cur_theta = normalize_angle(_cur_theta + d_theta);
    _cur_theta = gyro_angle_z;
  }

  vel_x     = dist    / odoInterval;
  vel_theta = d_theta / odoInterval;

  return;
}


// This is a possibly improved version of update_odom() that might replace update_odom()
void Odometer::update_kinematics(const float leftDelta, const float rightDelta, float& vel_x, float& vel_theta) {

  float new_x, new_y, new_heading, wd, x, y, heading;
  const float unitsAxisWidth = 0.3;

  // leftDelta and rightDelta = distance that the left and right wheel have moved along
  //  the ground

  if (fabs(leftDelta - rightDelta) < 1.0e-6) {      // basically going straight
    float midDelta = (leftDelta - rightDelta) / 2.0;
    new_x = x + midDelta * cos(heading);
    new_y = y + midDelta * sin(heading);
    new_heading = heading;
  } else {

    // The vehicle is traveling along an arc.  The radius "R" is the distance
    // from the center of the arc to the midpoint of the axel that conects the
    // two wheels
    float R = unitsAxisWidth * (leftDelta + rightDelta) / (2 * (rightDelta - leftDelta)),
          wd = (rightDelta - leftDelta) / unitsAxisWidth;

    new_x = x + R * (sin(wd + heading) - sin(heading));
    new_y = y - R * (cos(wd + heading) + cos(heading));
    new_heading = normalize_angle(heading + wd);
  }
}

void Odometer::publish_odom(ros::Time current_time, const float vx, const float vth) {

    odomMsg.header.stamp          = current_time;
    odomMsg.header.frame_id       = odom;
    odomMsg.child_frame_id        = base_link;

    odomMsg.pose.pose.position.x  = _cur_y;
    odomMsg.pose.pose.position.y  = (_cur_x) * -1;
    odomMsg.pose.pose.position.z  = 0.0;
//    odomMsg.pose.pose.orientation = tf::createQuaternionFromYaw(_cur_theta);
    odomMsg.pose.pose.orientation = tf::createQuaternionFromYaw(gyro_angle_w, gyro_angle_z);

    odomMsg.twist.twist.linear.x  = vx;
    odomMsg.twist.twist.linear.y  = 0;
    odomMsg.twist.twist.angular.z = vth;

    odom_pub.publish(&odomMsg);
  }


  void Odometer::broadcastTf(ros::Time current_time) {
    t.header.stamp            = current_time;
    t.header.frame_id         = odom;
    t.child_frame_id          = base_link;

    t.transform.translation.x = _cur_y;
    t.transform.translation.y = (_cur_x) * -1;
    t.transform.translation.z = 0.0;
//    t.transform.rotation      = tf::createQuaternionFromYaw(-_cur_theta);
    t.transform.rotation      = tf::createQuaternionFromYaw(gyro_angle_w, gyro_angle_z);


    tfBroadcaster.sendTransform(t);
  }


  float Odometer::normalize_angle(float angle) {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }


