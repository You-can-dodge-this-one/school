#ifndef MBOT_LINUX_SERIAL_H
#define MBOT_LINUX_SERIAL_H

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>



extern void serialInit();
extern void writeSpeed(double RobotV, double YawRate,unsigned char ctrlFlag);
extern bool readSpeed(double &vx,double &vth,double &th,unsigned char &ctrlFlag,double &battary,float &qx,float &qy,float &qz,float &qw);
unsigned char getCrc8(unsigned char *ptr, unsigned short len);

#endif
