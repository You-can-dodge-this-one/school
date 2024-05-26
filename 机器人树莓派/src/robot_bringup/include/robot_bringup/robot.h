#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>


#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float32.h>

#include <iostream>
#include <string.h>
#include <string> 

namespace robot
{
    class robot
    {
        public:
            robot();
            ~robot();
            bool init();                  
            bool deal(double RobotV, double RobotYawRate);
        
        private:
            void calcOdom();                             //里程计计算
            void pubOdomAndTf();                         //发布Odom和tf
	    void pubImuSensor();                         //发布IMU数据
	    void pubBattery();		                 //发布电池电压数据
        
        private:
            ros::Time current_time_, last_time_;         //时间

            double x_;                                   //机器人位姿
            double y_;
            double th_;                                  //里程计解算后的角度
            double th_imu;                               //imu获取的角度
        
            double vx_;                                  //机器人x方向速度
            double vy_;                                  //机器人y方向速度
            double vth_;                                 //机器人角速度
	    double battary_;                             //获取电池电压数据
            
            float qx_;                                   //底层获取的四元数数据x
	    float qy_; 					 //底层获取的四元数数据y
	    float qz_; 					 //底层获取的四元数数据z
            float qw_; 					 //底层获取的四元数数据w

            unsigned char sensFlag_;                     //通信预留发送和接收标志位，可进行信号控制使用
            unsigned char receFlag_;
               
            ros::NodeHandle nh;                          //创建句柄
            ros::Publisher pub_Odom, pub_ImuQuaternion, pub_battery;  //创建发布者，分别发布里程计和imu数据
            tf::TransformBroadcaster odom_broadcaster_;  //创建广播者

    };
    
}

#endif 

