#include <vector>
#include "robot_bringup/robot.h"
#include "robot_bringup/mbot_linux_serial.h"

using namespace std;

namespace robot
{
    boost::array<double, 36> odom_pose_covariance = {
        {1e-9, 0, 0, 0, 0, 0, 
         0, 1e-3,1e-9, 0, 0, 0, 
         0, 0, 1e6, 0, 0, 0,
         0, 0, 0, 1e6, 0, 0, 
         0, 0, 0, 0, 1e6, 0, 
         0, 0, 0, 0, 0, 1e-9}};
    boost::array<double, 36> odom_twist_covariance = {
        {1e-9, 0, 0, 0, 0, 0, 
         0, 1e-3,1e-9, 0, 0, 0, 
         0, 0, 1e6, 0, 0, 0, 
         0, 0, 0, 1e6, 0, 0, 
         0, 0, 0, 0, 1e6, 0, 
         0, 0, 0, 0, 0, 1e-9}};


    boost::array<double, 36> odom_pose_covariance2 = {
	{1e-3, 0, 0, 0, 0, 0, 
         0, 1e-3, 0, 0, 0, 0,
         0, 0, 1e6, 0, 0, 0,
         0, 0, 0, 1e6, 0, 0,
         0, 0, 0, 0, 1e6, 0,
         0, 0, 0, 0, 0, 1e3}};

    boost::array<double, 36> odom_twist_covariance2 = {
	{1e-3, 0, 0, 0, 0, 0, 
         0, 1e-3, 0, 0, 0, 0,
         0, 0, 1e6, 0, 0, 0,
         0, 0, 0, 1e6, 0, 0,
         0, 0, 0, 0, 1e6, 0,
         0, 0, 0, 0, 0, 1e3}};

    robot::robot():x_(0.0), y_(0.0), th_(0.0),vx_(0.0), vy_(0.0), vth_(0.0),sensFlag_(0),receFlag_(0) {}//构造函数
    robot::~robot(){}                                                                                   //析构函数
    /********************************************************
    函数功能：串口参数初始化、时间变量初始化、实例化发布对象
    入口参数：无
    出口参数：bool
    ********************************************************/
    bool robot::init()
    {
        // 串口初始化连接
        serialInit();
               
        ros::Time::init();
        current_time_ = ros::Time::now();
        last_time_ = ros::Time::now();
        
        //定义发布消息的名称
        pub_Odom          = nh.advertise<nav_msgs::Odometry>("odom", 50);
		
	
        pub_ImuQuaternion = nh.advertise<sensor_msgs::Imu>("imu_data", 20);

	
	pub_battery       = nh.advertise<std_msgs::Float32>("battery", 20);
	

        return true;
    }
 
    /********************************************************
    函数功能：根据机器人线速度和角度计算机器人里程计
    入口参数：无
    出口参数：无
    ********************************************************/
    void robot::calcOdom()
    {
        ros::Time curr_time;
        curr_time = ros::Time::now();
         
        double dt = (curr_time - last_time_).toSec(); //间隔时间
        double delta_x = (vx_ * cos(th_)) * dt;         //th_弧度
        double delta_y = (vx_ * sin(th_)) * dt;
        double delta_th = vth_ * dt;
       
        //打印时间间隔调试信息，不用的时候可以关闭
        //ROS_INFO("dt:%f\n",dt);                       //s

        //里程计累加
        x_ += delta_x;
        y_ += delta_y;

        //实时角度信息,如果这里不使用IMU，也可以通过这种方式计算得出
        th_ += delta_th;                                //里程计解算积分后角度

        last_time_ = curr_time;  

        //打印位姿调试信息，不用的时候可以关闭
       /*ROS_INFO("x_:%f\n",x_);
        ROS_INFO("y_:%f\n",y_);
        ROS_INFO("th_:%f\n",th_);
        ROS_INFO("th_imu:%f\n",th_imu);
        ROS_INFO("th_imu_diff:%f\n",th_imu_diff);
        ROS_INFO("vth_imu:%f\n",vth_imu);*/


      
    }
    /********************************************************
    函数功能：发布机器人里程计和TF
    入口参数：无
    出口参数：无
    ********************************************************/
    void robot::pubOdomAndTf()
    {
        current_time_ = ros::Time::now();
     
        geometry_msgs::Quaternion odom_quat;
        odom_quat = tf::createQuaternionMsgFromYaw(th_);
        // 发布里程计消息
        nav_msgs::Odometry msgl;
        msgl.header.stamp = current_time_;
        msgl.header.frame_id = "odom";

        msgl.pose.pose.position.x = x_;
        msgl.pose.pose.position.y = y_;
        msgl.pose.pose.position.z = 0.0;
        msgl.pose.pose.orientation = odom_quat;
        msgl.pose.covariance = odom_pose_covariance;

        msgl.child_frame_id = "base_link";
        msgl.twist.twist.linear.x = vx_;
        msgl.twist.twist.linear.y = vy_;
        msgl.twist.twist.angular.z = vth_;
        msgl.twist.covariance = odom_twist_covariance;

        if(vx_ == 0)
        {
    	 memcpy(&msgl.pose.covariance, &odom_pose_covariance, sizeof(odom_pose_covariance));
                
    	 memcpy(&msgl.twist.covariance, &odom_twist_covariance, sizeof(odom_twist_covariance));
        }
        else
        {
    	memcpy(&msgl.pose.covariance, &odom_pose_covariance2, sizeof(odom_pose_covariance2));
    	memcpy(&msgl.twist.covariance, &odom_twist_covariance2, sizeof(odom_twist_covariance2));
        }		
    
        pub_Odom.publish(msgl);

    }


   void robot::pubImuSensor()
   {

	sensor_msgs::Imu ImuQuaternion;

	ImuQuaternion.header.stamp = ros::Time::now(); 
	ImuQuaternion.header.frame_id = "imu_link"; 

	ImuQuaternion.orientation.x = 0.0; 
	ImuQuaternion.orientation.y = 0.0; 
	
        ImuQuaternion.orientation.z = qz_;
	ImuQuaternion.orientation.w = qw_;
	ImuQuaternion.orientation_covariance[0] = 1e6;
	ImuQuaternion.orientation_covariance[4] = 1e6;
	ImuQuaternion.orientation_covariance[8] = 1e-6;

	ImuQuaternion.angular_velocity.x = 0.0;		
	ImuQuaternion.angular_velocity.y = 0.0;		
	ImuQuaternion.angular_velocity.z = 0.0;

	ImuQuaternion.angular_velocity_covariance[0] = 1e6;
	ImuQuaternion.angular_velocity_covariance[4] = 1e6;
	ImuQuaternion.angular_velocity_covariance[8] = 1e-6;

	ImuQuaternion.linear_acceleration.x = 0; 
	ImuQuaternion.linear_acceleration.y = 0; 
	ImuQuaternion.linear_acceleration.z = 0;  

	pub_ImuQuaternion.publish(ImuQuaternion); 
 
   }

   void robot::pubBattery()
   {
	std_msgs::Float32  battery;
	battery.data  =  battary_/100;
	pub_battery.publish(battery);
   }


    /********************************************************
    函数功能：自定义deal，实现整合，并且发布TF变换和Odom
    入口参数：机器人线速度和角速度，调用上面三个函数
    出口参数：bool
    ********************************************************/
    bool robot::deal(double RobotV, double RobotYawRate)
    {
        // 向STM32发送对机器人的预期控制速度，以及预留信号控制位
        writeSpeed(RobotV, RobotYawRate, sensFlag_);
        // 从STM32读取机器人实际线速度，角速度和角度，以及预留信号控制位
        readSpeed(vx_, vth_, th_imu, receFlag_, battary_, qx_, qy_, qz_, qw_);
        // 里程计计算
        calcOdom();

        // 发布TF变换和Odom
        pubOdomAndTf();
	// 发布IMU话题数据
 	pubImuSensor();
	// 发布电池电压数据
	pubBattery();
 
    }
}

