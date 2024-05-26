

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "serial_msgs/serial.h"
#include<geometry_msgs/Twist.h>

#define rBUFFERSIZE     26
unsigned char r_buffer[rBUFFERSIZE];


serial::Serial ser;

    
         
int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node1");
    ros::NodeHandle nh;

  

    ros::Publisher msg_pub = nh.advertise<serial_msgs::serial>("read1", 1000);

    try
    {
      /*串口设置，我这里使用的是电脑后面的9针232口，如果usb转串口一般是ttyUSB0，可以使用dmseg命令查看，
      我用的Ubuntu16.04，测试ch340和PL2302的usb转串口都自带驱动了*/
        ser.setPort("/dev/robot");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
      /*如果dmesg命令能看到串口而打开失败，一般是权限问题，使用命令打开对应的串口，sudo chmod 666 /dev/ttyS0*/
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    
    while(ros::ok())
{
       // serial_msgs::serial msg;

       // ros::Rate loop_rate(10);

        if(ser.available())
	{
               ROS_INFO_STREAM("Read serial port");

               ser.read(r_buffer,rBUFFERSIZE);

		 for(int i=0;i<rBUFFERSIZE;i++)
		{
			


		     if((r_buffer[i]==0x55)&&((r_buffer[i+1])==0xaa)&&i<=12)
                     {

                        ROS_INFO("[0x%02x]",r_buffer[i]);
			ROS_INFO("[0x%02x]",r_buffer[i+1]);
			ROS_INFO("[0x%02x]",r_buffer[i+2]);
			ROS_INFO("[0x%02x]",r_buffer[i+3]);
			ROS_INFO("[0x%02x]",r_buffer[i+4]);
			ROS_INFO("[0x%02x]",r_buffer[i+5]);
			ROS_INFO("[0x%02x]",r_buffer[i+6]);
			ROS_INFO("[0x%02x]",r_buffer[i+7]);
			ROS_INFO("[0x%02x]",r_buffer[i+8]);
			ROS_INFO("[0x%02x]",r_buffer[i+9]);
			ROS_INFO("[0x%02x]",r_buffer[i+10]);
			ROS_INFO("[0x%02x]",r_buffer[i+11]);
			ROS_INFO("[0x%02x]",r_buffer[i+12]);
		     }
 			//i=0;
			//break;

		}
		ROS_INFO_STREAM("End reading from serial port");
        }

        //ros::spinOnce();
        //loop_rate.sleep();

}
}
