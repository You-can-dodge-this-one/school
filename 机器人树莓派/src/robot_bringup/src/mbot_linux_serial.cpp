#include "robot_bringup/mbot_linux_serial.h"
#include "robot_bringup/robot.h"

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include<geometry_msgs/Twist.h>


#define rBUFFERSIZE  62

using namespace std;
unsigned char r_buffer[rBUFFERSIZE];

//串口相关对象
serial::Serial ser;

const unsigned char ender[2] = {0x0d, 0x0a};
const unsigned char header[2] = {0x55, 0xaa};

//发送左右轮速控制速度共用体,传感器的X，Z，Angle
union sendData
{
	short d;
	unsigned char data[2];
}leftVelSet,rightVelSet;


//接收数据（左轮速、右轮速、角度）共用体（-32767 - +32768）
union receiveData
{
	short d;
	unsigned char data[2];
}leftVelNow,rightVelNow,angleNow,battery_volt;


union receiveData1
{
	float d;
	unsigned char data[4];
}q0,q1,q2,q3;

 

const double ROBOT_LENGTH = 192.00;  //mm
const double ROBOT_RADIUS = 96.00;  //两轮之间的半径长度mm

/********************************************************
函数功能：串口参数初始化
入口参数：无
出口参数：
********************************************************/
void serialInit()
{
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
   	//     return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
  	 //     return -1;
    }

}


/********************************************************
函数功能：将机器人的线速度和角速度分解成左右轮子速度，打包发送给下位机
入口参数：机器人线速度、角速度
出口参数：
********************************************************/
void writeSpeed(double RobotV, double YawRate,unsigned char ctrlFlag)
{
    unsigned char buf[11] = {0};//
    int i, length = 0;
    double r = RobotV / YawRate;//mm

    // 计算左右轮期望速度
    if(RobotV == 0)      //旋转
    {
        leftVelSet.d  = (short)(-YawRate * ROBOT_RADIUS);//mm/s
        rightVelSet.d = (short)(YawRate * ROBOT_RADIUS);//mm/s
    } 
    else if(YawRate == 0)//直线
    {
        leftVelSet.d  = (short)RobotV;//mm/s
        rightVelSet.d = (short)RobotV;
    }
    else                //速度不一致
    {
        leftVelSet.d  = (short)(YawRate * (r - ROBOT_RADIUS));//mm/s
        rightVelSet.d = (short)(YawRate * (r + ROBOT_RADIUS));
    }

    // 设置消息头
    for(i = 0; i < 2; i++)
        buf[i] = header[i];             //buf[0]  buf[1]
    
    // 设置机器人左右轮速度
    length = 5;
    buf[2] = length;                    //buf[2]
    for(i = 0; i < 2; i++)
    {
        buf[i + 3] = leftVelSet.data[i];  //buf[3] buf[4]
        buf[i + 5] = rightVelSet.data[i]; //buf[5] buf[6]
    }
    // 预留控制指令
    buf[3 + length - 1] = ctrlFlag;       //buf[7]

    // 设置校验值、消息尾
    buf[3 + length] = getCrc8(buf, 3 + length);//buf[8]
    buf[3 + length + 1] = ender[0];     //buf[9]
    buf[3 + length + 2] = ender[1];     //buf[10]

    // 通过串口下发数据
    ser.write(buf,sizeof(buf));
}

/********************************************************
函数功能：从下位机读取数据，解析出线速度、角速度、角度
入口参数：机器人线速度、角速度、角度，引用参数
出口参数：bool
********************************************************/
bool readSpeed(double &vx,double &vth,double &th,unsigned char &ctrlFlag,double &battary,float &qx,float &qy,float &qz,float &qw)
{
	if(ser.available())
	{
              // ROS_INFO_STREAM("Read serial port");

               ser.read(r_buffer,rBUFFERSIZE);

		 for(int i=0;i<rBUFFERSIZE;i++)
		{
			
		     if((r_buffer[i]==0x55)&&((r_buffer[i+1])==0xaa)&&i<=30)
                     {
 			if((r_buffer[i+29] == 0x0d) && (r_buffer[i+30] == 0x0a))
			{

		                for(int k = 0; k < 2; k++)
		                {
				      leftVelNow.data[k]   = r_buffer[i + 3 + k]; //buf[3] buf[4]
				      rightVelNow.data[k]  = r_buffer[i + 5 + k]; //buf[5] buf[6]
				      angleNow.data[k]     = r_buffer[i + 7 + k]; //buf[7] buf[8]
				      battery_volt.data[k] = r_buffer[i + 9 + k]; //buf[9] buf[10]
		                }
	
                                for(int k = 0; k < 4; k++)
				{
				      q0.data[k]          = r_buffer[i + 11 +k]; //buf[11] - buf[14]
				      q1.data[k]          = r_buffer[i + 15 +k]; //buf[15] - buf[18]
				      q2.data[k]          = r_buffer[i + 19 +k]; //buf[19] - buf[22]
				      q3.data[k]          = r_buffer[i + 23 +k]; //buf[23]  -buf[26]
				}
				/*      ROS_INFO("Left_v:%d\n",   leftVelNow.d);
				      ROS_INFO("Right_v:%d\n",  rightVelNow.d);
				      ROS_INFO("Angle:%d\n",    angleNow.d);
 				      ROS_INFO("Battery:%d\n",  battery_volt.d);
				      ROS_INFO("qx:%f\n",q0.d);
				      ROS_INFO("qy:%f\n",q1.d);
				      ROS_INFO("qz:%f\n",q2.d);
				      ROS_INFO("qw:%f\n",q3.d);
				*/
			}		     
		     }		
		}
         
        }
	 //===========================速度计算和Angle获取===========================================================
	    // x方向速度，以及角速度
	    vx       = (rightVelNow.d + leftVelNow.d) / 2.0 / 1000.0;        //m/s
	    vth      = (rightVelNow.d - leftVelNow.d) / ROBOT_LENGTH ;       //rad/s
	    th       = angleNow.d*0.01745;//实时角度信息(rad)
  
	    battary  = battery_volt.d;
	    qx       = q0.d;
	    qy       = q1.d;
	    qz       = q2.d;
	    qw       = q3.d;

	    return true;
}

/********************************************************
函数功能：获得8位循环冗余校验值
入口参数：数组地址、长度
出口参数：校验值
********************************************************/
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
    unsigned char crc;
    unsigned char i;
    crc = 0;
    while(len--)
    {
        crc ^= *ptr++;
        for(i = 0; i < 8; i++)
        {
            if(crc&0x01)
                crc=(crc>>1)^0x8C;
            else 
                crc >>= 1;
        }
    }
    return crc;
}
