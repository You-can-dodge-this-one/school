
#ifndef MY_SLAM_GMAPPING_H
#define MY_SLAM_GMAPPING_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/GetMap.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"

#include "../part_slam/gridfastslam/gridslamprocessor.h"

#include <boost/thread.hpp>

/***********************************************************************************************************
  基于滤波器的SLAM算法-《gmapping算法的删减版》
  在原来gmapping源码的基础之上，公众号：小白学移动机器人的作者对其进行了大刀阔斧的更改。
  （1）删除几乎所有不需要的代码，对代码的运行结构也进行了调整
  （2）对该代码进行详细中文注释，以及对核心代码进行更改
  =============公众号：小白学移动机器人========================================================================
  欢迎关注公众号，从此学习的路上变得不再孤单，加油！奥利给！！！
  2020年11月11日
***********************************************************************************************************/

//从smap的二位数组存储格式，转到一维数组，数组序号也需要转换到一维数组
#define   MAP_IDX(sx, i, j) ((sx) * (j) + (i))

#define   GMAPPING_FREE         (0)
#define   GMAPPING_UNKNOWN      (-1)
#define   GMAPPING_OCC          (100)

class MySlamGMapping
{
  public:
    MySlamGMapping();
    ~MySlamGMapping();
    
    void init();          //构造函数调用的初始化函数
    void startLiveSlam(); //开始实时SLAM 
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);                    //scan的回调函数ros壳的核心函数

    bool mapCallback(nav_msgs::GetMap::Request  &req,nav_msgs::GetMap::Response &res);   //订阅动态地图
    void publishLoop(double transform_publish_period);                                   //发布map和odom的TF变换
    void publishTransform();                                                          

  private:

    bool initMapper(const sensor_msgs::LaserScan& scan);                                 //第一帧scan初始化
    bool addScan(const sensor_msgs::LaserScan& scan, GMapping::OrientedPoint& gmap_pose);//对所有帧scan都有效
    bool getOdomPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t);            //获得当前帧对应的里程计位姿
    void updateMap(const sensor_msgs::LaserScan& scan);                                  //更新地图操作，主要是rviz的可视化操作部分

    ros::NodeHandle node_;                     //MySlamGMapping对象的句柄
    ros::Publisher sst_;                       //发布话题map
    ros::Publisher sstm_;                      //发布话题map_metadata
    ros::ServiceServer ss_;                    //发布服务dynamic_map

    tf::TransformListener tf_;                 //以下三行组合使用
    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;

    tf::TransformBroadcaster* tfB_;            //tf变换的发布者，发布map与odom的TF变换

    GMapping::GridSlamProcessor* gsp_;         //GridSlamSLAM的对象
    std::vector<double> laser_angles_;         //存储每一个激光点的角度

    bool got_first_scan_;                      //是否获得第一帧scan标志       
    bool got_map_;                             //获得地图的标志

    //使用nav_msgs::GetMap::Response，主要是方便被订阅
    nav_msgs::GetMap::Response map_;           //用来发布map的实体对象

    ros::Duration map_update_interval_;        //地图更新间隔，发布地图时用，单位秒，每隔几秒更新一次
    tf::Transform map_to_odom_;                //用来描述map到odom的TF变换
    boost::mutex map_to_odom_mutex_;           //map_to_odom互斥锁
    boost::mutex map_mutex_;                   //map互斥锁

    boost::thread* transform_thread_;          //发布转换关系的线程

    std::string laser_frame_;                  //激光雷达坐标系的名字
    std::string scan_topic_;                   //激光雷达数据话题的名字    
    std::string map_frame_;                    //地图坐标系的名字
    std::string odom_frame_;                   //里程计坐标系的名字

    //gmapping的参数
    double maxRange_;                          //激光雷达最大的量程
    double maxUrange_;                         //激光雷达最大使用距离
    double minimum_score_;                     //判别scan match成功与否的最小得分
    double sigma_;                             //用来表示计算粒子匹配得分
    int kernelSize_;                           
    double lstep_;
    double astep_;
    int iterations_;
    double lsigma_;                            //用来表示计算粒子似然得分
    double ogain_;
    int lskip_;
    double srr_;
    double srt_;
    double str_;
    double stt_;
    double linearUpdate_;
    double angularUpdate_;
    double temporalUpdate_;
    double resampleThreshold_;
    int particles_;
    double xmin_;
    double ymin_;
    double xmax_;
    double ymax_;
    double delta_;
    double occ_thresh_;
    
    ros::NodeHandle private_nh_;              //获得launch文件参数的句柄
    
    unsigned long int seed_;                  //高斯噪声的随机数种子
    
    double transform_publish_period_;
    double tf_delay_;
};

#endif
