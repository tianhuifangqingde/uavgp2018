#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include <std_msgs/UInt32.h> 
#include <std_msgs/Empty.h> 
#include<stdlib.h>
#include <sstream>
using namespace std;

serial::Serial ser; //声明串口对象 
  
//回调函数 
void write_callback(const std_msgs::String::ConstPtr& msg) 
{ 
    ROS_INFO_STREAM("Writing to serial port" <<msg->data); 
    ser.write(msg->data);   //发送串口数据 
} 

//laser
std_msgs::String current_laser;
std::string laser;

void get_laser_cb(const std_msgs::String::ConstPtr &msg)
{
    current_laser = *msg;
    laser = current_laser.data;
}

template <class Type>  //string转int/float
Type stringToNum(const string& str)  
{  
    istringstream iss(str);  
    Type num;  
    iss >> num;  
    return num;      
}

int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "test_laser_node"); 
    //声明节点句柄 
    ros::NodeHandle nh; 
    ros::Rate rate(50.0);

    //订阅主题，并配置回调函数 
    ros::Subscriber get_laser_sub = nh.subscribe<std_msgs::String>
            ("get_laser", 1, get_laser_cb);
    while (ros::ok)
    {
        /* code for loop body */
    
       ROS_INFO_STREAM("data:" << current_laser);
       // ROS_INFO_STREAM("data:" << stringToNum<float>(laser));
    
        ros::spinOnce(); 
        rate.sleep(); 
    }
    return 0;
} 
 

