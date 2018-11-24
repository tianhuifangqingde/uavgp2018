#include <ros/ros.h>
#include <ros/param.h>
#include <serial/serial.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <string>
#include <sstream>
#include <iostream>
#include<std_msgs/ByteMultiArray.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
using namespace std;


serial::Serial sp; //声明串口对象

void write_callback(const std_msgs::String::ConstPtr& msg) 
{ 
    ROS_INFO_STREAM("Writing to serial port" <<msg->data); 
    sp.write(msg->data);   //发送串口数据 
} 

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


int main(int argc,char** argv)
{
    ros::init(argc,argv,"sp_node");//初始化节点
    ros::NodeHandle nh;//声明节点句柄
     //订阅主题，并配置回调函数 
    //ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback); 
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    
    //发布主题 
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000); 
     ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    std::string uwb_port_name("/dev/ttyUSB0");
    
    try
    {
        sp.setPort(uwb_port_name);/* code for Try */
        sp.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        sp.setTimeout(to);
        sp.open();
    }
    catch (serial::IOException& e)
    {
       ROS_ERROR_STREAM("Unable to open port "); 
       return -1;  /* code for Catch */
    }
        
    //检测串口是否已经打开，并给出提示信息 
    if(sp.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port initialized"); 
    } 
    else 
    { 
        return -1; 
    } 
  
    //指定循环的频率 
    ros::Rate loop_rate(50); 
    int flag=0;
    while(ros::ok())
    {
      //flag++;
     // if(flag>100) break;
     geometry_msgs::TwistStamped velocity_tw;
     
     velocity_pub.publish(velocity_tw);
      ROS_INFO_STREAM("55"); 
      ROS_INFO("PX4 Mode: %s", current_state.mode.c_str());
        if (current_state.mode=="OFFBOARD")
        {
           
	    break;
        }
      ros::spinOnce(); 
      loop_rate.sleep(); 
    }
    while(ros::ok()) 
    { 
       
       
       //std_msgs::ByteMultiArray data_to_send;
       // data_to_send.data.push_back('F');
       // data_to_send.data.push_back(2.0);
       // sp.write((unsigned char *)data_to_send.data.data(),data_to_send.data.size());       
       
        uint8_t s_buffer[7];
        size_t sBUFFERSIZE=7;
        s_buffer[0]=0xaa;
        s_buffer[1]=0xaf; 
        s_buffer[2]=0x02;        
        s_buffer[3]=0x13;
        s_buffer[4]=0x01;
        s_buffer[5]=0x01;
        s_buffer[6]=s_buffer[0]+ s_buffer[1]+ s_buffer[2] +s_buffer[3]+s_buffer[4]+s_buffer[5];//s_buffer[4]^s_buffer[5]; 
        //sp.write(s_buffer,sBUFFERSIZE);
        ROS_INFO_STREAM("66"); 
	
        /*
        ROS_INFO_STREAM("Read: " << sp.available()); 
        if(sp.available()){ 
            ROS_INFO_STREAM("Reading from serial port\n"); 
            std_msgs::String result; 
            result.data = sp.read(sp.available()); 
            ROS_INFO_STREAM("Read: " << result.data[0]); 
            read_pub.publish(result); 
        }  
        */ 
        //处理ROS的信息，比如订阅消息,并调用回调函数 
            ros::spinOnce(); 
            loop_rate.sleep();   
    } 
 }