#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include <std_msgs/UInt32.h> 
#include <std_msgs/Empty.h> 
#include <std_msgs/UInt8MultiArray.h> 
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
  
int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "serial_example_node"); 
    //声明节点句柄 
    ros::NodeHandle nh; 
  
    //订阅主题，并配置回调函数 
   ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback); 
    //发布主题 
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("get_laser", 1000); 
  
    try 
    { 
    //设置串口属性，并打开串口 
        ser.setPort("/dev/ttyUSB0"); 
        ser.setBaudrate(115200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port "); 
        return -1; 
    } 
  
    //检测串口是否已经打开，并给出提示信息 
    if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port initialized"); 
    } 
    else 
    { 
        return -1; 
    } 
    struct point{
        float x;
        float y;
        float z; }target1,target2;
    ros::param::get("target1x", target1.x);
    ros::param::get("target1y", target1.y);
    ros::param::get("target2x", target2.x);
    ros::param::get("target2y", target2.y);
    //ROS_INFO("target1x,%f", target1.x);
    //ROS_INFO("target1x,%f", target2.x);
    //指定循环的频率 
    ros::Rate loop_rate(50); 
    unsigned int low = 0x00;
    unsigned int high= 0x00;
    unsigned int laser_distance;
    std_msgs::String result;
    std::string ss;
    std_msgs::UInt8MultiArray  serial_data;
    while(ros::ok()) 
    { 
        ROS_INFO_STREAM("length:" << ser.available());
        if(ser.available()){ 
            ROS_INFO_STREAM("Reading from serial port\n"); 
	    //ROS_INFO("target1x:%f",target1.x);
             
 
 
             
            ser.read (serial_data.data,ser.available()) ;
	   // ROS_INFO("%o",serial_data.data[0]);
 
            
        }
	    for (int i = 0 ; i < 18;i++)
            {
                 
                if(serial_data.data[i] == 170 && serial_data.data[i+1] == 175 && i<18)
                  { 
		
                 ROS_INFO("%d",serial_data.data[i+4] ); 
                }
            //ROS_INFO_STREAM("Read: " <<ss[i]); 
            } 
            /*
            if (ss[2]>'9')
                {
                    as = as *16+ ( int)(ss[2]-'a'+10);
                }
                else
                {
                    as = as *16+ (int)(ss[2]-'0');
                }
            if (ss[3]>'9')
                {
                    b = b *16+ ( int)(ss[3]-'a'+10);
                }
                else
                {
                    b = b *16+ (int)(ss[3]-'0');
                }
            */
        
      
            
            string Result;   
            ostringstream convert;  
            convert << laser_distance;
            Result = convert.str(); 
            std_msgs::String msg;//define standard string msg  
            msg.data = Result.data();//get the string need to be pblished  
              
            read_pub.publish(msg); 
            
         

        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
        loop_rate.sleep(); 
       low= 0;
       high= 0; 
  
    } 
} 

