#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include <std_msgs/UInt32.h> 
#include <std_msgs/Empty.h> 
#include <std_msgs/UInt8MultiArray.h> 
#include<stdlib.h>
#include <sstream>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/RCOut.h>

using namespace std;

serial::Serial ser; //声明串口对象 
  
//回调函数 

void write_callback(const std_msgs::String::ConstPtr& msg) 
{ 
    ROS_INFO_STREAM("Writing to serial port" <<msg->data); 
    ser.write(msg->data);   //发送串口数据 
} 
sensor_msgs::NavSatFix global_pose;
void global_pose_cb(const sensor_msgs::NavSatFixConstPtr& msg)
{
  global_pose = *msg;
  ROS_INFO("got global position [%d]: %f, %f, %f", global_pose.header.seq, global_pose.latitude, global_pose.longitude, global_pose.altitude);
}
///////state
std_msgs::UInt32 current_state;
void get_state_cb(const std_msgs::UInt32::ConstPtr &msg) //????
{
    current_state = *msg;
}




mavros_msgs::RCIn rcIn;
mavros_msgs::RCOut rcOut;

void get_rc_out(const mavros_msgs::RCOut::ConstPtr &msg)
{
    rcOut = *msg;
}

int flag=0;
void get_rc_in(const mavros_msgs::RCIn::ConstPtr &msg)
{
    rcIn = *msg;
    flag=rcIn.channels[7];
}
 
int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "serial_example_node"); 
    //声明节点句柄 
    ros::NodeHandle nh; 
  
    //订阅主题，并配置回调函数 
   ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback); 
   ros::Subscriber get_state_sub = nh.subscribe("mission_state", 10, get_state_cb);
   ros::Subscriber rc_out_sub = nh.subscribe<mavros_msgs::RCOut>("mavros/rc/out", 1, get_rc_out);
   ros::Subscriber rc_in_sub = nh.subscribe<mavros_msgs::RCIn>("mavros/rc/in", 1, get_rc_in);
   ros::Subscriber global_pose_sub = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 1, global_pose_cb);

   
    //发布主题 
    ///absorb flag
     ros::Publisher pump_pub = nh.advertise<std_msgs::UInt32>("control_flag", 10); 
    //ros::Publisher pump_pub = nh.advertise<std_msgs::String>("control_flag", 1000); 
   float x2,y2; 
  ros::param::get("target2x", x2);
  ros::param::get("target2y", y2);
  /////////////////control pump////////////////////////
   uint8_t s_buffer_absorb[7];
    size_t sBUFFERSIZE=7;
    s_buffer_absorb[0]=0xaa;
    s_buffer_absorb[1]=0xaf;
    s_buffer_absorb[2]=0x02;
    s_buffer_absorb[3]=0x13;
    s_buffer_absorb[4]=0x01;
    s_buffer_absorb[5]=0x01;
    s_buffer_absorb[6]=s_buffer_absorb[0]+ s_buffer_absorb[1]+ s_buffer_absorb[2] +s_buffer_absorb[3]+s_buffer_absorb[4]+s_buffer_absorb[5];//s_buffer[4]^s_buffer[5];

    uint8_t s_buffer_dis[7];
    s_buffer_dis[0]=0xaa;
    s_buffer_dis[1]=0xaf;
    s_buffer_dis[2]=0x02;
    s_buffer_dis[3]=0x13;
    s_buffer_dis[4]=0x00;
    s_buffer_dis[5]=0x00;
    s_buffer_dis[6]=s_buffer_dis[0]+ s_buffer_dis[1]+ s_buffer_dis[2] +s_buffer_dis[3]+s_buffer_dis[4]+s_buffer_dis[5];//s_buffer[4]^s_buffer[5];
    ROS_INFO_STREAM("check: " << s_buffer_dis[6]);
  
  
  
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
   
    //指定循环的频率 
    ros::Rate loop_rate(50); 
    std_msgs::UInt32 control_flag;
    control_flag.data=0;
    while(ros::ok()) 
    { 
   ros::param::get("target2x", x2);
  ros::param::get("target2y", y2);
        ROS_INFO_STREAM("length:" << ser.available());
        if(ser.available()){ 
            ROS_INFO_STREAM("Reading from serial port\n");
	    std_msgs::UInt8MultiArray  serial_data; 	                
            ser.read (serial_data.data,ser.available()) ;  
 	   
	    for (int i = 0 ; i < 14;i++) 
            {
                  
                if(serial_data.data[i] == 170 && serial_data.data[i+1] == 175 && i<14)
		{ 
                  ROS_INFO("%d",serial_data.data[i+4] ); 
		  ROS_INFO("%d",serial_data.data[i+5] );
		  ROS_INFO("%d",current_state.data ); 
		  if((serial_data.data[i+5]==0x01)&&((current_state.data==3)||(current_state.data==4)))
		  {
		     if(flag < 1350)
		     {
		       ser.write(s_buffer_absorb,sBUFFERSIZE);	
		     }
		     //else
		     //{
		     //  ser.write(s_buffer_dis,sBUFFERSIZE);
		     //}  
		  }
		 if((serial_data.data[i+5]==0x01))
		 {
		    control_flag.data=1;
		 }
		 else
		 {
		    control_flag.data=0;
		 }
		 ROS_INFO("%d", control_flag.data );
                } 
            } 
	 } 
  
         if((abs(global_pose.latitude-x2)<0.00001)&&(abs(global_pose.longitude-y2)<0.00001)&&current_state.data==5)
           { 
	     for(int i=0;i<10;i++)
	     {
	      ser.write(s_buffer_dis,sBUFFERSIZE);
	     }
	   }
	 
         pump_pub.publish(control_flag);
        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
        loop_rate.sleep(); 
      
  
    } 
    
} 

