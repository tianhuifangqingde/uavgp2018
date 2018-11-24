
/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/RCOut.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandCode.h>
#include <iostream>
#include <geometry_msgs/Vector3.h> //三个float64
#include <serial/serial.h>//control pump


using namespace std;



#define CHANNEL_INIT -1
#define LOW 0
#define MID 1
#define HIGH 2
struct point{
        float x;
        float y;
        float z; }point1,point2,point3;

//订阅UWB位置回调函数
geometry_msgs::Vector3 current_pos;
void get_location_cb(const geometry_msgs::Vector3::ConstPtr &msg) 
{
    current_pos = *msg;
}

sensor_msgs::NavSatFix current_gps;
void get_gps_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    current_gps = *msg;
    //ROS_INFO("%f %f %f",current_gps.longitude,current_gps.latitude,current_gps.altitude);
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

mavros_msgs::RCIn rcIn;
mavros_msgs::RCOut rcOut;

void get_rc_out(const mavros_msgs::RCOut::ConstPtr &msg)
{
    rcOut = *msg;
}
bool recorded = true;
int channel6_laststatus = CHANNEL_INIT;
int channel6_currentstatus = CHANNEL_INIT;
int channel7_laststatus = CHANNEL_INIT;
int channel7_currentstatus = CHANNEL_INIT;
//target3 channel8
int channel8_laststatus = CHANNEL_INIT;
int channel8_currentstatus = CHANNEL_INIT;
void checkChannel7()
{

    if (rcIn.channels[5] > 1500)
    {
        channel7_currentstatus = HIGH;
    }
    else
    {
        channel7_currentstatus = LOW;
    }
    if (channel7_currentstatus == HIGH)
    {
        point1.x = current_gps.latitude;
        point1.y = current_gps.longitude;
    }
    if (channel7_laststatus == HIGH && channel7_currentstatus == LOW)
    {
        ros::param::set("target1x",point1.x);
        ros::param::set("target1y",point1.y);
         ROS_INFO("Record Point:[%f %f]", point1.x, point1.y);
    }
    channel7_laststatus = channel7_currentstatus;
}
void checkChannel6()
{
 
    if (rcIn.channels[6] > 1500)
    {
        channel6_currentstatus = HIGH;
    }
    else
    {
        channel6_currentstatus = LOW;
    }
    if (channel6_currentstatus == HIGH)
    {
        point2.x = current_gps.latitude;
        point2.y = current_gps.longitude;
    }
    if (channel6_laststatus == HIGH && channel6_currentstatus == LOW)
    {
        ros::param::set("target2x", point2.x);
        ros::param::set("target2y", point2.y);
         ROS_INFO("Record Point:[%f %f]", point2.x, point2.y);
    }
    channel6_laststatus = channel6_currentstatus;
}
void checkChannel8()
{
      /*  uint8_t s_absorb_buffer[7];
        uint8_t s_dis_buffer[7];
        size_t sBUFFERSIZE=7;
        s_absorb_buffer[0]=0xaa;
        s_absorb_buffer[1]=0xaf; 
        s_absorb_buffer[2]=0x02;        
        s_absorb_buffer[3]=0x13;
        s_absorb_buffer[4]=0x01; //00为放
        s_absorb_buffer[5]=0x01; //00为放
        s_absorb_buffer[6]=s_absorb_buffer[0]+ s_absorb_buffer[1]+ s_absorb_buffer[2] +s_absorb_buffer[3]+s_absorb_buffer[4]+s_absorb_buffer[5];
        //dis
	s_dis_buffer[0]=0xaa;
        s_dis_buffer[1]=0xaf; 
        s_dis_buffer[2]=0x02;        
        s_dis_buffer[3]=0x13;
        s_dis_buffer[4]=0x00; //00为放
        s_dis_buffer[5]=0x00; //00为放
        s_dis_buffer[6]=s_dis_buffer[0]+ s_dis_buffer[1]+ s_dis_buffer[2] +s_dis_buffer[3]+s_dis_buffer[4]+s_dis_buffer[5];*/
    if (rcIn.channels[7] > 1350 && rcIn.channels[7] < 1650)
    {
        channel8_currentstatus = MID;
	point3.x = current_gps.latitude;
        point3.y = current_gps.longitude;
	//sp.write(s_dis_buffer,sBUFFERSIZE);
	
    }
    if (rcIn.channels[7] > 1650)
    {
        channel8_currentstatus = HIGH;
    }
    if (rcIn.channels[7] < 1350)
    {
        channel8_currentstatus = LOW;
    }
    if (channel8_laststatus == MID && channel8_currentstatus == HIGH)
    {
        ros::param::set("target3x", point3.x);
        ros::param::set("target3y", point3.y);
         ROS_INFO("Record Point:[%f %f]", point3.x, point3.y);
    }
    else if (channel8_laststatus == MID && channel8_currentstatus == LOW)
    {
        //sp.write(s_absorb_buffer,sBUFFERSIZE);
	ROS_INFO("123456");
	//ros::param::set("target3x", 0);
       // ros::param::set("target3y", 0);
    }
    channel8_laststatus = channel8_currentstatus;
}

void get_rc_in(const mavros_msgs::RCIn::ConstPtr &msg)
{
    rcIn = *msg;

    checkChannel7(); // record waypoints

    checkChannel6(); // push/clear waypoints
    
    checkChannel8();//target3
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_targets_node");
    ros::NodeHandle nh;
    
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000); 
  
    
   // ros::param::get("~wp_height", wp_height);
    //ros::param::get("~mission_speed", mission_speed);

   // ROS_INFO("wp_height: %f", wp_height);
   // ROS_INFO("mission_speed: %f", mission_speed);
     //订阅uwb位置
    ros::Subscriber get_uwb_sub = nh.subscribe<geometry_msgs::Vector3>
            ("uwb_position", 1, get_location_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber rc_out_sub = nh.subscribe<mavros_msgs::RCOut>("mavros/rc/out", 1, get_rc_out);
    ros::Subscriber rc_in_sub = nh.subscribe<mavros_msgs::RCIn>("mavros/rc/in", 1, get_rc_in);
   // ros::ServiceClient waypoint_setcurrent_client = nh.serviceClient<mavros_msgs::WaypointSetCurrent>("mavros/mission/set_current");
   // ros::ServiceClient waypoint_pull_client = nh.serviceClient<mavros_msgs::WaypointPull>("mavros/mission/pull");
    //ros::ServiceClient waypoint_push_client = nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
   // ros::ServiceClient waypoint_clear_client = nh.serviceClient<mavros_msgs::WaypointClear>("mavros/mission/clear");
    ros::Subscriber get_gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 1, get_gps_cb);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50);
    
    // wait for FCU connection
    while (ros::ok() )
    {
        float parameter=0;
	if(ros::param::get("target1x", parameter))
	ROS_INFO("parameter target1x = %f", parameter);
	if(ros::param::get("target1y", parameter))
	ROS_INFO("parameter target1y = %f", parameter);
	if(ros::param::get("target2x", parameter))
	ROS_INFO("parameter target2x = %f", parameter);
	if(ros::param::get("target2y", parameter))
	ROS_INFO("parameter target2y = %f", parameter);
	if(ros::param::get("target3x", parameter))
	ROS_INFO("parameter target3x = %f", parameter);
	if(ros::param::get("target3y", parameter))
	ROS_INFO("parameter target3y = %f", parameter);
	ros::spinOnce();
        rate.sleep();
    }

    
    return 0;
}