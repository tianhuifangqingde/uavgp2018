#include <cstdlib>
#include<cmath>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/UInt32.h>
 
//#include <mavros_msgs/SetMavFrame.h>
#define PI 3.1415926535898
sensor_msgs::NavSatFix global_pose;
void global_pose_cb(const sensor_msgs::NavSatFixConstPtr& msg)
{
  global_pose = *msg;
  ROS_INFO("got global position [%d]: %f, %f, %f", global_pose.header.seq, global_pose.latitude, global_pose.longitude, global_pose.altitude);
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
  ROS_INFO("got status: %d, %d", (int)msg->armed, (int)msg->connected);
}
//get rel_alt
std_msgs::Float64 current_rel_alt;
void rel_alt_cb(const std_msgs::Float64::ConstPtr& msg)
{
  current_rel_alt= *msg;
  ROS_INFO("got current_rel_alt: %f",current_rel_alt.data);
}

//get_image_info
std_msgs::Int32MultiArray image_info;
void catterCallback(const std_msgs::Int32MultiArray::ConstPtr& array)
{ 
   image_info=*array;
   ROS_INFO("%d  %d  %d  %d  %d", array->data[0], array->data[1], array->data[2], array->data[3]
, array->data[4] );
}
//get compass yaw
std_msgs::Float64 Uavyaw;
void yaw_cb(const std_msgs::Float64::ConstPtr& msg)
{
   Uavyaw= *msg;
  ROS_INFO("got  Uavyaw: %f",Uavyaw.data);
}
///////control_flag//////////////
std_msgs::UInt32 control_flag;
void get_flag_cb(const std_msgs::UInt32::ConstPtr &msg) //
{
    control_flag = *msg;
}

///////////////////////////////
int main(int argc, char **argv)
{

  int rate_hz = 20;

  ros::init(argc, argv, "global_pos_mission");
  ros::NodeHandle n;
  ros::NodeHandle& nh = n;
  ros::Rate rate(rate_hz);
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                              ("mavros/state", 10, state_cb);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                 ("mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                     ("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                       ("mavros/set_mode", 1);
  ros::Subscriber global_pose_sub = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 1, global_pose_cb);

  global_pose.header.seq = 0;

  ros::Subscriber rel_alt_sub=nh.subscribe<std_msgs::Float64>("mavros/global_position/rel_alt",1,rel_alt_cb);
    
  //sub image_info
  ros::Subscriber image_sub=nh.subscribe("target_detection1",10,catterCallback);
  //set mav_frame
   // ros::ServiceClient set_mav_frame_client= nh.serviceClient<mavros_msgs::SetMavFrame>
      //      ("mavros/setpoint_velocity/mav_frame");  
  //??local??(???)
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10); 
  //pub state
  ros::Publisher state_pub = nh.advertise<std_msgs::UInt32>
            ("mission_state", 10); 
  //sub yaw
  ros::Subscriber yaw_sub=nh.subscribe<std_msgs::Float64>("mavros/global_position/compass_hdg",1,yaw_cb);	    
  //get control flag
  ros::Subscriber get_flag_sub = nh.subscribe("control_flag", 10, get_flag_cb);
  // wait for FCU connection
  while(ros::ok() && (!current_state.connected || global_pose.header.seq == 0))
  {
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("got global position [%d]: %f, %f, %f", global_pose.header.seq, global_pose.latitude, global_pose.longitude, global_pose.altitude);

  ////////////////////////////////////////////
  ///////////////////ARM//////////////////////
  ////////////////////////////////////////////
  ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  mavros_msgs::CommandBool srv;
  srv.request.value = true;
  if(arming_cl.call(srv))
  {
    ROS_INFO("ARM send ok %d", srv.response.success);
  }
  else
  {
    ROS_ERROR("Failed arming or disarming");
  }

  ////////////////////////////////////////////
  /////////////////CLEAR MISSION/////////////////
  ////////////////////////////////////////////


  ros::ServiceClient wp_clear_client = n.serviceClient<mavros_msgs::WaypointClear>("/mavros/mission/clear");
  mavros_msgs::WaypointClear wp_clear_srv;
  if (wp_clear_client.call(wp_clear_srv))
  {
    ROS_INFO("Waypoint list was cleared");
  }
  else
  {
    ROS_ERROR("Waypoint list couldn't been cleared");
  }

  ////////////////////////////////////////////
  /////////////////DO MISSION/////////////////
  ////////////////////////////////////////////
  ros::ServiceClient client_wp = n.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");

  mavros_msgs::WaypointPush srv_wp;
  //srv_wp.request.start_index = 0;
  mavros_msgs::CommandHome set_home_srv;
 
  ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  mavros_msgs::SetMode srv_setMode;

  mavros_msgs::Waypoint wp;
  ////////////////////////////////////////////
  //get target1,target2 param
  struct target{float x;float y;float z;}target1,target2,target3;
  ros::param::get("target1x", target1.x);
  ros::param::get("target1y", target1.y);
  ros::param::get("target2x", target2.x);
  ros::param::get("target2y", target2.y);
  ros::param::get("target3x", target3.x);
  ros::param::get("target3y", target3.y);
  ROS_INFO("target1x,%f", target1.x);
   ROS_INFO("target2y,%f", target1.y); 
    ROS_INFO("target2x,%f", target2.x); 
     ROS_INFO("target2y,%f", target2.y);
       ROS_INFO("target3x,%f", target3.x); 
       ROS_INFO("target3y,%f", target3.y);
  ////////////////////////////////////////////
  //velocity initail
  
    geometry_msgs::TwistStamped velocity_tw,vs_body_axis;
     velocity_tw.twist.linear.x = 0;
     velocity_tw.twist.linear.y = 0;
     velocity_tw.twist.linear.z = 0;
      ros::Time last_request=ros::Time::now();
  ////////////////////////////////////////////
  int state =1,get_target=0;
  std_msgs::UInt32 mission_state;
  while(ros::ok())
  {
  if(state==1)
  {
  // fill wp
  wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
  wp.command = mavros_msgs::CommandCode::NAV_TAKEOFF;
  wp.is_current   = true;
  wp.autocontinue = true;
  wp.param1       = 5;
  wp.param4       =NAN;//??yaw??
  wp.z_alt        = 3;
  wp.x_lat        = global_pose.latitude;
  wp.y_long       = global_pose.longitude;
  srv_wp.request.waypoints.push_back(wp);
 // wp.command =mavros_msgs::CommandCode::DO_CHANGE_SPEED;
 ////////////////////////////////////////////////////////
 //target1:need switch local_velocity
  wp.command      = mavros_msgs::CommandCode::NAV_WAYPOINT;
  wp.is_current   = false;
  wp.autocontinue = true;
  wp.param1       = 3; 
  wp.z_alt        = 3;
  wp.x_lat        = target1.x;//47.398317;
  wp.y_long       = target1.y;//8.5460425;
  srv_wp.request.waypoints.push_back(wp);
  ////////////////////////////////
  wp.param1       = 3; 
  wp.z_alt        = 6;
  wp.x_lat        = target1.x;
  wp.y_long       = target1.y;
  srv_wp.request.waypoints.push_back(wp);
 
  
  //srv_wp.request.waypoints.push_back(wp);
  
  if (client_wp.call(srv_wp))
  {
    // ok, check srv.response
    ROS_INFO("Uploaded WPs!");
    mavros_msgs::State current_state;
  }
  else
  {
    // roscpp error
    ROS_ERROR("Upload Failed");
  }
  ///////////////////////////////////////
  //set mission
  
  srv_setMode.request.base_mode = 0; //(uint8_t)mavros_msgs::SetModeRequest::MAV_MODE_AUTO_ARMED;
  srv_setMode.request.custom_mode = "AUTO.MISSION";

  if(cl.call(srv_setMode))
  {
    ROS_INFO("setmode send ok %d value:", srv_setMode.response.success);
  }
  else
  {
    ROS_ERROR("Failed SetMode");
    return -1;
  }
  ///////////////////////////////////////
   state=2;
 }
 
 //ROS_INFO("got current_rel_alt: %f",current_rel_alt.data);
   
  
  if((state==2)&&current_rel_alt.data>5.6)
  {
    /* mavros_msgs::SetMavFrame mav_frame_set;
     mav_frame_set.request.mav_frame=8;
     set_mav_frame_client.call(mav_frame_set);*/
    /////////////////////////////////////////////
    //////////////////////////////////////////////
     
     velocity_pub.publish(velocity_tw);
     srv_setMode.request.custom_mode = "OFFBOARD";
     cl.call(srv_setMode);
     state=3;
  }
  if (state==3) 
  //(abs(global_pose.latitude-47.398317)<0.0003)&&(abs(global_pose.longitude-8.5460425)<0.0003)))
  {
     
    /*/set_mav_frame
     mavros_msgs::SetMavFrame mav_frame_set;
     mav_frame_set.request.mav_frame=8;
     set_mav_frame_client.call(mav_frame_set);
    ////////////////////////////////////////////*/
    //srv_setMode.request.custom_mode = "OFFBOARD";
    //cl.call(srv_setMode);
     get_target=image_info.data[0];
     //get_target=0;  
     if(get_target==1)
     {
       vs_body_axis.twist.linear.x = 0.0004*(image_info.data[1]-160)*current_rel_alt.data;
       vs_body_axis.twist.linear.y = -0.0004*(image_info.data[2]-120)*current_rel_alt.data;
       vs_body_axis.twist.linear.z = -0.15;

       if(abs(image_info.data[1]-160)<10){
            vs_body_axis.twist.linear.x=0;
       }
       if(abs(image_info.data[2]-120)<10){
            vs_body_axis.twist.linear.y=0;
       } 
       velocity_tw.twist.linear.z = vs_body_axis.twist.linear.z;
       velocity_tw.twist.linear.x = vs_body_axis.twist.linear.x * cos(Uavyaw.data*PI/180.0)  + vs_body_axis.twist.linear.y * sin(Uavyaw.data*PI/180.0);
       velocity_tw.twist.linear.y = -vs_body_axis.twist.linear.x * sin(Uavyaw.data*PI/180.0) + vs_body_axis.twist.linear.y * cos(Uavyaw.data*PI/180.0);
     } 
     else
     { 
       velocity_tw.twist.linear.x=0;
       velocity_tw.twist.linear.y=0;
       velocity_tw.twist.linear.z=0.05;
       if(current_rel_alt.data<2)
       {
	 velocity_tw.twist.linear.z=-0.2;
       }     
     }
     velocity_pub.publish(velocity_tw);
     if(control_flag.data>150)
     {
        
	//state=4;
	ROS_INFO("control_flag::  %d",control_flag.data);
     }
     
     if((ros::Time::now() - last_request) > ros::Duration(50.0))
        state=4;
  }

    ROS_INFO("state::  %d",state);

  if(state==4)
  {
  state=5;
  wp_clear_client.call(wp_clear_srv);
  //target2:put load
  wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
  wp.command      = mavros_msgs::CommandCode::NAV_WAYPOINT;
  wp.is_current   = true;
  wp.autocontinue = true;
  wp.param1       = 5;          // hold time sec
  wp.param4       =NAN;//yaw
  wp.z_alt        = 3;
  wp.x_lat        = global_pose.latitude;
  wp.y_long       = global_pose.longitude;
  srv_wp.request.waypoints.push_back(wp);
 

  wp.is_current   = false;
  wp.param1       = 5;          // hold time sec
  wp.param4       =NAN;//yaw
  wp.z_alt        = 3;
  wp.x_lat        = target2.x;
  wp.y_long       = target2.y; 
  srv_wp.request.waypoints.push_back(wp);

  wp.is_current   = false;
  wp.param1       = 5;          // hold time sec
  wp.param4       =NAN;//yaw
  wp.z_alt        = 3;
  wp.x_lat        = target3.x;
  wp.y_long       = target3.y; 
  srv_wp.request.waypoints.push_back(wp);
  
  wp.command      = mavros_msgs::CommandCode::NAV_LAND;
  wp.param4       =NAN;//yaw
  wp.z_alt        = 0;
  wp.x_lat        = target3.x;//39.962769;
  wp.y_long       = target3.y;//116.304855;
  srv_wp.request.waypoints.push_back(wp);

  if (client_wp.call(srv_wp))
  {
    // ok, check srv.response
    ROS_INFO("Uploaded WPs!");
    mavros_msgs::State current_state;
  }
  else
  {
    // roscpp error
    ROS_ERROR("Upload Failed");
  }
  ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  mavros_msgs::SetMode srv_setMode;
  srv_setMode.request.base_mode = 0; //(uint8_t)mavros_msgs::SetModeRequest::MAV_MODE_AUTO_ARMED;
  srv_setMode.request.custom_mode = "AUTO.MISSION";

  if(cl.call(srv_setMode))
  {
    ROS_INFO("setmode send ok %d value:", srv_setMode.response.success);
  }
  else
  {
    ROS_ERROR("Failed SetMode");
    return -1;
  }

  }
  mission_state.data=state;
  state_pub.publish(mission_state);
 ////////////////////////////////////////////////////////////
 //call_cb    
      ros::spinOnce();
      rate.sleep();
   
  }


  /*wp.command      = mavros_msgs::CommandCode::NAV_WAYPOINT;
  wp.is_current   = false;
  wp.autocontinue = true;
  wp.param1       = 5;          // hold time sec
  wp.z_alt        = 5;
  wp.x_lat        = 47.3977436;
  wp.y_long       = 8.5456237;
  srv_wp.request.waypoints.push_back(wp);
  wp.x_lat        = 47.3983173;
  wp.y_long       = 8.5460421;
  srv_wp.request.waypoints.push_back(wp);
  wp.param1       = 5; 
  wp.z_alt        = 2;
  wp.x_lat        = 47.398317;
  wp.y_long       = 8.5460425;
  srv_wp.request.waypoints.push_back(wp);
  wp.z_alt        = 5;
  wp.x_lat        = 47.398317;
  wp.y_long       = 8.5460425;
  srv_wp.request.waypoints.push_back(wp);
  wp.command      = mavros_msgs::CommandCode::NAV_LAND;
  wp.z_alt        = 0;
  wp.x_lat        = 47.3972267;
  wp.y_long       = 8.5460425;
  srv_wp.request.waypoints.push_back(wp);
  //...
*/
  
 return 0;
}






