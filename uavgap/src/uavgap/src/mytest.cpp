#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <string>
#include <sstream>
#include <iostream>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <geometry_msgs/Vector3.h> //三个float64
#include <cmath>
#include<sensor_msgs/NavSatFix.h>
#include<std_msgs/ByteMultiArray.h>
#include<mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
/************************************串口**************************************/
serial::Serial sp; //声明串口对象


/************************************PID****************************************/
#define Err_Limit_Flag 0
#define Integrate_Separation_Flag 0
#define Integrate_Limit_Flag 0
typedef struct
{
    float Expect;//期望 
    float FeedBack;//反馈
    float Err;//误差
    float Last_Err;//上一时刻误差 
    float Err_Max;//误差限幅值 
    float Integrate_Separation_Err;//积分分离偏差值     先设100把 
    float Integrate;//积分值 
    float Integrate_Max;//积分限幅值 
    float Kp;
    float Ki;
    float Kd;
    float Control_OutPut;//控制器输出 
    float Last_Control_OutPut;//上一时刻控制器输出 
    float Control_OutPut_Limit;//输出限幅 

    float Last_FeedBack;//上一时刻反馈值 
    float Dis_Err;//微分量 
    float Dis_Error_History[5];//历史微分量 
}PID_Controler;

float PID_Control(PID_Controler *Controler)
{
    /*偏差计算*/ 
    Controler->Last_Err = Controler->Err;//保存上一时刻误差 
    Controler->Err = Controler->Expect - Controler->FeedBack;//计算当前误差 
    if(Err_Limit_Flag == 1) {//误差限幅标志位 （航点控制时不加误差限幅）
        if(Controler->Err >= Controler->Err_Max)   Controler->Err = Controler->Err_Max;
        if(Controler->Err <= -Controler->Err_Max)  Controler->Err = -Controler->Err_Max;
    }
    /*积分计算*/ 
    if(Integrate_Separation_Flag==1) {//积分分离标志位 
        if(fabs(Controler->Err) <= Controler->Integrate_Separation_Err)//abs->int fabs->float
            Controler->Integrate += Controler->Ki * Controler->Err;
    }
    else {
        Controler->Integrate += Controler->Ki * Controler->Err;
    }
    /*积分限幅*/ 
    if(Integrate_Limit_Flag==1) {//积分限幅标志位 
        if(Controler->Integrate >= Controler->Integrate_Max)
            Controler->Integrate = Controler->Integrate_Max;
         if(Controler->Integrate <= -Controler->Integrate_Max)
            Controler->Integrate = -Controler->Integrate_Max ;
    }
    /*总输出*/
    Controler->Last_Control_OutPut = Controler->Control_OutPut;//保存上一时刻输出值 
    Controler->Control_OutPut = Controler->Kp * Controler->Err//P 
                              + Controler->Integrate//I
                              + Controler->Kd*(Controler->Err-Controler->Last_Err);//D
    /*总输出限幅*/
    if(Controler->Control_OutPut >= Controler->Control_OutPut_Limit)
        Controler->Control_OutPut = Controler->Control_OutPut_Limit;
    if(Controler->Control_OutPut <= -Controler->Control_OutPut_Limit)
        Controler->Control_OutPut = -Controler->Control_OutPut_Limit;
    /*返回输出值*/
    return Controler->Control_OutPut;
}
PID_Controler pos_pid_x;
PID_Controler pos_pid_y;
PID_Controler pos_pid_z;
/*******************************回调函数******************************************/
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
geometry_msgs::PoseStamped current_local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_local_pos = *msg;
}

/*订阅UWB位置回调函数
geometry_msgs::Vector3 current_pos;
void get_location_cb(const geometry_msgs::Vector3::ConstPtr &msg) 
{
    current_pos = *msg;
}
*/
//gps
sensor_msgs::NavSatFix current_gps;
void get_gps_cb(const sensor_msgs::NavSatFix::ConstPtr &msg) 
{
    current_gps = *msg;
    //ROS_INFO("%f %f %f",current_gps.longitude,current_gps.latitude,current_gps.altitude);

}
//
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "holdheight");
    ros::NodeHandle nh;
    //订阅状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    //发布local位置
    ros::Publisher set_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    //接收local位置
    ros::Subscriber local_pos_pub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_pos_cb);
    //发布global位置
    ros::Publisher  global_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
            ("mavros/setpoint_position/global", 10);
    //解锁上锁
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    //设置模式
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    //发布local速度(惯性系)
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    //发布接收到的吸盘命令(发送格式要改!!!!!!!!!!!!!!!!)
    ros::Publisher dis_pub = nh.advertise<std_msgs::UInt32>("absorb_cmd", 1);
    /*订阅uwb位置
    ros::Subscriber get_uwb_sub = nh.subscribe<geometry_msgs::Vector3>
            ("uwb_position", 1, get_location_cb);
     */	    
    //get gps
    ros::Subscriber get_gps_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 1, get_gps_cb);
  
    ros::Rate rate(20.0);
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }    
     //设置起飞点，抓箱点，放箱点
    geometry_msgs::TwistStamped speed;
    speed.twist.linear.x=0;
    speed.twist.linear.y=0.2;
    speed.twist.linear.z=0;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    //mavros_msgs::CommandBool arm_cmd;
    //arm_cmd.request.value = true;
    //全局位置test
    mavros_msgs::GlobalPositionTarget target;
    target.latitude=current_gps.latitude;
    target.longitude=current_gps.longitude;
    target.altitude=4.0;
    target.coordinate_frame=6;
    target.type_mask=504; 
    target.header=current_gps.header;  
    //target.velocity.x=0.5;
////////////////////////////////////////////
/////////////////CLEAR MISSION/////////////////
////////////////////////////////////////////


  ros::ServiceClient wp_clear_client = nh.serviceClient<mavros_msgs::WaypointClear>("/mavros/mission/clear");
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
    //local位置test
    geometry_msgs::PoseStamped poz;
     poz.pose.position.x=0;
     poz.pose.position.y=2;
     poz.pose.position.z=3;
    //注：把自动解锁自动切外部控制放在循环外可避免切不回手动
    //问题：自动解算自动offb放在while外面时，while里takeoff可以，但是位置控制不可以。位置控制时必须要在offb？
    ros::Time last_request = ros::Time::now();
    int cnt=0;
    int flag=0;
    while(ros::ok()) {
    //自动解锁切offb
     /*       if( !current_state.armed ) {
        if( arming_client.call(arm_cmd) &&
              arm_cmd.response.success) {
            ROS_INFO("Vehicle armed");
        }
    }
    if( current_state.mode != "OFFBOARD") {
        if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success) {
            ROS_INFO("Offboard enabled");
        }
    } */
     if(flag<10) 
      {target.latitude=current_gps.latitude;
      target.longitude=current_gps.longitude;
      target.altitude=4; 
      flag++;
      }   
     // global_pos_pub.publish(target);
       //set_local_pos_pub.publish(poz);
        velocity_pub.publish(speed);
       ros::spinOnce();
       rate.sleep(); 
       //cnt++;
       if(ros::Time::now()-last_request>ros::Duration(35.0))
       break;
       ROS_INFO("%f %f %f",current_gps.longitude,current_gps.latitude,current_gps.altitude);
       ROS_INFO("z: %f",target.altitude);
       ROS_INFO("x: %f",target.latitude);
       ROS_INFO("y: %f",target.longitude);
    }
offb_set_mode.request.custom_mode = "AUTO.LAND";
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
    {
        ROS_INFO("AUTO.LAND enabled");
        last_request = ros::Time::now();
    }
    return 0;
}