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

//订阅UWB位置回调函数
geometry_msgs::Vector3 current_pos;
void get_location_cb(const geometry_msgs::Vector3::ConstPtr &msg) 
{
    current_pos = *msg;
}
//gps
sensor_msgs::NavSatFix current_gps;
void get_gps_cb(const sensor_msgs::NavSatFix::ConstPtr &msg) 
{
    current_gps = *msg;
}
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
    //订阅uwb位置
    ros::Subscriber get_uwb_sub = nh.subscribe<geometry_msgs::Vector3>
            ("uwb_position", 1, get_location_cb);
    //get gps
    ros::Subscriber get_gps_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 1, get_gps_cb);
  
   std::string cmd_port_name("/dev/ttyUSB0");
   try
    {
        sp.setPort(cmd_port_name);
        sp.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        sp.setTimeout(to);
        sp.open();
    }
    catch (serial::IOException& e)
    {
       ROS_ERROR_STREAM("Unable to open port "); 
       return -1;  
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
    
    ros::Rate rate(20.0);
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    /******x_pid参数(float)******/
    pos_pid_x.Kp = 0.6;
    pos_pid_x.Kd = 0.0;
    pos_pid_x.Ki = 0.0;//少加点就行
    pos_pid_x.Err_Max = 50.0;//误差限幅值
    pos_pid_x.Integrate_Max = 10.0;//积分限幅值
    pos_pid_x.Integrate_Separation_Err = 100.0;//积分分离值
    pos_pid_x.Control_OutPut_Limit = 1;//输出控制量限幅！！！很重要

    /******y_pid参数(float)******/
    pos_pid_y.Kp = 0.6;
    pos_pid_y.Kd = 0.0;
    pos_pid_y.Ki = 0.0;
    pos_pid_y.Err_Max = 50.0;//误差限幅值
    pos_pid_y.Integrate_Max = 10.0;//积分限幅值
    pos_pid_y.Integrate_Separation_Err = 100.0;//积分分离值
    pos_pid_y.Control_OutPut_Limit = 1;//输出控制量限幅！！！很重要

    /******z_pid参数(float)******/
    pos_pid_z.Kp = 0.5;
    pos_pid_z.Kd = 0.0;
    pos_pid_z.Ki = 0.0;
    pos_pid_z.Err_Max = 50.0;//误差限幅值
    pos_pid_z.Integrate_Max = 10.0;//积分限幅值
    pos_pid_z.Integrate_Separation_Err = 100.0;//积分分离值
    pos_pid_z.Control_OutPut_Limit = 0.6;//输出控制量限幅！！！很重要
  
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
    int watch_gps=10;
    while(watch_gps-- )
   { ROS_INFO_STREAM("gps_alttitude: " << current_gps.altitude);
     sp.write(s_buffer_dis,sBUFFERSIZE);
     }
    //设置起飞点，抓箱点，放箱点
    struct point{
        float x;
        float y;
        float z; }takeoff_pos,target1,target2;
    float take_off_initial = 52.7;
    ros::param::get("take_off_initial", take_off_initial);
    ros::param::get("target1x", target1.x);
    ros::param::get("target1y", target1.y);
    ros::param::get("target2x", target2.x);
    ros::param::get("target2y", target2.y);
    takeoff_pos.x = 1.6;
    takeoff_pos.y = 1.6;
    takeoff_pos.z = take_off_initial+2.5;
    //target1.x = 3.80;
    //target1.y =4.56;
    target1.z = take_off_initial+2.5;
   // target2.x = 6.0;
   //target2.y = 3.0;
    target2.z = take_off_initial+2.5;
    ROS_INFO("Record Point1:[%f %f]", target1.x, target1.y);
    ROS_INFO("Record Point2:[%f %f]", target2.x, target2.y);

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    //mavros_msgs::CommandBool arm_cmd;
    //arm_cmd.request.value = true;
    int state = 1;
    int cnt = 2;//搬运次数
   // sp.write(s_buffer_dis,sBUFFERSIZE);//test pumb
    //注：把自动解锁自动切外部控制放在循环外可避免切不回手动
    //问题：自动解算自动offb放在while外面时，while里takeoff可以，但是位置控制不可以。位置控制时必须要在offb？
    ros::Time last_request = ros::Time::now();
    while(ros::ok() && state == 1) {
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
        ROS_INFO_STREAM("state = " << state);
//        sp.write(s_buffer_absorb,sBUFFERSIZE);//test pumb
        pos_pid_x.Expect = takeoff_pos.x;
        pos_pid_y.Expect = takeoff_pos.y;
        pos_pid_z.Expect = takeoff_pos.z;
        pos_pid_x.FeedBack = current_pos.x;
        pos_pid_y.FeedBack = current_pos.y;//y的反馈值始终有偏差？？？
        pos_pid_z.FeedBack = current_gps.altitude;
        geometry_msgs::TwistStamped velocity_tw;
        velocity_tw.twist.linear.x = PID_Control(&pos_pid_x);
        velocity_tw.twist.linear.y = PID_Control(&pos_pid_y);
        velocity_tw.twist.linear.z = PID_Control(&pos_pid_z);
        ROS_INFO("%f %f %f" ,pos_pid_x.Expect ,pos_pid_y.Expect ,pos_pid_z.Expect);
        ROS_INFO("%f %f %f" ,current_pos.x,current_pos.y ,current_gps.altitude);
        ROS_INFO("%f %f %f" ,pos_pid_x.FeedBack ,pos_pid_y.FeedBack ,pos_pid_z.FeedBack);
        ROS_INFO("%f %f %f" ,pos_pid_x.Err ,pos_pid_y.Err ,pos_pid_z.Err);
        ROS_INFO("%f %f %f" ,PID_Control(&pos_pid_x) ,PID_Control(&pos_pid_y) ,PID_Control(&pos_pid_z));
        velocity_pub.publish(velocity_tw);
        if(pos_pid_z.Err < 0.15) {
           state++;
        }
        ros::spinOnce();
        rate.sleep();
    }
    while(cnt--) {
        //到搬运点
        last_request = ros::Time::now();
        while(ros::ok() && state == 2) {
            ROS_INFO_STREAM("state = " << state);
            pos_pid_x.Expect = target1.x;
            pos_pid_y.Expect = target1.y;
            pos_pid_z.Expect = target1.z;
            pos_pid_x.FeedBack = current_pos.x;
            pos_pid_y.FeedBack = current_pos.y;
            pos_pid_z.FeedBack = current_gps.altitude;
            geometry_msgs::TwistStamped velocity_tw;
            velocity_tw.twist.linear.x = PID_Control(&pos_pid_x);
            velocity_tw.twist.linear.y = PID_Control(&pos_pid_y);
            velocity_tw.twist.linear.z = PID_Control(&pos_pid_z);
            ROS_INFO("%f %f %f" ,pos_pid_x.Expect ,pos_pid_y.Expect ,pos_pid_z.Expect);
            ROS_INFO("%f %f %f" ,current_pos.x ,current_pos.y ,current_gps.altitude);
            ROS_INFO("%f %f %f" ,pos_pid_x.FeedBack ,pos_pid_y.FeedBack ,pos_pid_z.FeedBack);
            ROS_INFO("%f %f %f" ,pos_pid_x.Err ,pos_pid_y.Err ,pos_pid_z.Err);
            ROS_INFO("%f %f %f" ,PID_Control(&pos_pid_x) ,PID_Control(&pos_pid_y) ,PID_Control(&pos_pid_z));
            velocity_pub.publish(velocity_tw);
            if((pos_pid_x.Err < 0.15) && (pos_pid_y.Err < 0.15) && (pos_pid_z.Err < 0.15) && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                state++;
            }
                
            ros::spinOnce();
            rate.sleep();
        }
        //搬运点降落
        last_request = ros::Time::now();
        while(ros::ok() && state == 3){
            ROS_INFO_STREAM("state = " << state);
            pos_pid_x.Expect = target1.x;
            pos_pid_y.Expect = target1.y;
            pos_pid_z.Expect = target1.z-4.5;
            pos_pid_x.FeedBack = current_pos.x;
            pos_pid_y.FeedBack = current_pos.y;
            pos_pid_z.FeedBack = current_gps.altitude;
          
           
            geometry_msgs::TwistStamped velocity_tw;
            velocity_tw.twist.linear.x = PID_Control(&pos_pid_x);
            velocity_tw.twist.linear.y = PID_Control(&pos_pid_y);
            velocity_tw.twist.linear.z = PID_Control(&pos_pid_z);
            ROS_INFO("%f %f %f" ,pos_pid_x.Expect ,pos_pid_y.Expect ,pos_pid_z.Expect);
            ROS_INFO("%f %f %f" ,current_pos.x ,current_pos.y ,current_gps.altitude);
            ROS_INFO("%f %f %f" ,pos_pid_x.FeedBack ,pos_pid_y.FeedBack ,pos_pid_z.FeedBack);
            ROS_INFO("%f %f %f" ,pos_pid_x.Err ,pos_pid_y.Err ,pos_pid_z.Err);
            ROS_INFO("%f %f %f" ,PID_Control(&pos_pid_x) ,PID_Control(&pos_pid_y) ,PID_Control(&pos_pid_z));
            velocity_pub.publish(velocity_tw);
            if(ros::Time::now() - last_request > ros::Duration(10.0)) {//haiyaojiaxianzhi!!!
                state++;
		int cnt_test = 5;
		while(cnt_test--)
	        {sp.write(s_buffer_absorb,sBUFFERSIZE);}
            }
            ros::spinOnce();
            rate.sleep();
        }
        last_request = ros::Time::now();
        //搬运点起飞
        while(ros::ok() && state == 4) {
            ROS_INFO_STREAM("state = " << state);
            pos_pid_x.Expect = target1.x;
            pos_pid_y.Expect = target1.y;
            pos_pid_z.Expect = target1.z;
            pos_pid_x.FeedBack = current_pos.x;
            pos_pid_y.FeedBack = current_pos.y;
            pos_pid_z.FeedBack = current_gps.altitude;


            geometry_msgs::TwistStamped velocity_tw;
            velocity_tw.twist.linear.x = PID_Control(&pos_pid_x);
            velocity_tw.twist.linear.y = PID_Control(&pos_pid_y);
            velocity_tw.twist.linear.z = PID_Control(&pos_pid_z);
            ROS_INFO("%f %f %f" ,pos_pid_x.Expect ,pos_pid_y.Expect ,pos_pid_z.Expect);
            ROS_INFO("%f %f %f" ,current_pos.x ,current_pos.y ,current_gps.altitude);
            ROS_INFO("%f %f %f" ,pos_pid_x.FeedBack ,pos_pid_y.FeedBack ,pos_pid_z.FeedBack);
            ROS_INFO("%f %f %f" ,pos_pid_x.Err ,pos_pid_y.Err ,pos_pid_z.Err);
            ROS_INFO("%f %f %f" ,PID_Control(&pos_pid_x) ,PID_Control(&pos_pid_y) ,PID_Control(&pos_pid_z));
            velocity_pub.publish(velocity_tw);
            if(ros::Time::now() - last_request > ros::Duration(7.0)) {
                state++;
            }
            ros::spinOnce();
            rate.sleep();
        }
        //到投递点
        last_request = ros::Time::now();
        while(ros::ok() && state == 5) {
            ROS_INFO_STREAM("state = " << state);
            pos_pid_x.Expect = target2.x;
            pos_pid_y.Expect = target2.y;
            pos_pid_z.Expect = target2.z;
            pos_pid_x.FeedBack = current_pos.x;
            pos_pid_y.FeedBack = current_pos.y;
            pos_pid_z.FeedBack = current_gps.altitude;


           
            geometry_msgs::TwistStamped velocity_tw;
            velocity_tw.twist.linear.x = PID_Control(&pos_pid_x);
            velocity_tw.twist.linear.y = PID_Control(&pos_pid_y);
            velocity_tw.twist.linear.z = PID_Control(&pos_pid_z);
            ROS_INFO("%f %f %f" ,pos_pid_x.Expect ,pos_pid_y.Expect ,pos_pid_z.Expect);
            ROS_INFO("%f %f %f" ,current_pos.x ,current_pos.y ,current_gps.altitude);
            ROS_INFO("%f %f %f" ,pos_pid_x.FeedBack ,pos_pid_y.FeedBack ,pos_pid_z.FeedBack);
            ROS_INFO("%f %f %f" ,pos_pid_x.Err ,pos_pid_y.Err ,pos_pid_z.Err);
            ROS_INFO("%f %f %f" ,PID_Control(&pos_pid_x) ,PID_Control(&pos_pid_y) ,PID_Control(&pos_pid_z));
            velocity_pub.publish(velocity_tw);
            if((pos_pid_x.Err < 0.15) && (pos_pid_y.Err < 0.15)&& (ros::Time::now() - last_request > ros::Duration(5.0))) {
		int cnt_test = 5;
		while(cnt_test--)
		{sp.write(s_buffer_dis,sBUFFERSIZE);
		}
		state=2;
            }
            ros::spinOnce();
            rate.sleep();
        }

    }
offb_set_mode.request.custom_mode = "AUTO.LAND";
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
    {
        ROS_INFO("AUTO.LAND enabled");
        last_request = ros::Time::now();
    }
    return 0;
}