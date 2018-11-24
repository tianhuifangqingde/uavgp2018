#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>//解锁
#include <mavros_msgs/SetMode.h>//设置模式
#include <mavros_msgs/State.h>//订阅状态
#include <geometry_msgs/TwistStamped.h>
#include <string>
#include <sstream>
#include <iostream>
#include <std_msgs/Float64.h>//global position relative alt
#include <geometry_msgs/Vector3.h> //三个float64
#include <cmath>
#include<sensor_msgs/NavSatFix.h>
#include <serial/serial.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>

using namespace std;

#define Err_Limit_Flag 1
#define Integrate_Separation_Flag 1
#define Integrate_Limit_Flag 1

serial::Serial sp; //声明单片机串口对象

template <class Type>  //string转int/float
Type stringToNum(const string& str)  
{  
    istringstream iss(str);  
    Type num;  
    iss >> num;  
    return num;      
} 

void write_callback(const std_msgs::String::ConstPtr& msg) 
{ 
    ROS_INFO_STREAM("Writing to serial port" <<msg->data); 
    sp.write(msg->data);   //发送串口数据 
} 

//订阅状态回调函数
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
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
//laser
std_msgs::String current_laser;
std::string laser;
void get_laser_cb(const std_msgs::String::ConstPtr &msg)
{
    current_laser = *msg;
    laser = current_laser.data;
}

//位置控制器结构体
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
//PID
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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_control");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    
    //发布速度命令
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    //publish the alttitude    
    ros::Publisher alt_pub = nh.advertise<std_msgs::Float64>
            ("mavros/global_position/rel_alt", 10);
    //get gps
    ros::Subscriber get_gps_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 1, get_gps_cb);
	//get uwb    
    ros::Subscriber get_uwb_sub = nh.subscribe<geometry_msgs::Vector3>
            ("uwb_position", 1, get_location_cb);
    //gps pub
    ros::Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 10);
    
    //get laser
    ros::Subscriber get_laser_sub = nh.subscribe<std_msgs::String>
            ("get_laser", 1, get_laser_cb);

    std::string pump_port_name("/dev/ttyUSB0");

    try
    {
        sp.setPort(pump_port_name);/* code for Try */
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
    
    PID_Controler pos_pid_x;
    PID_Controler pos_pid_y;
    PID_Controler pos_pid_z;
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);//20Hz 50ms
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";//只是初始化，此时遥控一直给命令会覆盖
    std_msgs::Float64 alt;
    sensor_msgs::NavSatFix exp_gps;
    alt.data=2.5;
        /******x_pid参数(float)******/
    pos_pid_x.Kp = 1.0;
    pos_pid_x.Kd = 0.0;
    pos_pid_x.Ki = 0.0;//少加点就行
    pos_pid_x.Err_Max = 50.0;//误差限幅值
    pos_pid_x.Integrate_Max = 10.0;//积分限幅值
    pos_pid_x.Integrate_Separation_Err = 100.0;//积分分离值
    pos_pid_x.Control_OutPut_Limit = 1;//输出控制量限幅！！！很重要
    /******y_pid参数(float)******/
    pos_pid_y.Kp = 1.0;
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
    pos_pid_z.Control_OutPut_Limit = 0.3;//输出控制量限幅！！！很重要    
    
    //起飞点期望：只赋值一次
    pos_pid_x.Expect = current_pos.x;
    pos_pid_y.Expect = current_pos.y; 
    pos_pid_z.Expect = 300; //在地面时gps数+2.5

    //pid变量初始化
    pos_pid_x.Err = 0;
    pos_pid_y.Err = 0;
    
    pos_pid_z.Err = 0;
    ros::Time last_request = ros::Time::now();

    int state = 1;
    int number_check = 1;//接收吸盘状态
    geometry_msgs::Vector3 takeoff_pos;
    geometry_msgs::Vector3 target1;
    geometry_msgs::Vector3 target2;
    //起飞点只赋值１次
    takeoff_pos.x = current_pos.x;    //当前位置要摆放为起飞点
    takeoff_pos.y = current_pos.y;    //当前位置要摆放为起飞点
    takeoff_pos.z = 300;    //激光雷达距离单位是厘米
   //抓取点位置
    target1.x = 5.0; //需要现场测试
    target1.y = 0.0; //需要现场测试
    target1.z = 300;
    //摆放点位置
    target2.x = 5.0;
    target2.y = 5.0;
    target2.z = 300;


    //注：把自动解锁自动切外部控制放在循环外可避免切不回手动
/* if( !current_state.armed ) {
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
    last_request = ros::Time::now();
    while(ros::ok()) {
        /**定点悬停*

        //更新反馈值
        pos_pid_x.FeedBack = current_pos.x;
        pos_pid_y.FeedBack = current_pos.y;
        
	pos_pid_z.FeedBack = current_gps.altitude;  
        geometry_msgs::TwistStamped velocity_tw;
        velocity_tw.twist.linear.x = PID_Control(&pos_pid_x);
        velocity_tw.twist.linear.y = PID_Control(&pos_pid_y);
	
        velocity_tw.twist.linear.z = PID_Control(&pos_pid_z);
	//alt_pub.publish(alt);
	//gps_pub.publish(exp_gps);
        ROS_INFO("velocity_tw_x_y:%f %f", velocity_tw.twist.linear.x, velocity_tw.twist.linear.y);
	
	ROS_INFO("velocity_tw_z:%f", velocity_tw.twist.linear.z);
        velocity_pub.publish(velocity_tw);

        ROS_INFO_STREAM("location_x: " << current_pos.x);
        ROS_INFO_STREAM("location_y: " << current_pos.y);
        //ROS_INFO_STREAM("rel_alt: " << current_pos.z);
	ROS_INFO_STREAM("gps_alttitude: " << current_gps.altitude);*/

/*任务流程：1.起飞点起飞  2.飞到搬运点降落，发送吸气命令   
           3.自动起飞，接收洗盘命令，失败重新降落，重复2   
            4.飞至作业点，发送放气命令  
             5.重复2（时间判断&&电量判断）
              6.返航降落*/
//注意：任务之间必须加执行时间或者结束条件，否则会瞬间执行下一个外部控制;
    //任务1
    while(state == 1) {
        //ROS_INFO_STREAM("state: " << state);
 /*       offb_set_mode.request.custom_mode = "AUTO.TAKEOFF";
        if(set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success) {
                    ROS_INFO("takeoff enabled");
                    takeoff_pos = current_pos;
                    state++;
                    last_request = ros::Time::now();
                    while(ros::Time::now() - last_request < ros::Duration(5.0));
                }   */
       if(pos_pid_z.Err < 15) {
           state++;

       }
     
       pos_pid_z.Expect = 300; //在地面时gps数+2.5 
       pos_pid_z.FeedBack = stringToNum<float>(laser); 
       geometry_msgs::TwistStamped velocity_tw;
       velocity_tw.twist.linear.x = 0;
       velocity_tw.twist.linear.y = 0;
       velocity_tw.twist.linear.z = PID_Control(&pos_pid_z);
       
    }
    
    //任务2
    last_request = ros::Time::now();
    while(state == 2) {
         //更新反馈值
        pos_pid_x.FeedBack = current_pos.x;
        pos_pid_y.FeedBack = current_pos.y;
	    pos_pid_z.FeedBack = stringToNum<float>(laser);

        pos_pid_x.Expect = target1.x;
        pos_pid_y.Expect = target1.y;
        pos_pid_z.Expect = target1.z;  

        geometry_msgs::TwistStamped velocity_tw;
        velocity_tw.twist.linear.x = PID_Control(&pos_pid_x);
        velocity_tw.twist.linear.y = PID_Control(&pos_pid_y);
        velocity_tw.twist.linear.z = PID_Control(&pos_pid_z);

        velocity_pub.publish(velocity_tw);

        if((fabs(current_pos.x - target1.x) < 0.1) && (fabs(current_pos.y - target1.y) < 0.1)) {
             offb_set_mode.request.custom_mode = "AUTO.LAND";
             if(set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success) {
                    ROS_INFO("land enabled");
                    //发吸标志，等几秒
          uint8_t s_buffer[7];
          size_t sBUFFERSIZE=7;
          s_buffer[0]=0xaa;
          s_buffer[1]=0xaf; 
          s_buffer[2]=0x13;        
          s_buffer[3]=0x02;
          s_buffer[4]=0x01; //01为吸
          s_buffer[5]=0x01; //01为吸
          s_buffer[6]=s_buffer[0]+ s_buffer[1]+ s_buffer[2] +s_buffer[3]+s_buffer[4]+s_buffer[5];//s_buffer[4]^s_buffer[5]; 
          sp.write(s_buffer,sBUFFERSIZE);
          ROS_INFO_STREAM("sent suck command enabled"); 
                    state++;
             }
        }
    }
    //任务3
    if(state == 3) {
        offb_set_mode.request.custom_mode = "AUTO.TAKEOFF";
        if(set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success) {
                    ROS_INFO("takeoff enabled");
                    state++;
                }   
    }
    if(number_check == 0)  state = 2;  
    //任务4
    while(state == 4) {
         //更新反馈值
        pos_pid_x.FeedBack = current_pos.x;
        pos_pid_y.FeedBack = current_pos.y;
	    pos_pid_z.FeedBack = stringToNum<float>(laser);

        pos_pid_x.Expect = target2.x;
        pos_pid_y.Expect = target2.y;
        pos_pid_z.Expect = target2.z;   

        geometry_msgs::TwistStamped velocity_tw;
        velocity_tw.twist.linear.x = PID_Control(&pos_pid_x);
        velocity_tw.twist.linear.y = PID_Control(&pos_pid_y);
        velocity_tw.twist.linear.z = PID_Control(&pos_pid_z);

        velocity_pub.publish(velocity_tw);

        if((fabs(current_pos.x - target2.x) < 0.1) && (fabs(current_pos.y - target2.y) < 0.1)) {
                    
//发标志，等几秒
          uint8_t s_buffer[7];
          size_t sBUFFERSIZE=7;
          s_buffer[0]=0xaa;
          s_buffer[1]=0xaf; 
          s_buffer[2]=0x13;        
          s_buffer[3]=0x02;
          s_buffer[4]=0x00; //00为放
          s_buffer[5]=0x00; //00为放
          s_buffer[6]=s_buffer[0]+ s_buffer[1]+ s_buffer[2] +s_buffer[3]+s_buffer[4]+s_buffer[5];//s_buffer[4]^s_buffer[5]; 
          sp.write(s_buffer,sBUFFERSIZE);
          ROS_INFO_STREAM("sent put command enabled"); 

                    state = 2;
        }
    }   
    //任务5
    while(state == 2) {
        offb_set_mode.request.custom_mode = "AUTO.LAND";
        if(set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success) {
                    ROS_INFO("land enabled");
                    while(ros::Time::now() - last_request < ros::Duration(5.0)) ;  //判断小于５秒？
                    state++;
                }   
    }
    
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}