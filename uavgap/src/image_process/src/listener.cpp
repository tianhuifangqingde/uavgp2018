#include "ros/ros.h"
#include <iostream>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"

using namespace std;

void catterCallback(const std_msgs::Int32MultiArray::ConstPtr& array)
{
 
   ROS_INFO("%d  %d  %d  %d  %d", array->data[0], array->data[1], array->data[2], array->data[3]
, array->data[4] );
 
}

void catterCallback2(const std_msgs::Int32::ConstPtr& array)
{
 
   ROS_INFO("%d", array->data);
 
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"listener");
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe("target_detection1",10,catterCallback);
    ros::Subscriber sub2=nh.subscribe("keyboard",10,catterCallback2);
    ros::spin();
    return 0;
}