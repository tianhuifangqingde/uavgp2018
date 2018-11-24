#include "ros/ros.h"
#include "std_msgs/String.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "bg.h"
#include "removesmallregion.h"
#include "cutroi.h"


using namespace std;
using namespace cv;

 int a=0;
void readTxt(string file)
{
    ifstream infile; 
    infile.open(file.data());    
    assert(infile.is_open());   
    string s;
    while(getline(infile,s))
    {
	a= atoi(s.c_str());
    };
    infile.close();           
}

 template<typename T> string toString(const T& t){
    ostringstream oss;   
    oss<<t;             
    return oss.str();
}

int main(int argc, char **argv)
{
      
    ros::init(argc,argv,"dimage_process_H");
    ros::NodeHandle nh;
    ros::Publisher chatter_pub=nh.advertise<std_msgs::Int32MultiArray>("target_detection1",10);
    ros::Publisher keyboard_pub = nh.advertise<std_msgs::Int32> ("keyboard",10);
    ros::Rate loop_rate(20);
    int count=0;
    

   
    string save_video_name;
    readTxt("/home/odroid/uavgap/src/image_process/src/shipin.txt");
    stringstream ss;      
    a++;    
    ss << a;                  
    ss >> save_video_name;     
    save_video_name=save_video_name+".avi";
    fstream output_stream;
    output_stream.open("/home/odroid/uavgap/src/image_process/src/shipin.txt",ios::out | ios::app);
    output_stream << a<< endl;

 
    //VideoCapture capture("/home/odroid/uavgap/src/image_process/src/96.avi");
    VideoCapture capture(0);
    VideoWriter writer("/home/odroid/uavgap/src/image_process/src/"+save_video_name, CV_FOURCC('M', 'J', 'P', 'G'), 25.0, Size(320, 240));  
    while(ros::ok())
    {

	Mat frame;
        capture >>frame;
        if(frame.cols==0||frame.rows==0)
        {
            continue;
        } 
 
        resize(frame,frame,Size(320,240));
        writer << frame;  
 
        int kernel_size = 3;
	Mat kernel = Mat::ones(kernel_size,kernel_size,CV_32F)/(float)(kernel_size*kernel_size);
	Mat outputImage;
	Mat filterFrame;
	filter2D(frame,filterFrame,-1,kernel);
	colorTransform(filterFrame,outputImage);
	//FT(filterFrame,outputImage);
	  
	RemoveSmallRegion(outputImage,outputImage,100,1,1);
	chao_fillHole(outputImage,outputImage); 
        Rect target;int success=0;int angle=0; 
	cutROI(outputImage,frame,target,angle,success);
        putText(frame,"Pos:("+toString(target.x)+","+toString(target.y)+") Size:("+toString(target.width)+","+toString(target.height)+")", Point(target.x,target.y),CV_FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,255),2,0);
        imshow(save_video_name,frame);
	 
      
	std_msgs::Int32MultiArray msg; 
        msg.data.push_back(success);
        msg.data.push_back(target.x-(int)target.width/3);
 	msg.data.push_back(target.y-(int)target.width/4);
	msg.data.push_back(target.width);
 	msg.data.push_back(target.height);
        msg.data.push_back(angle);
 
	chatter_pub.publish(msg);
	ROS_INFO("%d  %d  %d  %d  %d  %d",msg.data[0],msg.data[1],msg.data[2],msg.data[3],msg.data[4],msg.data[5]);
	loop_rate.sleep();
	count++;

        int c = waitKey(1);
        if( c == 27  )
        {
           return 0;
        }
	std_msgs::Int32 key;
        key.data=c;
        keyboard_pub.publish(key);

        ros::spinOnce(); 
        loop_rate.sleep();
    }
 
	capture.release();
 
	return 0;
 
}