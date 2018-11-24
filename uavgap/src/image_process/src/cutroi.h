#ifndef CUTROI_H
#define CUTROI_H


#include<opencv2/opencv.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<iostream>
#include <fstream>
using namespace std;
using namespace cv;
long currentFrame;
int target_pos_x=0;int target_pos_y=0;int target_width=0;int target_height=0;

double getDistance (CvPoint point1,CvPoint point2 )  
{  
    double distance;  
    distance = powf((point1.x - point2.x),2) + powf((point1.y - point2.y),2);  
    distance = sqrtf(distance);  
  
    return distance;  
}  

 


void cutROI(Mat & inputImage,Mat & currentframe,Rect &target,int &angle,int &success)
{
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    Mat image = inputImage.clone();
    findContours( image,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE,Point(0,0));
    vector<vector<Point> > contours_poly( contours.size());
    vector<Rect> boundRect (contours.size());
    Point2f rect[4];
    vector<Mat> image_roi_ (contours.size());
    vector<RotatedRect> box(contours.size());

    for(unsigned int i = 0;i < contours.size();i++)
    {
        approxPolyDP(Mat(contours[i]),contours_poly[i],3,true);
        boundRect[i] = boundingRect(Mat(contours_poly[i]));
        image_roi_[i] = inputImage(boundRect[i]);
    }
    //??????
    if( contours.size()>0)
    {

        double maxarea = 0;
        int maxAreaIdx = 0;
        for (int index = contours.size() - 1; index >= 0; index--)
        {
            double tmparea = fabs(contourArea(contours[index]));
            if (tmparea>maxarea)
            {
                maxarea = tmparea;
                maxAreaIdx = index;//??????????
            }
        }
        box[maxAreaIdx] = minAreaRect(Mat(contours[maxAreaIdx]));
        box[maxAreaIdx].points(rect);

        if(contourArea(contours[maxAreaIdx]) > 200 )
        {  
            angle=box[maxAreaIdx].angle;
            target_pos_x=0; target_pos_y=0; 
            for (int j=0;j<4;j++)
            {
                line(currentframe,rect[j],rect[(j+1)%4],Scalar(0,0,255),2,8,0);
                target_pos_x=target_pos_x+rect[j].x;
		target_pos_y=target_pos_y+rect[j].y;
            }
            Point2f point1,point2; 
            point1= rect[0];
            if(getDistance(point1,rect[1])>getDistance(point1,rect[3]))
            {
	      point2=rect[1];
	    }
	    else
   	    {
	      point2=rect[3];
	    }
            float k=-(point2.y-point1.y)/(point2.x-point1.x);
            angle=atan(k)*180/3.141592654;

        
 
	    circle(currentframe, point1, 10, Scalar(0, 255, 0));//green	
            circle(currentframe, point2, 10, Scalar(0, 0, 255));//red


            success=1;
            target.x=target_pos_x/4;
 	    target.y=target_pos_y/4;
            target.width=floor(sqrt((rect[0].x-rect[1].x)*(rect[0].x-rect[1].x)+(rect[0].y-rect[1].y)*(rect[0].y-rect[1].y)));
            target.height=floor(sqrt((rect[1].x-rect[2].x)*(rect[1].x-rect[2].x)+(rect[1].y-rect[2].y)*(rect[1].y-rect[2].y)));
       	 }
	}
        else
        {
            success=0;
            angle=0;
            target.x=0;
 	    target.y=0;
            target.width=0;
            target.height=0;
        }
       
}



#endif // CUTROI_H
