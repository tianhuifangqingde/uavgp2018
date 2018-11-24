#ifndef BG_H
#define BG_H


#include<opencv2/opencv.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<iostream>

using namespace std;
using namespace cv;

void colorTransform(Mat& inputImage,Mat& outputImage)
{
    int rowNumber = inputImage.rows;
    int colNumber = inputImage.cols;
    outputImage.create(rowNumber,colNumber,CV_8UC1);
    double b_div_g;
    const double e = 1;

    for(int i = 0;i < rowNumber;i++)
    {
        for(int j = 0;j < colNumber;j++)
        {
            if(inputImage.at<Vec3b>(i,j)[0] > inputImage.at<Vec3b>(i,j)[1] && inputImage.at<Vec3b>(i,j)[0] > inputImage.at<Vec3b>(i,j)[2] && inputImage.at<Vec3b>(i,j)[0] >=50)
            {
	    
        b_div_g = double(100*(inputImage.at<Vec3b>(i,j)[0])/((inputImage.at<Vec3b>(i,j)[1])+e));  //B/G，找lan色

                 b_div_g = cvRound(b_div_g);
            }
            else
                b_div_g = 0;
        if(b_div_g>255)
            b_div_g=255;
         outputImage.at<uchar>(i,j) = b_div_g;



        }
    }

    //cout<<"R/G中最大R/G值"<<max_t<<"，"<<"最小R/G值"<<min_t<<endl;
    threshold(outputImage,outputImage,120,255,THRESH_BINARY);
    //adaptiveThreshold(outputImage,outputImage,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,7,0);
    //imshow("two_value",outputImage);
  
    
    
}

void FT(Mat &src,Mat &outputImage ){
    int iLowH = 100; int iHighH = 140; int iLowS = 90; int iHighS = 255; int iLowV = 90; int iHighV = 255;//ÉèÖÃÀ¶É«µÄÑÕÉ«²ÎÁ¿¡£

    outputImage.create(src.rows,src.cols,CV_8UC1);

    vector<Mat> hsvSplit;
    cvtColor(src, outputImage, COLOR_BGR2HSV);  //ÒòÎªÎÒÃÇ¶ÁÈ¡µÄÊÇ²ÊÉ«Í¼£¬Ö±·½Í¼¾ùºâ»¯ÐèÒªÔÚHSV¿Õ¼ä×ö

    split(outputImage, hsvSplit);

    equalizeHist(hsvSplit[2],hsvSplit[2]);

    merge(hsvSplit,outputImage);



    inRange(outputImage, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), outputImage);  //¿ª²Ù×÷ (È¥³ýÒ»Ð©Ôëµã)

    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));

    morphologyEx(outputImage, outputImage, MORPH_OPEN, element); //±Õ²Ù×÷ (Á¬½ÓÒ»Ð©Á¬Í¨Óò)

    morphologyEx(outputImage, outputImage, MORPH_CLOSE, element);

    
    GaussianBlur(outputImage,outputImage, Size(3, 3), 0, 0);
    threshold(outputImage,outputImage,140,255,THRESH_BINARY);
    //imshow("ÂË²¨ºóµÄÍ¼Ïñ", outputImage);


}

#endif // BG_H
