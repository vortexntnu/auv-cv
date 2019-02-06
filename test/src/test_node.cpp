//
//  main.cpp
//  CV
//
//  Created by Thomas Hellum on 22/10/2018.
//  Copyright © 2018 Thomas Hellum. All rights reserved.
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm> 
#include "std_msgs/Bool.h"
//#include "std_msgs/String.h"
#include <sstream>

using namespace cv;
using namespace std;


Mat convert_frame(cv_bridge::CvImagePtr frame);


//Video
Mat frame; 
Mat frame_converted;

static const std::string OPENCV_WINDOW = "Image window";
/*
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher detect_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image", 1,
      &ImageConverter::imageCb, this);
    detect_pub_ = nh_.advertise<std_msgs::Bool>("detected_line", 1000);

    namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
		
		///////////////////////// CV ////////////////////////////

		/// Convert color of image
		frame_converted = convert_frame(cv_ptr);			

		///////////////////////// END CV ////////////////////////////

		
    // Update GUI Window
		imshow(OPENCV_WINDOW, frame_converted);
    waitKey(3);

  }
};
*/

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher detect_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image", 1,
      &ImageConverter::imageCb, this);
    detect_pub_ = nh_.advertise<std_msgs::Bool>("detected_line", 1000);

    namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
		
		///////////////////////// CV ////////////////////////////

    int iLowH = 0;
    int iHighH = 179;

    int iLowS = 0; 
    int iHighS = 255;

    int iLowV = 0;
    int iHighV = 255;

		/// Convert color of image
		//frame_converted = convert_frame(cv_ptr);
    
    Mat imgOriginal;
    Mat imgHSV;

    //cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
    imgHSV = convert_frame(cv_ptr);

    Mat imgThresholded;

    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

    //morphological opening (remove small objects from the foreground)
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

    //morphological closing (fill small holes in the foreground)
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    namedWindow( "Thresholded Image", CV_WINDOW_AUTOSIZE);
    namedWindow( "Original", CV_WINDOW_AUTOSIZE);
    resizeWindow("Original", 800, 600);
    resizeWindow("Threshold Image", 800, 600);

    imshow("Thresholded Image", imgThresholded); //show the thresholded image
    imshow("Original", imgOriginal); //show the original image
  	

		///////////////////////// END CV ////////////////////////////

		
    // Update GUI Window
		//imshow(OPENCV_WINDOW, frame_converted);
    waitKey(3);

  }
};



Mat convert_frame(cv_bridge::CvImagePtr frame) 
{
	Mat canny_output, kernel, frame_converted;

	cvtColor(frame->image, frame_converted, COLOR_BGR2HSV);
	//inRange(frame_converted, Scalar(0,0,50), Scalar(20,255,255), frame_converted);
  //inRange(frame_converted, Scalar(160,0,60), Scalar(180,255,255), red2);
  //GaussianBlur( frame_converted, frame_converted, Size(9,9), 0, 0);
	//Canny( frame_converted, canny_output, 10, 50, 3 );
	//kernel = Mat::ones(3, 3, CV_32F); //evt CV_8UC3
	//dilate(canny_output, frame_converted, kernel);

	return frame_converted;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}