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
#include "std_msgs/Int32.h"
//#include "std_msgs/Bool.h"
//#include "std_msgs/String.h"
#include <sstream>

using namespace cv;
using namespace std;

Mat blur(cv_bridge::CvImagePtr frame);
Mat convert_frame(cv_bridge::CvImagePtr frame);
vector<Vec4i> filter_lines(vector<Vec4i> lines);
vector<Vec4i> insertion_sort_lines(vector<Vec4i> lines);

vector<Vec4i> lines_filtered;
vector<Vec4i> lines_sorted;

//Video
Mat frame, frame_converted, frame_blurred;
Rect2d bbox;

static const std::string OPENCV_WINDOW = "Image window";

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
		detect_pub_ = nh_.advertise<std_msgs::Int32>("detected_color", 1000);


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

		int largest_area = 0;
		int largest_contour_index = 0;

		/// Convert color of image
		frame_converted = convert_frame(cv_ptr);
		frame_blurred = blur(cv_ptr);		

		vector<vector<Point> > contours;
    findContours(frame_converted, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		for ( int i = 0; i< contours.size(); i++ ) // iterate through each contour. 
    {
      double area = contourArea(contours[i]);  //  Find the area of contour
      if( area > largest_area){
      	largest_area = area;
      	largest_contour_index = i;                //Store the index of largest contour
				
				// Find the bounding rectangle for biggest contour
				bbox = boundingRect(contours[largest_contour_index]);

    	}
    }

		///////////////////////// END CV ////////////////////////////

		
    // Update GUI Window
		rectangle(frame_blurred, bbox, Scalar(0,255,0), 2, 1); 
		imshow(OPENCV_WINDOW, frame_blurred); 
		//imshow(OPENCV_WINDOW, frame_converted);
    waitKey(3);

		///////////////////////// PUBLISH ////////////////////////////
		
		// Output modified video stream
		std_msgs::Int32 detected;
		detected.data = static_cast<int>(largest_area);

		ROS_INFO("%d", detected.data);
    detect_pub_.publish(detected);
		
		///////////////////////// END PUBLISH ////////////////////////////

  }
};


Mat blur(cv_bridge::CvImagePtr frame)
{
	Mat blurry;
	GaussianBlur(frame->image, blurry, Size(9,9),0,0);
	return blurry;
}


Mat convert_frame(cv_bridge::CvImagePtr frame) 
{
	Mat frame_converted, blurry, red1, red2, red3;

	cvtColor(frame->image, frame_converted, CV_BGR2HSV);
  inRange(frame_converted, Scalar(0,0,50), Scalar(20,255,255), red1);
  inRange(frame_converted, Scalar(160,0,60), Scalar(180,255,255), red2);
  // Combining red masks
  addWeighted(red1, 1.0, red2, 1.0, 0.0, red3);
  GaussianBlur(red3, blurry, Size(9,9),0,0);
  //blur(cameraFrameGrey, blury, Size(9,9),Point(-1,-1));
  Canny(blurry, frame_converted, 10, 50, 3);

	return frame_converted;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();




  return 0;
}

