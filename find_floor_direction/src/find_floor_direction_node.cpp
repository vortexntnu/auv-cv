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
#include "std_msgs/Int8.h"
//#include "std_msgs/String.h"
#include <sstream>

using namespace cv;
using namespace std;


Mat convert_frame(cv_bridge::CvImagePtr frame);
vector<Vec4i> filter_lines(vector<Vec4i> lines);
Vec4i get_direction(vector<Vec4i> lines);
double distance_between_two_points(int x1, int x2, int y1, int y2);
double distance_between_two_points(Vec4i& line);
double pythagoras(int height, int width);
int line_to_deg(Vec4i& line);



vector<Vec4i> lines_filtered;
vector<Vec4i> lines_sorted;
Vec4i line_dir;

//Video
Mat frame; 
Mat frame_converted;

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
    detect_pub_ = nh_.advertise<std_msgs::Int8>("direction_line", 1000);

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

		// Probabilistic Line Transform
	  vector<Vec4i> lines; // will hold the results of the detection
	  HoughLinesP(frame_converted, lines, 1, CV_PI/180, 50, 50, 10 ); // runs the actual detection
		
		//Filter out lines that doesn't mach desired output
		
		lines_filtered = filter_lines(lines);


		//check if empty
		if (lines_filtered.size() > 0)
		{
			line_dir = get_direction(lines_filtered);
		}
	    // Draw the lines
	  for( size_t i = 0; i < lines_filtered.size(); i++ )
	  {	
			//cout << lines_filtered[i] << endl;
			Vec4i l = lines_filtered[i];
			line( frame_converted, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2, LINE_AA);
	  }

		///////////////////////// END CV ////////////////////////////

		
    // Update GUI Window
		imshow(OPENCV_WINDOW, frame_converted);
    waitKey(3);

		///////////////////////// PUBLISH ////////////////////////////

    // Output modified video stream
		std_msgs::Int8 direction;
		direction.data = 0;

		if (lines_filtered.size() != 0)
			direction.data = line_to_deg(line_dir);


		ROS_INFO("%d", direction.data);
    detect_pub_.publish(direction);

		///////////////////////// END PUBLISH ////////////////////////////

  }
};


double distance_between_two_points(int x1, int x2, int y1, int y2)
{
	int distancex = abs(x2 - x1)^2;
  int distancey = abs(y2 - y1)^2;

	return sqrt(distancex - distancey);
}


double distance_between_two_points(Vec4i& line)
{
	int distancex = abs(line[2] - line[0]);
  int distancey = abs(line[3] - line[1]);

  return pythagoras(distancey, distancex);
}


double pythagoras(int height, int width)
{
  return sqrt((height^2) + (width^2));
}


Mat convert_frame(cv_bridge::CvImagePtr frame) 
{
	Mat canny_output, kernel, frame_converted;

	cvtColor(frame->image, frame_converted, COLOR_BGR2HSV);
	inRange(frame_converted, Scalar(0,0,0), Scalar(72,255,255), frame_converted);
  GaussianBlur( frame_converted, frame_converted, Size(9,9), 0, 0);
	Canny( frame_converted, canny_output, 10, 50, 3 );
	kernel = Mat::ones(3, 3, CV_32F); //evt CV_8UC3
	dilate(canny_output, frame_converted, kernel);

	return frame_converted;
}


vector<Vec4i> filter_lines(vector<Vec4i> lines)
{
	vector<Vec4i> lines_filtered;
	int height, height_new, width, width_new, length, length_smallest, x_diff;
	vector<Vec4i>::iterator it_max;
			
	//iterate through every detected line
	for( size_t i = 0; i < lines.size(); i++ )
	{		
		Vec4i l = lines[i]; 

		//Notice: l[3] (y_1) may be smaller than l[1] (y_2)
		//l[2] > l[0]
		length = distance_between_two_points(l);

		//make sure there are at least 1 object in the list to later compare
		if (lines_filtered.size() == 0)
		{
			lines_filtered.insert(lines_filtered.begin(), l);
			length_smallest = length;
		}

		vector<Vec4i>::reverse_iterator it = lines_filtered.rbegin();
		for (it; it != lines_filtered.rend(); ++it ) 
		{
			length_smallest = length;
			length = distance_between_two_points(*it); 
		} 
		/*
		vector<Vec4i>::iterator it = lines_filtered.end();
		while (it != lines_filtered.begin() || length > length_smallest)
		{
			--it;
			cout << "compare" << endl;
			length_smallest = length;
			length = distance_between_two_points(*it);
		}
		*/
		if (length > length_smallest)
			it_max = lines_filtered.insert(it.base(), *it);

		if (lines_filtered.size() > 4)
			lines_filtered.pop_back();	
		
	}

	return lines_filtered;
}


Vec4i get_direction(vector<Vec4i> lines)
{
	int i_max = 0;
	int y_pos = 0;
	int y_max = -1;
	
	// Find position of element with highest position of y coordinate
	for (int i = 0; i < lines.size(); i++) 
	{
		y_pos = min(lines[i][3], lines[i][1]);
		if (y_pos > y_max)
		{
			y_max = y_pos;
			i_max = i;
		}
	}

	return lines[i_max];
}	

int line_to_deg(Vec4i& line)
{
	return atan2(abs(line[1]-line[3]), abs(line[0]-line[2]));
}

////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}


