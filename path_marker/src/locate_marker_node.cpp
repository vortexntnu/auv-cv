#include <path_marker/SlidingWindowMemory.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <math.h>
#include <sstream>
#include <string>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm> 
#include "std_msgs/Float32.h"


using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{

private:
  	ros::NodeHandle nh_;
  	image_transport::ImageTransport it_;
  	image_transport::Subscriber image_sub_;
	ros::Publisher detect_pub_;

	Canny_SlidingWindowMemory canny_memory;

	char* src;

	int canny_low;
	int canny_high;
	int kernel_value;
	int threshold_value;
	int min_line_length;
	int max_line_gap;

	Rect2d bbox;
	vector<vector<Point>> contours;

public:
	ImageConverter(int argc, char** argv)
		: it_(nh_)
	{
		if((argv[1] == NULL))
		{
			std::cout << "Please provide a ROS topic to subscribe. The following are recomended:" << std::endl;
			std::cout << "- /camera/front" << std::endl;
			std::cout << "- /camera/under" << std::endl;
			std::cout << "Subscribing to default topic of simulator: /manta/manta/cameraunder/camera_image" << std::endl;
    	}
		else
		{
			src = argv[1];
		}


		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe("image", 1, &ImageConverter::imageCb, this);
		detect_pub_ = nh_.advertise<std_msgs::Float32>("path_angle", 1000);


		cv::namedWindow(OPENCV_WINDOW, WINDOW_NORMAL);


		canny_low = 23;
		canny_high = 55;
		kernel_value = 5;
		threshold_value = 120;
		min_line_length = 10;
		max_line_gap = 10;

	}

	~ImageConverter()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}
	

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
  	{
		cv_bridge::CvImagePtr cv_ptr;
		Mat frame;
		Mat canny;
		vector<Vec4i> lines;

		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
			catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		/***************
			   CV
		****************/

		frame = blur(cv_ptr);

		if (!strcmp(src, "/camera/front") || !strcmp(src, "/camera/under")) // Regular camera
		{
			// HUE: low, high; SAT: low, high; VALUE: low, high
			frame = convert_color(frame, 0, 72, 0, 255, 0, 255); 
		}
		else // Simulator
		{
			// HUE: low, high; SAT: low, high; VALUE: low, high
			frame = convert_color(frame, 10, 30, 20, 80, 20, 80); 
		}

        canny = detect_edges(frame, 9);
		//frame = canny_memory.sliding_window(canny);
		frame = dilate_erode(frame);
		findContours(frame, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);


		double cnt_size = 0;
		vector<Point> cnt;

		double cnt_size_max = 0;
		vector<Point> cnt_max;

		for (int i = 0; i < contours.size(); i++) {
			cnt = contours[i];
			cnt_size = contourArea(cnt);
			if (cnt_size <= 50 || cnt_size < cnt_size_max)
            	continue;

			cnt_size_max = cnt_size;
			cnt_max = cnt;
		}


		if (contours.size() > 0)
		{
			bbox = boundingRect(cnt_max);
			center = extract_center(bbox);
			rectangle(frame, bbox.tl(), bbox.br(), Scalar(0,255,0),5);
		}

		// Update GUI Window
		cv::waitKey(3);
		cv::imshow(OPENCV_WINDOW, frame);

		/***************
			Publish
		****************/

		// Output modified video stream
		std_msgs::Float32 direction;

		// if (lines.size() > 0)
		// 	direction.data = get_direction();

		//ROS_INFO("%f", direction.data);
    	detect_pub_.publish(direction);
		
  	}

	Mat read(VideoCapture cap, Mat frame){

		bool bSuccess = cap.read(frame);

		if (!bSuccess)
		{
			cout << "Cannot read a frame from video stream" << endl;
		}
		return frame;
	}

	string get_string(int i){
		stringstream i_ss;
		i_ss << i;
		string i_string = i_ss.str();
		return i_string;
	}

	void saveFrame(Mat frame, Mat original, int i){
		imwrite("images/real/analyzed_"+get_string(i)+"_"
				+"kernel_"+get_string(kernel_value)+"_"
				+"canL_"+get_string(canny_low)+"_"
				+"canH_"+get_string(canny_high)+"_"
				+"thresh_"+get_string(threshold_value)+"_"
				+"minLine_"+get_string(min_line_length)+"_"
				+"maxGap_"+get_string(max_line_gap)+"_"
				+".jpg",frame);
		imwrite("images/real/original_"+get_string(i)+"_"
				+"kernel_"+get_string(kernel_value)+"_"
				+"canL_"+get_string(canny_low)+"_"
				+"canH_"+get_string(canny_high)+"_"
				+"thresh_"+get_string(threshold_value)+"_"
				+"minLine_"+get_string(min_line_length)+"_"
				+"maxGap_"+get_string(max_line_gap)+"_"
				+".jpg",original);
	}

	Mat blur(cv_bridge::CvImagePtr frame, int blur_size = 3){
		Mat frame_mat;
		GaussianBlur(frame->image, frame_mat, Size(blur_size, blur_size),0,0);
		
		return frame_mat;
	}

	Mat convert_color(Mat frame, int H_lb = 0, int H_ub = 179, int S_lb = 0, int S_ub = 255, int V_lb = 0, int V_ub = 255)
	{
		Mat frame_converted;

    	cvtColor(frame, frame_converted, COLOR_BGR2HSV);
	    inRange(frame_converted, Scalar(H_lb,S_lb,V_lb), Scalar(H_ub,S_ub,V_ub), frame_converted);
		
		return frame_converted;
	}


	Mat detect_edges(Mat frame, int kernel_size = 3){
		Mat edgy;
		Canny(frame,edgy,canny_low,canny_high,3);
		
		return edgy;
	}

	Mat dilate_erode(Mat frame){
		int an = kernel_value;
		int element_shape = MORPH_RECT;
		Mat element = getStructuringElement(element_shape, Size(an*2+1, an*2+1), Point(an, an) );
		dilate(frame, frame, element );
		erode(frame, frame, element );
		
		return frame;
	}

	std::tuple<double, double> extract_center(Rect2d bbox){
		cx = bbox.x + bbox.width/2; 
		cy = bbox.y + bbox.height/2; 
		return (cx,cy);
	}
	

};




int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_marker");
	ImageConverter ic(argc, argv);
	ros::spin();
	return 0;
}
