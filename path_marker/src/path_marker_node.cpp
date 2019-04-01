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

bool compareByLength(Vec4i &a, Vec4i &b);
double distance_between_two_points(Vec4i& line);

#define PI 3.14159265


class ImageConverter
{

private:
  	ros::NodeHandle nh_;
  	image_transport::ImageTransport it_;
  	image_transport::Subscriber image_sub_;
	ros::Publisher detect_pub_;


	Angle_SlidingWindowMemory angle_memory;
	Canny_SlidingWindowMemory canny_memory;

	char* src;

	int canny_low;
	int canny_high;
	int kernel_value;
	int threshold_value;
	int min_line_length;
	int max_line_gap;

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
		frame = canny_memory.sliding_window(canny);
		frame = dilate_erode(frame);

		lines = find_lines(frame);
		lines = filter_lines(lines);
		frame = drawLines(frame, lines);

		if (lines.size() > 0)
		{
			frame = plot_direction(frame, lines);
		}

		// Update GUI Window
		cv::waitKey(3);
		cv::imshow(OPENCV_WINDOW, frame);

		/***************
			Publish
		****************/

		// Output modified video stream
		std_msgs::Float32 direction;

		if (lines.size() > 0)
			direction.data = get_direction();

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

	vector<Vec4i> find_lines(Mat frame){
		vector<Vec4i> lines;
		HoughLinesP(frame, lines, 3, CV_PI/180, threshold_value, min_line_length, max_line_gap );
		return lines;
	}


	Mat drawLines(Mat frame, vector<Vec4i>& lines){
		cvtColor(frame, frame, CV_GRAY2BGR);
		for( size_t i = 0; i < lines.size(); i++ )
		{
			Vec4i l = lines[i];
			line( frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
		}
		return frame;
	}


	vector<Vec4i> filter_lines(vector<Vec4i> lines)
	{
		std::sort(lines.begin(), lines.end(), compareByLength);
		int endOf = min(static_cast<int>(lines.size()), 4);
		vector<Vec4i>::const_iterator first = lines.begin();
		vector<Vec4i>::const_iterator last = lines.begin() + endOf;
		vector<Vec4i> lines_filtered(first, last);
		
		return lines_filtered;
	}


	Vec4i get_minLine(vector<Vec4i> lines)
	{
		int i_min = 0;
		int y_pos = 0;
		int y_min = -1;

		// Find position of element with highest position of y coordinate
		for (int i = 0; i < lines.size(); i++) 
		{
			//cout << lines[i] << endl;

			y_pos = min(lines[i][3], lines[i][1]);
			if (y_pos < y_min)
			{
				y_min = y_pos;
				i_min = i;
			}
		}
		//cout << endl;


		return lines[i_min];
	}

	
	Mat plot_direction(Mat frame, vector<Vec4i> lines)
	{
		int height = frame.rows;
		int width = frame.cols;
		double length = 200;
		double angle;

		Vec4i l = get_minLine(lines);

		double dx = l[2]-l[0];
		double dy = l[3]-l[1];

		if(atan2(dy, dx) < 0)
		{
			angle = atan2(dy, dx) + PI;
		} else 
		{
			angle = atan2(dy, dx);
		}
		// cout << "Angle: " << angle << endl;
		angle = angle_memory.sliding_window(angle);
		// cout << "Angle memory: " << angle << endl;
		// cout << endl;
		line( frame,
					Point(width/2-length*cos(angle), height/2-length*sin(angle)),
					Point(width/2+length*cos(angle), height/2+length*sin(angle)),
					Scalar(255,255,0), 3, CV_AA);
		return frame;
	}	


	float get_direction()
	{
		float angle = angle_memory.get_value();
		return angle - (3.14159265/2);
	}

};


bool compareByLength(Vec4i &a, Vec4i &b)
{
	return distance_between_two_points(a) > distance_between_two_points(b);
}

double distance_between_two_points(Vec4i& line)
{
	double dx = line[2] - line[0];
	double dy = line[3] - line[1];

	return hypot(dy, dx);
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_marker");
	ImageConverter ic(argc, argv);
	ros::spin();
	return 0;
}
