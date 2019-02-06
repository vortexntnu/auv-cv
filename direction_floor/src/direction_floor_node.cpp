#include<direction_floor/SlidingWindowMemory.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <math.h>
#include <sstream>
#include <string>
#include <fstream>

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{

private:
  	ros::NodeHandle nh_;
  	image_transport::ImageTransport it_;
  	image_transport::Subscriber image_sub_;
  	image_transport::Publisher image_pub_;

	SlidingWindowMemory angle_memory;
	SlidingWindowMemory u_prime_memory;
	SlidingWindowMemory v_prime_memory;
	SlidingWindowMemory u_secondary_memory;

	SlidingWindowMemory v_secondary_memory;
	int canny_low;
	int canny_high;
	int kernel_value;
	int threshold_value;
	int min_line_length;
	int max_line_gap;

public:
	ImageConverter()
		: it_(nh_)
	{
		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/camera/image", 1,
		&ImageConverter::imageCb, this);
		//image_pub_ = it_.advertise("/image_converter/output_video", 1);

		cv::namedWindow(OPENCV_WINDOW, WINDOW_NORMAL);

		canny_low = 23;
		canny_high = 55;
		kernel_value = 1;
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

		frame = blur(cv_ptr);
        Mat frame_converted;

    	cvtColor(frame, frame_converted, COLOR_BGR2HSV);
	    inRange(frame_converted, Scalar(0,0,0), Scalar(72,255,255), frame_converted);

        frame = detect_edges(frame_converted);
		frame = dilate_erode(frame);

		lines = find_lines(frame);
		//lines = cam_object.remove_border_lines(lines, cap);
		//lines = cam_object.sort_lines(lines);

		frame = drawLines(frame, lines);
		frame = drawOneLine(frame, lines);

		// Update GUI Window
		//cv::imshow(OPENCV_WINDOW, frame);
		//

		cv::waitKey(3);
		cv::imshow(OPENCV_WINDOW, frame);
		// Output modified video stream
		//image_pub_.publish(cv_ptr->toImageMsg());
  	}

	Mat read(VideoCapture cap, Mat frame){

		bool bSuccess = cap.read(frame);

		if (!bSuccess)
		{
			cout << "Cannot read a frame from video stream" << endl;
		}
		return frame;
	}

	void showFrame(Mat frame){
		imshow("frame", frame);
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

	Mat blur(cv_bridge::CvImagePtr frame){
		Mat frame_mat;
		GaussianBlur(frame->image, frame_mat, Size(3,3),0,0);
		return frame_mat;
	}

	Mat detect_edges(Mat frame){
		Mat edgy;
		Canny(frame,edgy,canny_low,canny_high,3);
		return edgy;
	}

	Mat dilate_erode(Mat frame){
		int an = kernel_value;
		int element_shape = MORPH_RECT;
		Mat element = getStructuringElement(element_shape, Size(an*2+1, an*2+1), Point(an, an) );
		dilate(frame, frame, element );
		//erode(frame, frame, element );
		return frame;
	}

	vector<Vec4i> find_lines(Mat frame){
		vector<Vec4i> lines;
		HoughLinesP(frame, lines, 1, CV_PI/180, threshold_value, min_line_length, max_line_gap );
		return lines;
	}

	vector<Vec4i> remove_border_lines(vector<Vec4i>& lines, VideoCapture cap){
		vector<Vec4i> lines_passed;
		double width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
		double height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
		for( size_t i = 0; i < lines.size(); i++ )
		{
			Vec4i l = lines[i];
			if(l[0] < width - 120 && l[0] > 120){
				if(l[1] < height - 120 && l[1] > 120){
					lines_passed.push_back(l);
				}
			}

		}
		return lines_passed;
	}

	vector<Vec4i> sort_lines(vector<Vec4i>& lines){
		vector<Vec4i> lines_passed;
		double avg_angle = 0;

		for( size_t i = 0; i < lines.size(); i++ )
		{
			Vec4i l = lines[i];
			double delta_u = l[2]-l[0];
			double delta_v = l[3]-l[1];

			avg_angle += atan2(delta_v, delta_u)*180/3.14159265;

			lines_passed.push_back(l);
		}
		if(lines.size() > 0){
			avg_angle = avg_angle / lines.size();

		}
		return lines_passed;
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

	Mat drawOneLine(Mat frame, vector<Vec4i>& lines){
		double avg_angle = 0;

		int height = frame.rows;
		int width = frame.cols;

		vector<double> lines_histogram;
		vector< vector<double> > lines_location_histogram;

		vector<double> location_info_template;
		location_info_template.push_back(0.0);
		location_info_template.push_back(0.0);
		location_info_template.push_back(0.0);

		int circle_sections = 180/1;

		for( int i = 0; i < circle_sections; i ++){
			lines_histogram.push_back(0);
			lines_location_histogram.push_back(location_info_template);
		}



		for( size_t i = 0; i < lines.size(); i++ ){
			Vec4i l = lines[i];
			double delta_u = l[2]-l[0];
			double delta_v = l[3]-l[1];
			double l_length = sqrt(delta_u*delta_u+delta_v*delta_v);
			double angle;
			int angle_int;

			if(atan2(delta_v, delta_u) < 0){
				angle = atan2(delta_v, delta_u)+3.14159265;
			} else {
				angle = atan2(delta_v, delta_u);
			}
			angle_int = (angle*circle_sections/3.14159265);
			lines_histogram[angle_int]+=l_length;

			vector<double> location_info_prev = lines_location_histogram.at(angle_int);
			double length_prev = location_info_prev[2];
			double length_total = length_prev+l_length;
			double length_weighting = length_prev/length_total;

			double avg_u_prev = location_info_prev[0];
			double avg_u_new = delta_u/2 + l[0];
			double avg_u = length_weighting*avg_u_prev + (1-length_weighting)*avg_u_new;

			double avg_v_prev = location_info_prev[1];
			double avg_v_new = delta_v/2 + l[1];
			double avg_v = length_weighting*avg_v_prev + (1-length_weighting)*avg_v_new;

			vector<double> location_info;
			location_info.push_back(avg_u);
			location_info.push_back(avg_v);
			location_info.push_back(length_total);

			lines_location_histogram.at(angle_int) = location_info;
		}

		double max_value = 0;
		double max_int = 0;
		double second_highest_int = 0;
		double second_highest_value = 0;
		double u_temp;
		double v_temp;
		double u_prime = 0;
		double u_secondary = 0;
		double v_prime = 0;
		double v_secondary = 0;

		for( unsigned int i = 0; i < lines_histogram.size(); i ++){
			if(lines_histogram[i]>max_value && abs(max_int-i)>2){
				second_highest_int = max_int;
				second_highest_value = max_value;
				max_value = lines_histogram[i];
				max_int = i;

				u_secondary = u_prime;
				v_secondary = v_prime;
				if(i>0 && i<180){
					double weight_previous = lines_location_histogram.at(i-1).at(2);
					double weight_current = lines_location_histogram.at(i).at(2);
					double weight_next = lines_location_histogram.at(i+1).at(2);
					double weight_sum = weight_previous + weight_current + weight_next;

					double u_previous = lines_location_histogram.at(i-1).at(0);
					double u_current = lines_location_histogram.at(i).at(0);
					double u_next = lines_location_histogram.at(i+1).at(0);

					double average_u = u_previous*(weight_previous)/weight_sum+u_current*(weight_current)/weight_sum+u_next*(weight_next)/weight_sum;


					double v_previous = lines_location_histogram.at(i-1).at(1);
					double v_current = lines_location_histogram.at(i).at(1);
					double v_next = lines_location_histogram.at(i+1).at(1);

					double average_v = v_previous*(weight_previous)/weight_sum+v_current*(weight_current)/weight_sum+v_next*(weight_next)/weight_sum;

					u_temp = 0.5+average_u;

					v_temp = 0.5+average_v;
				} else {
					u_temp = 0.5+lines_location_histogram.at(i).at(0);
					v_temp = 0.5+lines_location_histogram.at(i).at(1);
				}

				u_prime = u_temp;
				v_prime = v_temp;
			}
		}


		if(!(u_secondary==0)){
			u_secondary = u_secondary_memory.sliding_window(u_secondary);
			v_secondary = v_secondary_memory.sliding_window(v_secondary);
		}

		u_prime = u_prime_memory.sliding_window(u_prime);
		v_prime = v_prime_memory.sliding_window(v_prime);


		int u_prime_int = (int)u_prime;
		int v_prime_int = (int)v_prime;
		int u_secondary_int = (int)u_secondary;
		int v_secondary_int = (int)v_secondary;


		double max_angle = (max_int / circle_sections) * 3.14159265;
		double second_angle = (second_highest_int / circle_sections) * 3.14159265;
		double length = 200;
		/*line( frame,
				Point(width/2-length*cos(max_angle), height/2-length*sin(max_angle)),
				Point(width/2+length*cos(max_angle), height/2+length*sin(max_angle)),
				Scalar(0,255,0), 3, CV_AA);*/
		double weighting_factor = max_value/(max_value+second_highest_value);
		avg_angle = max_angle;

		avg_angle = angle_memory.sliding_window( avg_angle );
		line( frame,
					Point(width/2-length*cos(avg_angle), height/2-length*sin(avg_angle)),
					Point(width/2+length*cos(avg_angle), height/2+length*sin(avg_angle)),
					Scalar(255,255,0), 3, CV_AA);

		/*
        circle( frame, Point(u_prime_int,v_prime_int),50, Scalar(255,0,0),CV_FILLED, 8,0);
		if(u_secondary != 0){
			circle( frame, Point(u_secondary_int,v_secondary_int),50, Scalar(0,255,0),CV_FILLED, 8,0);
		}*/
		return frame;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pipe_detection");
	ImageConverter ic;
	ros::spin();
	return 0;
}