#ifndef SLIDINGWINDOWMEMORY_H
#define SLIDINGWINDOWMEMORY_H

#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class Angle_SlidingWindowMemory{
	public:
		Angle_SlidingWindowMemory();
		double sliding_window(double new_value);
		double get_value();
	private:
		std::vector<double> angle_memory;
		double memory_size;
		double avg_angle;
};


class Canny_SlidingWindowMemory{
	public:
		Canny_SlidingWindowMemory();
		cv::Mat sliding_window(cv::Mat frame);
	private:
		std::vector<cv::Mat> canny_memory;
		int memory_size;
		cv::Mat canny_sum;
};

#endif // SLIDINGWINDOWMEMORY_H