#include <path_marker/SlidingWindowMemory.h>
#include <iostream>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace std;


Angle_SlidingWindowMemory::Angle_SlidingWindowMemory(){
	memory_size = 15;
	for(int i = 0; i < memory_size; i++)
	{
		angle_memory.push_back(-1);
	}
}



double Angle_SlidingWindowMemory::sliding_window(double new_value){
	for(int i = 1; i < angle_memory.size(); i++){
		angle_memory.at(i-1) = angle_memory.at(i);
	}
	angle_memory.at(angle_memory.size()-1) = new_value;
	double value_sum = 0;
	double value_instances = 0;
	for(int i = 0; i < angle_memory.size(); i++){
		if(angle_memory.at(i)>-1){
			value_sum += angle_memory[i];
			value_instances += 1;
		}
	}
	
	avg_angle = value_sum / value_instances;

	return avg_angle;
}

double Angle_SlidingWindowMemory::get_value(){
	return avg_angle;
}



/****************************************
* CANNY 								*
****************************************/

Canny_SlidingWindowMemory::Canny_SlidingWindowMemory()  
{
	memory_size = 3;
	cv::Mat empty_canny;
	for(int i = 0; i < memory_size; i++)
	{
		canny_memory.push_back(empty_canny);
	}
}


template<typename T>
void pop_front(std::vector<T>& vec)
{
    vec.erase(vec.begin());
}


cv::Mat Canny_SlidingWindowMemory::sliding_window(cv::Mat frame){
	canny_memory.push_back(frame);
	if (canny_memory.size() > memory_size) {
		pop_front(canny_memory);
	}

	canny_sum = frame + canny_memory[0] + canny_memory[1];

	return canny_sum;
}
