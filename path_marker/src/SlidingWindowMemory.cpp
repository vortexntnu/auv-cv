#include <path_marker/SlidingWindowMemory.h>
#include <iostream>
#include <vector>

using namespace std;

SlidingWindowMemory::SlidingWindowMemory(){
	sliding_window_memory_size = 15;
	for(int i = 0; i<sliding_window_memory_size;i++)
	{
		sliding_window_memory.push_back(-1);
	}
}

double SlidingWindowMemory::sliding_window(double new_value){
	for(int i = 1; i < sliding_window_memory.size(); i++){
		sliding_window_memory.at(i-1) = sliding_window_memory.at(i);
	}
	sliding_window_memory.at(sliding_window_memory.size()-1) = new_value;
	double value_sum = 0;
	double value_instances = 0;
	for(int i = 0; i < sliding_window_memory.size(); i++){
		if(sliding_window_memory.at(i)>-1){
			value_sum += sliding_window_memory[i];
			value_instances += 1;
		}
	}

	sliding_avg_value = value_sum / value_instances;

	return sliding_avg_value;
}

double SlidingWindowMemory::get_value(){
	return sliding_avg_value;
}
