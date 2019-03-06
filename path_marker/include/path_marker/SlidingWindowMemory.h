#ifndef SLIDINGWINDOWMEMORY_H
#define SLIDINGWINDOWMEMORY_H

#include <vector>

class SlidingWindowMemory{
	public:
		SlidingWindowMemory();
		double sliding_window(double new_value);
		double get_value();
	private:
		std::vector<double> sliding_window_memory;
		double sliding_window_memory_size;
		double sliding_avg_value;
};

#endif // SLIDINGWINDOWMEMORY_H