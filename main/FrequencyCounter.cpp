#include "./local_libs/FrequencyCounter.h"
#include <limits.h>
#include <Arduino.h>
namespace lyncs
{
FrequencyCounter::FrequencyCounter(/* args */)
:period_filter_(lyncs::MedianFilter<unsigned long, 10>()),
last_time_(0),
previous_last_time_(0),
period_time_(UINT_MAX)
{
}

FrequencyCounter::~FrequencyCounter()
{
}

void FrequencyCounter::Init()
{
}

void FrequencyCounter::Interrupt(){
	unsigned long temp_time = micros();
	period_filter_.InputData(temp_time - last_time_);
	last_time_ = temp_time;

	period_time_ = period_filter_.GetData();
}

void FrequencyCounter::Update()
{
	if (last_time_ == previous_last_time_)
	{
		++reset_counter_;
	}
	else
	{
		reset_counter_ = 0;
	}
	previous_last_time_ = last_time_;

	if (reset_counter_ > 10)
	{
		period_time_ = UINT_MAX;
	}
}

double FrequencyCounter::GetFreq(){
	
	if (reset_counter_ > 10) {
		return 0;
	}else{
		return double(1000000) / double(period_time_);
	}
}

} // namespace lyncs