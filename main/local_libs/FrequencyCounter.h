#pragma once
#include "./MedianFilter.h"
namespace lyncs
{

class FrequencyCounter
{
  private:
  unsigned long last_time_;
  unsigned long period_time_;
  unsigned long previous_last_time_;
  int reset_counter_;
  lyncs::MedianFilter<unsigned long, 10> period_filter_;
  public:
	FrequencyCounter(/* args */);
	~FrequencyCounter();
	void Init();
	void Interrupt();// interrupt
	void Update();// in loop()
	double GetFreq();
};

} // namespace lyncs