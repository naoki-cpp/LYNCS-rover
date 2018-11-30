#pragma once
#include <Arduino.h>
namespace lyncs
{

class RoverMotor
{
  private:
	const unsigned int kOutR1_;
	const unsigned int kOutR2_;
	const unsigned int kOutL1_;
	const unsigned int kOutL2_;
	void RoverOutput(uint8_t outR,uint8_t outL);
  public:
	RoverMotor();
	~RoverMotor();
	void Init();
	void RoverPower(double outV,double outT);
};
} // namespace lyncs