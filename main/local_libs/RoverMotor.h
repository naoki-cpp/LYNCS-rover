#pragma once
#include <Servo.h>
namespace lyncs
{

class RoverMotor
{
  private:
	Servo servo_right_;
	Servo servo_left_;
	void RoverOutput(double outR,double outL);
  public:
	RoverMotor();
	~RoverMotor();
	void Init();
	void RoverPower(double outV,double outT);
};
} // namespace lyncs