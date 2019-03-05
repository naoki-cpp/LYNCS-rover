#include <Arduino.h>
#include "./local_libs/RoverMotor.h"
#include "./PinDefinitions.h"

#define INLINE inline __attribute__((always_inline))

INLINE constexpr double Limit(double inf, double sup, double x)
{
	return max(inf, min(sup, x));
}

namespace lyncs
{

RoverMotor::RoverMotor()
{
}

RoverMotor::~RoverMotor()
{
}
void RoverMotor::Init()
{
	servo_right_.attach(ROVERMOTOR_RIGHT);
	servo_left_.attach(ROVERMOTOR_LEFT);
}
void RoverMotor::RoverOutput(double outR, double outL)
{
	servo_right_.writeMicroseconds(1500 - outR * 500);
	servo_left_.writeMicroseconds(1500 + outL * 500);
}
void RoverMotor::RoverPower(double outV, double outT)
{
	//上限下限
	outV = Limit(-1, 1, outV);
	outT = Limit(-1, 1, outT);

	double outR = (outV + outT)/2;
	double outL = (outV - outT)/2;

	RoverOutput(outR, outL);
}
} // namespace lyncs
