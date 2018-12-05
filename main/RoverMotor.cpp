#include <Arduino.h>
#include "./local_libs/RoverMotor.h"
#include "./PinDefinitions.h"

#define INLINE inline __attribute__((always_inline))

INLINE double Limit(double inf, double sup, double x)
{
	return max(inf, min(sup, x));
}

namespace lyncs
{

RoverMotor::RoverMotor()
	: kOutR1_(ROVERMOTOR_RIGHT_1),
	  kOutR2_(ROVERMOTOR_RIGHT_2),
	  kOutL1_(ROVERMOTOR_LEFT_1),
	  kOutL2_(ROVERMOTOR_LEFT_2)
{
}

RoverMotor::~RoverMotor()
{
}
void RoverMotor::Init()
{
	pinMode(kOutR1_, OUTPUT);
	pinMode(kOutR2_, OUTPUT);
	pinMode(kOutL1_, OUTPUT);
	pinMode(kOutL2_, OUTPUT);
}
void RoverMotor::RoverOutput(uint8_t outR, uint8_t outL)
{
	digitalWrite(kOutR1_, LOW);
	analogWrite(kOutR2_, outR);
	digitalWrite(kOutL1_, LOW);
	analogWrite(kOutL2_, outL);
}
void RoverMotor::RoverPower(double outV, double outT)
{
	//上限下限
	outV = Limit(-0.5, 0.5, outV);
	outT = Limit(-0.5, 0.5, outT);
	uint8_t outR;
	uint8_t outL;
	if (outT >= 0)
	{
		outR = (outT + outV) * 255.0;
		outL = outV * 255.0;
	}
	if (outT < 0)
	{
		outR = outV * 255.0;
		outL = (outV - outT) * 255.0;
	}
	RoverOutput(outR, outL);
}
} // namespace lyncs
