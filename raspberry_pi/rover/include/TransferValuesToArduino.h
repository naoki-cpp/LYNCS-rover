#pragma once
#include <cstdint>
class TransferValuesToArduino
{
private:
	int fd_;
	uint8_t mode;
	uint8_t bits;
	uint32_t speed;
	uint16_t delay;
public:
	TransferValuesToArduino(/* args */);
	~TransferValuesToArduino();
	int Init();
	int Transfer(int angle, unsigned char order);
};
