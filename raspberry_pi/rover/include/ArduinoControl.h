#pragma once
#include "../include/Csearch.h"
#include "../include/TransferValuesToArduino.h"
class ArduinoControl
{
  private:
	Csearch csearch_;
	TransferValuesToArduino transfer_;
  public:
	ArduinoControl(/* args */);
	~ArduinoControl();
	int Init();
	int Transfer(int angle, unsigned char order);
	void Csearch1();
	void Csearch2();
};