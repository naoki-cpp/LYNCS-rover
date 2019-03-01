#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include "../include/Csearch.h"
#include "../include/TransferValuesToArduino.h"

class ArduinoControl
{
  private:
	std::ofstream log_file_;
	Csearch csearch_;
	TransferValuesToArduino transfer_;
  public:
	ArduinoControl(/* args */);
	~ArduinoControl();
	int Init();
	int Transfer(int angle, unsigned char order);
	int Csearch1();
	int Csearch2();
	void LogOutput(std::string str);
};
