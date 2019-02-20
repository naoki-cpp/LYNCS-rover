#include "../include/Csearch.h"
#include "../include/CalculateCsearch.h"
#include "../include/TransferValuesToArduino.h"
#include "../include/ArduinoControl.h"
#include <iostream>
using namespace std;

int count = 0;

ArduinoControl::ArduinoControl(/* args */)
{
}

ArduinoControl::~ArduinoControl()
{
}

int ArduinoControl::Init(){
	return csearch_.Init();
}

void ArduinoControl::Csearch1()
{
	int judgei;
	char k = 0;
	double xy[2];

	while (k < 4)
	{
		judgei = csearch_.Search(110, 101, 100, 90, xy);
		if (judgei == 2 or judgei == 3)
		{
			TransferValuesToArduino(0, 1);
			cout << count++ <<endl;
			break;
		}
		if (judgei == 2)
		{
			break;
		}

		k++;
	}
}

void ArduinoControl::Csearch2()
{
	int judgei;
	char k = 0;
	double answer;
	double xy[2];

	while (k < 4)
	{
		judgei = csearch_.Search(10, 0, 180, 140, xy);
		if (judgei == 2)
		{
			answer = ConvertCoordinateToAngle(xy) * 1000;
			TransferValuesToArduino((int)answer, 4);
			cout << answer << endl;
			break;
		}
		if (judgei == 0)
		{
			TransferValuesToArduino(0, 2);
			break;
		}
		if (judgei == 3)
		{
			TransferValuesToArduino(0, 3);
			break;
		}

		k++;
	}
}
