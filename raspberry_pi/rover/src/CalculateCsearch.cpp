#include <cmath>
#include "../include/CalculateCsearch.h"
double ConvertCoordinateToAngle(double coordinate[2]){
	double conX=((coordinate[0]-180)/180)/sqrt(3);
	return atan(conX);
}
