#include "../include/ForTest.h"
#include "../include/Csearch.h"
#include <iostream>
using namespace std;

int Test(){
	double coordinate[2];
	if(Csearch(10,0,180,140, coordinate) == 2){
		cout << "x:  " <<coordinate[0] << endl;
		cout << "y:  " <<coordinate[1] << endl;
	}else{
		cout << "error" << endl;
	}
}
