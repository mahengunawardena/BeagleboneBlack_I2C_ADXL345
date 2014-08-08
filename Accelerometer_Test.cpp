/*
 * Accelerometer_Test.cpp
 *
 *  Created on: Aug 5, 2014
 *      Author: mahen
 */

#include "ADXL345Accelerometer.h"
#include <iostream>
#include <string>

using namespace std;


int main(int argc, char *argv[])
{
	cout << "Testing ADXL345Accelerometer " << endl;

	ADXL345Accelerometer Accelerometer_Test(1, 0x53);
	cout<<"Device ID : "<< hex << Accelerometer_Test.getAccelerometer_ID()<<endl;
	Accelerometer_Test.SetPowerMode(0x01);
	Accelerometer_Test.getAccelerationData();

	while (1) {
		cout << "X Data :" << dec <<Accelerometer_Test.getAcceleration_X() << " Y Data :" << dec <<Accelerometer_Test.getAcceleration_Y() << " Z Data :" << dec <<Accelerometer_Test.getAcceleration_Z()<<"\t";
		cout << "Pitch : " << dec << Accelerometer_Test.getPitch() << "\tRoll : " << dec << Accelerometer_Test.getRoll()<<endl;
		usleep(1000000);
		Accelerometer_Test.getAccelerationData();
	}
	cout << "Testing Accelerometer Complete" << endl;

	return 0;
}



