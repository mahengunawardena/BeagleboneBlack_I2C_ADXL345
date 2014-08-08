//============================================================================
// Name        : ADXL345Accelerometer.cpp
// Author      : Mahendra Gunawardena
// Version     : Rev 0.01
// Copyright   : Your copyright notice
// Description : ADXL345Accelerometer in C++, Ansi-style
//============================================================================
/*
 * ADXL345Accelerometer.cpp
 * Implementation of a class to interface with the Analog Devices ADXL345 3 Axis Accelerometer
 * over the I2C bus
 *
 * Copyright Mahendra Gunawardena, Mitisa LLC
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL I
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stropts.h>
#include <stdio.h>
#include "ADXL345Accelerometer.h"
#include <iostream>
#include <math.h>
using namespace std;


ADXL345Accelerometer::ADXL345Accelerometer(int busID, int deviceAddress) {
	i2cBus = busID;
	i2cAddress = deviceAddress;
	accelerationX = accelerationY = accelerationZ=0;
	roll = pitch = 0;
	rawData_X = rawData_Y = rawData_Z = 0;
	deviceTemperature = tenBitAddress = opResult = 0;

	initAccelerometer();
}

int ADXL345Accelerometer::initAccelerometer()
{
	char namebuf[MAX_BUS];
	snprintf(namebuf, sizeof(namebuf), "/dev/i2c-%d", i2cBus);
	//Create a file descriptor for the I2C bus
	if ((i2cHandle = open(namebuf, O_RDWR))<0) {
		 cout << "Failed to open ADXL345 Sensor on " << namebuf << " I2C Bus" << endl;
		 return (1);
	}
	opResult = ioctl(i2cHandle, I2C_TENBIT, tenBitAddress);
	// Tell the I2C peripheral device address of the accelerometer
	if((opResult = ioctl(i2cHandle, I2C_SLAVE, i2cAddress))<0) {
		cout << "I2C_SLAVE address " << i2cAddress << " failed..." << endl;
		return(2);
	}
	return (0);
}
unsigned int ADXL345Accelerometer::getAccelerometer_ID(){
	unsigned char rxBuffer[1];		// receive buffer
	unsigned char txBuffer[1];		// transmit buffer

	txBuffer[0] = 0x00; // This is the address we want to read from.
	if ((opResult = write(i2cHandle, txBuffer, 1))!=1){
		cout <<"No ACK bit!" << endl;
		return (0x01);
	}
	opResult = read(i2cHandle, rxBuffer, 1);
	//cout <<"Part ID: "  << hex << (int) rxBuffer[0]<< endl;
	return ((unsigned int)rxBuffer[0]);
}

void ADXL345Accelerometer::SetPowerMode(unsigned char powerMode)
{
	unsigned char rxBuffer[1];		// receive buffer
	unsigned char txBuffer[2];		// transmit buffer

	unsigned char PowerCtl[1];

	//snprintf(txBuffer, sizeof(txBuffer), "%x", );
	txBuffer[0] = 0x2d ;// (unsigned char) ADXL345_POWER_CTL; // This is the address we want to read from.
	if ((opResult = write(i2cHandle, txBuffer, 1))!=1){
		cout <<"No ACK bit!" << endl;
	}
	opResult = read(i2cHandle,rxBuffer, 1);
	//cout <<"Part ID: "  << hex << (int) rxBuffer[0]<< endl;

	PowerCtl[0] = rxBuffer[0] & ~ADXL345_PWRCTL_MEASURE;
	txBuffer[1] = PowerCtl[0] | (powerMode * ADXL345_PWRCTL_MEASURE);
	if ((opResult = write(i2cHandle, txBuffer , 2))!=2){
		cout <<"No ACK bit!" << endl;
	}
}


int ADXL345Accelerometer::getAccelerationData()
{
	unsigned char rxBuffer[1];		// receive buffer
	unsigned char txBuffer[1];		// transmit buffer
	unsigned char dataBuffer[6];

	txBuffer[0] = 0x32 ;// (unsigned char) ADXL345_POWER_CTL; // This is the address we want to read from.
	for (int i=0;i<=5;i++) {
		if ((opResult = write(i2cHandle, txBuffer, 1))!=1){
			cout <<"No ACK bit Write!" << endl;
		}
		if((opResult = read(i2cHandle,rxBuffer, 1))!=1){
			cout <<"No ACK bit Read!" << endl;
		}
		dataBuffer[i]=rxBuffer[0];
		//cout<<"Data :"<<hex<<(int)dataBuffer[i]<<endl;
		txBuffer[0] = txBuffer[0] +1;
	}
	rawData_X = (dataBuffer[1]<<8)|dataBuffer[0];
	rawData_Y = (dataBuffer[3]<<8)|dataBuffer[2];
	rawData_Z = (dataBuffer[5]<<8)|dataBuffer[4];
	//cout<<"Raw X: " << hex << rawData_X << " Raw Y: "<< hex << rawData_Y << " Raw Z: "<< hex << rawData_Z<< endl;
	accelerationX = (signed int)(((signed int)rawData_X) * 3.9);
	accelerationY = (signed int)(((signed int)rawData_Y) * 3.9);
	accelerationZ = (signed int)(((signed int)rawData_Z) * 3.9);
	pitch = 180 * atan (accelerationX/sqrt(accelerationY*accelerationY + accelerationZ*accelerationZ))/M_PI;
	roll = 180 * atan (accelerationY/sqrt(accelerationX*accelerationX + accelerationZ*accelerationZ))/M_PI;
	//cout<<"acceleration X: " << dec << accelerationX << " acceleration Y: "<< dec << accelerationY << " acceleration Z: "<< dec << accelerationZ<< endl;
	//cout<<"acceleration X: " <<  accelerationX << " acceleration Y: "<<  accelerationY << " acceleration Z: "<< accelerationZ<< endl;
	return (0);
}

signed int ADXL345Accelerometer::getAcceleration_X(){
	return (accelerationX);
}

signed int ADXL345Accelerometer::getAcceleration_Y(){
	return (accelerationY);
}

signed int ADXL345Accelerometer::getAcceleration_Z(){
	return (accelerationZ);
}
signed int ADXL345Accelerometer::getPitch(){
	return (pitch);
}
signed int ADXL345Accelerometer::getRoll(){
	return (roll);
}
ADXL345Accelerometer::~ADXL345Accelerometer() {
	// TODO Auto-generated destructor stub
}

