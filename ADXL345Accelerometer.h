//============================================================================
// Name        : ADXL345Accelerometer.h
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

#ifndef ADXL345ACCELEROMETER_H_
#define ADXL345ACCELEROMETER_H_

#define ADXL345_POWER_CTL		0x2D	// R/W Power saving features control

/* ADXL345_POWER_CTL Bits */
#define ADXL345_PWRCTL_LINK			(1 << 5)
#define ADXL345_PWRCTL_AUTO_SLEEP	(1 << 4)
#define ADXL345_PWRCTL_MEASURE		(1 << 3)
#define ADXL345_PWRCTL_SLEEP		(1 << 2)
#define ADXL345_PWRCTL_WAKEUP(x)	((x) & 0x3)

#define MAX_BUS 				64

class ADXL345Accelerometer {

private:
	int i2cBus, i2cAddress,i2cHandle;
	char namebuf[MAX_BUS];

	signed int accelerationX, accelerationY, accelerationZ;
	signed int roll,pitch;
	signed short rawData_X, rawData_Y, rawData_Z;
	int opResult, tenBitAddress;

	float deviceTemperature;

public:
	ADXL345Accelerometer(int bus, int address);
	int initAccelerometer();
	unsigned int getAccelerometer_ID();
	void SetPowerMode(unsigned char );
	int getAccelerationData();
	signed int getAcceleration_X();
	signed int getAcceleration_Y();
	signed int getAcceleration_Z();
	signed int getPitch();
	signed int getRoll();

	virtual ~ADXL345Accelerometer();
};

#endif /* ADXL345ACCELEROMETER_H_ */
