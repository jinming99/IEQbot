/*
	K_Series.h -Library for interfacing with a K-series sensor
	Created by Jason Berger
	for CO2METER.com
	OCT-12-2012

*/

#if defined (SPARK)
#include "application.h"
#else
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#endif	//Spark

#ifndef kSeries_h
#define kSeries_h

#if !defined (SPARK)
#include <SoftwareSerial.h> 	//Virtual Serial library
#endif

class kSeries
{
  public:
    kSeries(uint8_t Rx, uint8_t Tx);
    double getCO2(char format);
	double getTemp(char unit);
	double getRH();
	bool _K33;
	bool _ASCII;
	int cmdInit();
  private:
#if defined (SPARK)
	USARTSerial* _Serial;
#else
	SoftwareSerial* _Serial;
#endif
	void chkSensorType();
	void chkASCII();
	void chkK33();
	int sendRequest(int reqType, int respSize, int respInd);
	long getResp(int size, int strt);	
	void wait(int ms);
};

#endif
