#pragma once

#include <FuGPS.h>

#include "./Observer.h"



class GpsObserver :	public Observer
{
public:
	GpsObserver();

	void initialize();

	void updatePosition();

	bool hasValidPosition();

	void printDebugInfo();

protected:
	//GPS module
	FuGPS _gps;

	bool _gpsAlive = false;
	bool _didSetTimeWithoutFix = false;
	bool _didSetTimeWithFix = false;
};

