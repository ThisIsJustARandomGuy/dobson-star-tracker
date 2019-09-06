#pragma once
/*
 * Dobson.h
 *
 *  Created on: 27.08.2019
 *      Author: lukas
 */

#include <AccelStepper.h>
#include <FuGPS.h>

#include "./Mount.h"
#include "./location.h"
#include "./Dobson.h"


struct AzAltPosition {
	long azimuth;
	long altitude;
};


class Dobson: public Mount {
public:
	Dobson(AccelStepper &azimuthStepper, AccelStepper &altitudeStepper, FuGPS &gps);

	void calculateMotorTargets();
	void interpolatePosition();
	//void Dobson::CalculateAltAz(double RA, double Dec, double Lat, double Long);

	void move();

	long _lastCalcMicros = 0;
	bool _didMove = false;

protected:
	AccelStepper &_azimuthStepper;
	AccelStepper &_altitudeStepper;
	FuGPS &_gps;
	float _azimuthDegrees;
	float _altitudeDegrees;

	AzAltPosition _steppersHomed;
	AzAltPosition _steppersTarget;
	AzAltPosition _steppersLastTarget;
};
