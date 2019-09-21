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
	double azimuth;
	double altitude;
};


class Dobson: public Mount {
public:
	Dobson(AccelStepper &azimuthStepper, AccelStepper &altitudeStepper, FuGPS &gps);

	// Calculates the next targets for the steppers, based on the GPS position, current time and target
	// It also calls the interpolatePosition() method
	void calculateMotorTargets();
	
	// Calculates the current position in Ra/Dec, which is reported back to Stellarium or other connected tools
	// This does not yet update the stepper motor targets, but stores them in the protected member variable _steppersTarget
	void interpolatePosition();

	// Sets the actual motor targets, based on the contents of _steppersTarget
	void move();

	// How long calculateMotorTargets took to execute (including interpolatePosition())
	long _lastCalcMicros = 0;

	// It is set to true at the end of the move() method, if at least one stepper target was changed
	// It is then reset at the beginning of calculateMotorTargets()
	bool _didMove = false;

protected:
	// Reference to the azimuth stepper
	AccelStepper &_azimuthStepper;
	
	// Reference to the altitude stepper
	AccelStepper &_altitudeStepper;

	// Reference to the GPS module
	FuGPS &_gps;

	// This is written to (and used) by calculateMotorTargets() and just used by interpolatePosition()
	double _currentLocalSiderealTime;
	
	// Target position in degrees
	AzAltPosition _targetDegrees;

	// The position the steppers were, when homing was performed
	AzAltPosition _steppersHomed;

	// Current stepper target position for the steppers (in steps). It is written to at the end of calculateMotorTargets()
	AzAltPosition _steppersTarget;

	// Target position for the steppers before the last move. It is written to at the end of move()
	AzAltPosition _steppersLastTarget;
};
