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


class DirectDrive : public Mount {
public:
	DirectDrive(AccelStepper& azimuthStepper, AccelStepper& altitudeStepper, FuGPS& gps);

	// This runs at the very end of the Arduino setup() function and sets the operating mode and initial target
	void initialize();

	// Calculates the next targets for the steppers, based on the GPS position, current time and target
	// This does not yet update the stepper motor targets, but stores them in the protected member variable _steppersTarget
	// It also calls the azAltToRaDec() method with the current stepper position and stores the result
	void calculateMotorTargets();

	// Sets the actual motor targets, based on the contents of _steppersTarget
	void move();

	// This is set to true at the end of the move() method, if at least one stepper target was changed
	// It is then reset at the beginning of calculateMotorTargets()
	bool _didMove = false;

	#ifdef DEBUG_TIMING
		// How long calculateMotorTargets took to execute
		long _lastCalcMicros = 0;
	#endif

protected:
	// Reference to the azimuth stepper
	AccelStepper& _azimuthStepper;

	// Reference to the altitude stepper
	AccelStepper& _altitudeStepper;

	// Reference to the GPS module
	FuGPS& _gps;

	// Current stepper target position for the steppers (in steps). It is written to at the end of calculateMotorTargets()
	AzAlt<long> _steppersTarget;

	// Target position for the steppers before the last move. It is written to at the end of move()
	AzAlt<long> _steppersLastTarget;
};
