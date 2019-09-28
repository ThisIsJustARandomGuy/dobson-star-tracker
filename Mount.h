/*
 * Mount.h
 *
 *  Created on: 28.08.2019
 *      Author: lukas
 */

#ifndef MOUNTS_MOUNT_H_
#define MOUNTS_MOUNT_H_

#include "./location.h"

enum Mode {
	// INITIALIZING
	// This is only on when initializing critical hardware such as the stepper drivers.
	// When this mode is active, you can not rely on any value reported by the telescope (stepper pos, GPS pos, desired pos etc.)
	//
	// Motors are on: YES
	// Tracking active: NO
	// Desired/Reported stepper position updates: YES
	INITIALIZING,

	// HOMING
	// While this mode is on, the telescope stands still and assumes that it is correctly pointing at the desired position.
	// Once a target gets selected the telescope assumes that this is where it's pointed at and starts tracking it
	//
	// Motors are on: YES
	// Tracking active: NO
	// Desired/Reported stepper position updates: YES
	HOMING,
	
	// TRACKING
	// While this mode is on, the telescope tracks the desired position
	//
	// Motors are on: YES
	// Tracking active: YES
	// Desired/Reported stepper position updates: YES
	TRACKING
};

class Mount {
public:

	Mode getMode() {
		return _mode;
	}

	void setMode(Mode mode) {
		_mode = mode;
		DEBUG_PRINTLN("New opmode is: ");
		DEBUG_PRINTLN(_mode);
	}

	virtual void initialize();

	virtual void calculateMotorTargets();

	virtual void move();

	void updateGpsPosition(TelescopePosition pos) {
		_gpsPosition = pos;
	}

	void setHomed(const bool value = true) {
		_isHomed = value;
		if (value) {
			setMode(Mode::TRACKING);
		}
		else {
			setMode(Mode::HOMING);
		}
	}

	bool isHomed() {
		return _isHomed;
	}

	void ignoreUpdates(const bool value = true) {
		_ignoreMoves = value;
	}

	void setTarget(RaDecPosition target) {
		DEBUG_PRINTLN("Target set");
		DEBUG_PRINTLN(target.rightAscension);
		DEBUG_PRINTLN(target.declination);
		_lastTarget = _target;
		_target = target;
	}

	RaDecPosition getTarget() {
		return _target;
	}

	RaDecPosition getCurrentPosition() {
		return _currentPosition;
	}

protected:
	// Which mode the telescope is curently in. See above for what the constants do
	Mode _mode = Mode::INITIALIZING;

	TelescopePosition _gpsPosition;

	bool _ignoredMoveLastIteration = false;
	bool _isHomed = false;
	bool _ignoreMoves = false;

	RaDecPosition _target;

	RaDecPosition _lastTarget;

	RaDecPosition _currentPosition;


};
#endif /* MOUNTS_MOUNT_H_ */
