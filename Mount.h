/*
 * Mount.h
 *
 *  Created on: 28.08.2019
 *      Author: lukas
 */

#ifndef MOUNTS_MOUNT_H_
#define MOUNTS_MOUNT_H_

#include "./location.h"

class Mount {
public:
	virtual void calculateMotorTargets();

	virtual void move();

	void updateGpsPosition(TelescopePosition pos) {
		_gpsPosition = pos;
	}

	void setHomed() {
		_isHomed = true;
	}

	void setTarget(RaDecPosition target) {
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
	TelescopePosition _gpsPosition;

	bool _homedLastIteration = false;
	bool _isHomed = false;

	RaDecPosition _target;

	RaDecPosition _lastTarget;

	RaDecPosition _currentPosition;


};
#endif /* MOUNTS_MOUNT_H_ */
