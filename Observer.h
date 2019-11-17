#pragma once

#include "./location.h"

/*
 * This is the base class for GpsPositionProvider and MockPositionProvider which are used
 * to get the GPS position or the fixed latitude and longitude set in config.h
 */
class Observer {
public:
	virtual void initialize();
	
	virtual void updatePosition();
	
	virtual bool hasValidPosition();

	virtual void printDebugInfo();

	void setPosition(ObserverPosition position) {
		_position = position;
	}

	ObserverPosition getPosition() {
		return _position;
	}

	float altitude() {
		return _position.altitude;
	}

	float latitude() {
		return _position.latitude;
	}

	float longitude() {
		return _position.longitude;
	}

protected:
	ObserverPosition _position;
};
