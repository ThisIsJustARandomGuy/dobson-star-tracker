/*
 * Telescope.h
 *
 *  Created on: 08.08.2019
 *      Author: lukas
 */

#ifndef TELESCOPE_H_
#define TELESCOPE_H_

#include <AccelStepper.h>
#include <FuGPS.h>
#include "./config.h"

/**
 * This is a telescope. It takes coordinates in right ascension and altitude
 * and makes the telescope move. It does not handle positioning or timing
 */
class Telescope {
private:

	boolean isHomed = false;

	long _desiredAzimuth = 0;
	long _desiredAltitude = 0;

	long _lastDesiredAzimuth = 0;
	long _lastDesiredAltitude = 0;

public:
	Telescope(AccelStepper azimuth, AccelStepper altitude, FuGPS gps);

	// Desired and current right ascension
	void setDesiredRightAscension(long rightAscension);
	long getDesiredRightAscension();
	long getCurrentRightAscension();

	// Desired and current declination
	void setDesiredDeclination(long altitude);
	long getDesiredDeclination();
	long getCurrentDeclination();
};


#endif /* TELESCOPE_H_ */
