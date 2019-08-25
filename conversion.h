/*
 * conversion.h
 *
 *  Created on: 03.08.2019
 *      Author: lukas
 */

#ifndef CONVERSION_H_
#define CONVERSION_H_

#define DEBUG true

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <FuGPS.h>

#include "location.h"

void initCommunication();
bool parseCommands(MultiStepper &motors, bool homingMode);
void receiveCommandChar();
bool communication(MultiStepper &motors, bool homingMode);

float ecliptic_longitude_sun(float T);
float deg2rad(float deg);
float rad2deg(float rad);
bool EQ_to_AZ(MultiStepper &motors, AccelStepper &az, AccelStepper &el,
		FuGPS &gps, Position &pos,
		bool justHomed);

#endif /* CONVERSION_H_ */
