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

#include "Mounts/Mount.h"
#include "location.h"

void initCommunication();
bool parseCommands(Mount &telescope, FuGPS &gps, bool homingMode);
void receiveCommandChar();
bool handleSerialCommunication(Mount &telescope, FuGPS &gps, bool homingMode);

float ecliptic_longitude_sun(float T);
float deg2rad(float deg);
float rad2deg(float rad);
long handleMovement(Mount &telescope, AccelStepper &stepper_azimuth,
		AccelStepper &stepper_altitude,
		FuGPS &gps, TelescopePosition &pos,
		bool justHomed);

#endif /* CONVERSION_H_ */
