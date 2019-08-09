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


void initConversion();
void loopConversion();
void AZ_to_EQ();
bool parseCommands(MultiStepper &motors, bool homingMode);
void receiveCommandChar();
bool communication(MultiStepper &motors, bool homingMode);
void read_sensors(AccelStepper &az, AccelStepper &el);

float ecliptic_longitude_sun(float T);
float deg2rad(float deg);
float rad2deg(float rad);
void EQ_to_AZ(MultiStepper &motors, AccelStepper &az, AccelStepper &el,
		bool justHomed);

#endif /* CONVERSION_H_ */
