/*
 * conversion.h
 *
 *  Created on: 03.08.2019
 *      Author: lukas
 */

#ifndef CONVERSION_H_
#define CONVERSION_H_

#define DEBUG false

#include <AccelStepper.h>


void initConversion();
void loopConversion();
void AZ_to_EQ(AccelStepper &az, AccelStepper &el);
void showNewData(AccelStepper &az, AccelStepper &el);
void recvWithStartEndMarkers();
void communication(AccelStepper &az, AccelStepper &el);
void read_sensors(AccelStepper &az, AccelStepper &el);

float ecliptic_longitude_sun(float T);
float deg2rad(float deg);
float rad2deg(float rad);
void EQ_to_AZ(float ra, float dec, AccelStepper &az_s, AccelStepper &el_s);

#endif /* CONVERSION_H_ */
