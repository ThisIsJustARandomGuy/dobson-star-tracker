/*
 * conversion.h
 *
 *  Created on: 03.08.2019
 *      Author: lukas
 */

#ifndef CONVERSION_H_
#define CONVERSION_H_

#include <AccelStepper.h>


void initConversion();
void loopConversion();
void AZ_to_EQ(AccelStepper &az, AccelStepper &el);
void showNewData(AccelStepper &az, AccelStepper &el);
void recvWithStartEndMarkers();
void communication(AccelStepper &az, AccelStepper &el);
void read_sensors(AccelStepper &az, AccelStepper &el);

#endif /* CONVERSION_H_ */
