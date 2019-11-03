#pragma once
/*
 * conversion.h
 *
 *  Created on: 03.08.2019
 *      Author: lukas
 */

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <FuGPS.h>

#include "./Mount.h"
#include "location.h"


void initCommunication(Mount& telescope);
bool parseCommands(Mount &telescope, FuGPS &gps, bool homingMode);
void receiveCommandChar();
bool handleSerialCommunication(Mount &telescope, FuGPS &gps, bool homingMode);