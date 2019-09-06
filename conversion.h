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

 // This Macro converts a character to an integer
#define char_to_int(x) (x - '0')
// This Macro converts two characters to an integer. Example: ctoi10('2', '3') => 23
#define multi_char_to_int(x, y) ((x - '0') * 10 + (y - '0'))


void initCommunication();
bool parseCommands(Mount &telescope, FuGPS &gps, bool homingMode);
void receiveCommandChar();
bool handleSerialCommunication(Mount &telescope, FuGPS &gps, bool homingMode);