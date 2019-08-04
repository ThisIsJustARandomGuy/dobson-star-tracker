/*
 * Moon.cpp
 *
 *  Created on: 03.08.2019
 *      Author: lukas
 */

#include <Arduino.h>
#include "Moon.h"

Moon::Moon() {
	// TODO Auto-generated constructor stub

}
// Azimuth in steps lol
long Moon::getAzimuth() {
	return (long)random(0, 12000);
}

long Moon::getElevation() {
	return (long)random(0, 12000);
}
