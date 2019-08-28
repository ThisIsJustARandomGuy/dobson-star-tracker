/*
 * Conversion.cpp
 *
 *  Created on: 03.08.2019
 *      Author: lukas
 */
#ifndef CONVERSION_H_

#include <Arduino.h>
#include <AccelStepper.h>
#include <Time.h>

#include "./config.h"
#include "./conversion.h"
#include "./location.h"


// DESIRED COORDINATES. THIS IS IMPORTANT
float ra_h = 16;
float ra_m = 41.7;
float ra_deg = (ra_h + ra_m / 60) * 15;

float dec_d = 36;
float dec_m = 28;
float dec_deg = dec_d + dec_m / 60;

boolean isPositiveDeclination = false;

boolean isHomed = false;

char txAR[10]; // Gets reported to stellarium when it asks for right ascension "16:41:34#";
char txDEC[11]; // Same as above with declination. sprintf(txDEC, "+36d%c28:%02d#", 223, int(dec_m), 0);

const byte numChars = 32;
char receivedChars[numChars];

bool newData = false; // Gets set to true whenever a complete command is buffered
static boolean recvInProgress = false; // True while a command is being received
static byte ndx = 0; // Number of command character received
const char startMarker = ':'; // Commands begin with this character
const char endMarker = '#'; // Commands end with this character

const double pi = 3.14159265358979324;

void initCommunication() {
	sprintf(txAR, "%02d:%02d:%02d#", int(ra_h), int(ra_m),
			int(0));
	sprintf(txDEC, "%c%02d%c%02d:%02d#", dec_d > 0 ? '+' : '-', int(dec_d), 223,
			int(dec_m), int(0));
}

void receiveCommandChar() {

	char rc;

	if (Serial.available() > 0 && newData == false) {
		rc = Serial.read();

		if (recvInProgress == true) {
			//Serial.print("ReceivedChar: ");
			//Serial.println(rc);
			if (rc != endMarker) {
				receivedChars[ndx] = rc;
				ndx++;
				if (ndx >= numChars) {
					ndx = numChars - 1;
				}
			} else {
				//Serial.println("Got end marker");
				receivedChars[ndx] = '\0'; // terminate the string
				recvInProgress = false;
				ndx = 0;
				newData = true;
			}
		} else if (rc == startMarker) {
			//Serial.println("Got start marker");
			recvInProgress = true;
		}
	}
}

float last_ra_deg = ra_deg;
float last_dec_deg = dec_deg;

long positions[2];

#define SERIAL_CMD_INVALID_COMMAND     -1
#define SERIAL_CMD_GET_RIGHT_ASCENSION 0
#define SERIAL_CMD_GET_DECLINATION     1
#define SERIAL_CMD_MOVE_STOP           2
#define SERIAL_CMD_MOVE_START          3
#define SERIAL_CMD_SET_RIGHT_ASCENSION 4
#define SERIAL_CMD_SET_DECLINATION     5

float debugPositions[][2] = { { 16.7, 36.5 }, { 16.7, 35.5 },
		{ 16.7, 37.5 }, {
		15.7, 36.5 }, { 14.7, 36.5 }, { 17.7, 36.5 }, { 18.7, 36.5 }, { 19.7,
		36.5 }, { 17.7, 38.5 }, { 17.7, 39.5 } };
const int maxDebugPos = 9;

// This Macro converts a character to an integer
#define char_to_int(x) (x - '0')
// This Macro converts two characters to an integer. Example: ctoi10('2', '3') => 23
#define multi_char_to_int(x, y) ((x - '0') * 10 + (y - '0'))

/**
 * This gets called whenever
 */
bool parseCommands(Mount &telescope, FuGPS &gps, bool homingMode) {
	if (newData == true) {
		if (receivedChars[0] == 'G' && receivedChars[1] == 'R') {
			// GR: Get Right Ascension
			Serial.print(txAR);
		} else if (receivedChars[0] == 'G' && receivedChars[1] == 'D') {
			// GD: Get Declination
			Serial.print(txDEC);
		} else if (receivedChars[0] == 'Q') {
			// Stop moving
			// TODO Implement this
		} else if (receivedChars[0] == 'M' && receivedChars[1] == 'S') {
			// MS: Move Start
			Serial.print("0");

			// TODO Homing code needs to be better. It has to disable the steppers and there must be some way to enable/disable it
			// If homing mode is true we set isHomed to true
			// and return true to indicate to the loop() function that homing is complete.
			if (homingMode) {
				isHomed = true;
				newData = false;
				return true;
			} else {
				// :Sr,16:41:00#
				// :Sr,15:41:00#
				// :Sd,+36:28:00#
				// :Sd,+16:30:00#
				// Otherwise send a move command to the motors.
				// This will move both motors such that they end their moves at the same time
				// motors.moveTo(positions);
			}
		} else if (receivedChars[0] == 'S' && receivedChars[1] == 'r') {
			// Set Right Ascension (in hours, minutes and seconds)

			// Parse the coordinates part of the command to integers
			int hrs = multi_char_to_int(receivedChars[3], receivedChars[4]);
			int mins = multi_char_to_int(receivedChars[6], receivedChars[7]);
			int secs = multi_char_to_int(receivedChars[9], receivedChars[10]);

			// This is what we return to Stellarium when it asks for the current right ascension
			sprintf(txAR, "%02d:%02d:%02d#", int(hrs), int(mins), int(secs));

			// This is our ultimate target value
			last_ra_deg = ra_deg;
			//ra_deg = 360. * (hrs / 24. + mins / (24. * 60.) + secs / (24. * 3600.));
			ra_deg = (hrs + mins / 60. + secs / 3600.) * 15;

			RaDecPosition scopeTarget = telescope.getTarget();
			scopeTarget.rightAscension = ra_deg;
			telescope.setTarget(scopeTarget);

			Serial.print("1");
		} else if (receivedChars[0] == 'S' && receivedChars[1] == 'd') {
			// Set target Declination (in degrees, minutes and seconds)

			int multi = (receivedChars[3] == '+') ? 1 : -1; // TODO bool may be more memory efficient

			int deg = multi_char_to_int(receivedChars[4], receivedChars[5]);
			int mins = multi_char_to_int(receivedChars[7], receivedChars[8]);
			int secs = multi_char_to_int(receivedChars[10], receivedChars[11]);


			sprintf(txDEC, "%c%02d%c%02d:%02d#", receivedChars[3], deg, 223,
					mins, secs);

			last_dec_deg = dec_deg;

			//dec_deg = multi * (deg + mins / 60. + secs / 3600.);
			dec_deg = (multi * deg) + mins / 60. + secs / 3600.;
			isPositiveDeclination = multi > 0;

			RaDecPosition scopeTarget = telescope.getTarget();
			scopeTarget.declination = dec_deg;
			telescope.setTarget(scopeTarget);


			Serial.print("1");
		} else if (receivedChars[0] == 'D' && receivedChars[1] == 'B'
				&& receivedChars[2] == 'G') {
			// DEBUG messages
			if (receivedChars[3] == 'M') {
				if (receivedChars[4] == 'I' || receivedChars[4] == 'D') {
					// Increase or Decrease Azimuth or Declination
					int add = receivedChars[4] == 'I' ? 1 : -1;
					if (receivedChars[5] == 'A') {
						// DBGMAXXX Move Azimuth to XXX
						ra_deg += add;
						RaDecPosition pos = telescope.getTarget();
						pos.rightAscension += add;
						telescope.setTarget(pos);
						Serial.println(
								add > 0 ?
										"Add 1 deg ascension" :
										"Sub 1 deg ascension");
					} else if (receivedChars[5] == 'D') {
						// DBGMD[+/-]XX Move Declination to +/-XX
						dec_deg += add;
						RaDecPosition pos = telescope.getTarget();
						pos.declination += add;
						telescope.setTarget(pos);
						Serial.println(
								add > 0 ?
										"Add 1 deg declination" :
										"Sub 1 deg declination");
					}
				} else {
				// Debug move to position stored in debugPositions[targetIndex]
					int targetIndex = char_to_int(receivedChars[4]);
					if (targetIndex > maxDebugPos) {
						Serial.println("Invalid index");
					} else {
						Serial.println("Moving from ALT / DEC");
						Serial.println(ra_deg);
						Serial.println(dec_deg);
						Serial.println("Moving to ALT / DEC");
						Serial.println(debugPositions[targetIndex][0] * 15);
						Serial.println(debugPositions[targetIndex][1]);
						ra_deg = debugPositions[targetIndex][0] * 15;
						dec_deg = debugPositions[targetIndex][1];
						RaDecPosition newPos = { debugPositions[targetIndex][0] * 15, debugPositions[targetIndex][1] };
						telescope.setTarget(newPos);
					}
				}
			} else if (receivedChars[3] == 'H') {
				// Home the scope
				telescope.setHomed();
			} else if (receivedChars[3] == 'D' && receivedChars[4] == 'M') {
				// Disable Motors and Pause for X seconds
				digitalWrite(ALT_ENABLE_PIN, HIGH);
				digitalWrite(AZ_ENABLE_PIN, HIGH);
				Serial.print("Disabling motors for: ");
				Serial.println((receivedChars[5] - '0') * 1000);
				delay((receivedChars[5] - '0') * 1000);
				Serial.println("Continuing");
				digitalWrite(ALT_ENABLE_PIN, LOW);
				digitalWrite(AZ_ENABLE_PIN, LOW);
			} else if (receivedChars[3] == 'G' && receivedChars[4] == 'P' && receivedChars[5] == 'S') {
				// GPS Debug info
				Serial.println("GPS Status: ");
				Serial.print("Alive      ... ");
				Serial.println(gps.isAlive() ? "Yes" : "No");
				Serial.print("Fix        ... ");
				Serial.println((gps.hasFix() ? "Yes" : "No"));
				Serial.print("Satellites ... ");
				Serial.println(String(gps.Satellites, 6));
				Serial.print("Quality    ... ");
				Serial.println(String(gps.Quality, 6));
				Serial.print("Altitude   ... ");
				Serial.println(String(gps.Altitude, 6));
				Serial.print("Latitude   ... ");
				Serial.println(String(gps.Latitude, 6));
				Serial.print("Longitude  ... ");
				Serial.println(String(gps.Longitude, 6));
			}
		} else if (receivedChars[0] == 'H' && receivedChars[1] == 'L'
				&& receivedChars[2] == 'P') {
			Serial.println(":HLP# Print available Commands");
			Serial.println(":GR# Get Right Ascension");
			Serial.println(":GD# Get Declination");
			Serial.println(
					":Sr,HH:MM:SS# Set Right Ascension; Example: :Sr,12:34:56#");
			Serial.println(
					":Sd,[+/-]DD:MM:SS# Set Declination (DD is degrees) Example: :Sd,+12:34:56#");
			Serial.println(":MS# Start Move");
			Serial.println(":DBGM[0-5]# Move to debug position X");
			Serial.println(":DBGMIA# Increase Ascension by 1 degree");
			Serial.println(":DBGMDA# Decrease Ascension by 1 degree");
			Serial.println(":DBGMID# Increase Declination by 1 degree");
			Serial.println(":DBGMDD# Decrease Declination by 1 degree");
			Serial.println(":DBGDM[0-9]# Disable Motors for X seconds");
		} else {
			Serial.println("ERROR: Unknown command");
			Serial.println(receivedChars);
		}
		newData = false;
	}
	return false;
}

// Returns true if homing command was sent
bool handleSerialCommunication(Mount &telescope, FuGPS &gps, bool homingMode) {
	receiveCommandChar();
	return parseCommands(telescope, gps, homingMode);
}


// According to http://www.geoastro.de/elevaz/basics/index.htm
const long timeLast = 0;

//#ifdef GPS_FIXED_POS
long current_lat = LAT;
long current_lng = LNG;
//#else
// GPS CODE ensues
//#endif


float deg2rad(float degs) {
	return degs * pi / 180;
}

float rad2deg(float rad) {
	return rad * 180 / pi;
}

volatile float rlyaz = 0;
volatile float rlydec = 0;

// Stepper positions when homing was performed
long home_az = 0;
long home_alt = 0;

// The last reported positions
long last_reported_az = 0;
long last_reported_dec = 0;

long last_desired_az = 0;  // Last azimuth (IN STEPS) we desired
long last_desired_dec = 0; // Last declination (IN STEPS) we desired


long current_year = 2019;
long current_month = 8;
// TODO!!!!!
// This needs to be addressed next. We need to create (e.g. copy) either a table containing
// these values, or we need to calculate them
float current_jul_magic_year = 6938.5; // -731.5=1998 //6938.5=2019
float current_jul_magic_mo = 212; // 212=August. This is why we need lookup tables


#endif
