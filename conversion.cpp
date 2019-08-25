/*
 * Conversion.cpp
 *
 *  Created on: 03.08.2019
 *      Author: lukas
 */
#ifndef CONVERSION_H_

#include <Arduino.h>
#include <AccelStepper.h>

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

/**
 * This gets called whenever
 */
bool parseCommands(MultiStepper &motors, bool homingMode) {
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
			int hrs = (receivedChars[3] - '0') * 10 + (receivedChars[4] - '0');
			int mins = (receivedChars[6] - '0') * 10
					+ (receivedChars[7] - '0');
			int secs = (receivedChars[9] - '0') * 10
					+ (receivedChars[10] - '0');

			// This is what we return to Stellarium when it asks for the current right ascension
			sprintf(txAR, "%02d:%02d:%02d#", int(hrs), int(mins), int(secs));

			// This is our ultimate target value
			last_ra_deg = ra_deg;
			//ra_deg = 360. * (hrs / 24. + mins / (24. * 60.) + secs / (24. * 3600.));
			ra_deg = (hrs + mins / 60. + secs / 3600.) * 15;

			Serial.print("1");
		} else if (receivedChars[0] == 'S' && receivedChars[1] == 'd') {
			// Set target DEC

			int multi = (receivedChars[3] == '+') ? 1 : -1; // TODO bool may be more memory efficient

			int deg = ((receivedChars[4] - '0') * 10 + (receivedChars[5] - '0'));

			long mins = (receivedChars[7] - '0') * 10
					+ (receivedChars[8] - '0');
			
			long secs = (receivedChars[10] - '0') * 10
					+ (receivedChars[11] - '0');

			sprintf(txDEC, "%c%02d%c%02d:%02d#", receivedChars[3], int(deg),
					223, int(mins), int(secs));

			last_dec_deg = dec_deg;

			//dec_deg = multi * (deg + mins / 60. + secs / 3600.);
			dec_deg = (multi * deg) + mins / 60. + secs / 3600.;
			isPositiveDeclination = multi > 0;

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
						Serial.println(
								add > 0 ?
										"Add 1deg ascension" :
										"Sub 1 deg ascension");
					} else if (receivedChars[5] == 'D') {
						// DBGMD[+/-]XX Move Declination to +/-XX
						dec_deg += add;
						Serial.println(
								add > 0 ?
										"Add 1deg declination" :
										"Sub 1 deg declination");
					}
				} else {
				// Debug move to position stored in debugPositions[targetIndex]
				int targetIndex = receivedChars[4] - '0';
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
				}
				}
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
bool communication(MultiStepper &motors, bool homingMode) {
	receiveCommandChar();
	return parseCommands(motors, homingMode);
}


float ecliptic_longitude_sun(float T) {
	float k = 2 * pi / 360;

	//mean anomaly, degree
	float M = 357.52910 + 35999.05030 * T - 0.0001559 * T * T
			- 0.00000048 * T * T * T;
	// mean longitude, degree
	float L0 = 280.46645 + 36000.76983 * T + 0.0003032 * T * T;
	// Sun's equation of center
	float DL = (1.914600 - 0.004817 * T - 0.000014 * T * T) * sin(k * M)
			+ (0.019993 - 0.000101 * T) * sin(k * 2 * M)
			+ 0.000290 * sin(k * 3 * M);

	// true longitude, degree
	return L0 + DL;
}


long jul_day_2k = 2451545;

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


/**
 * This function converts from right ascension + declination to azimuth and altitude.
 * Returns true if a move was done
 */
bool EQ_to_AZ(MultiStepper &motors, AccelStepper &az_s, AccelStepper &el_s,
		FuGPS &gps, Position &pos, bool justHomed) {
	// KEEP TIME
	// TODO Month rollover etc
	// This will be handled by GPS eventually, but we may need a better way to prevent bugs during testing

	current_lat = pos.latitude > 1.00 ? pos.latitude : LAT;
	current_lng = pos.longitude > 1.00 ? pos.longitude : LNG;

	long passed_seconds = (millis() * TIME_FACTOR) / 1000; // Seconds that have passed since exceution started

	int current_day = 24;
	int current_hour = 23; //gps.Hours;
	int current_minute = 02; //gps.Minutes;
	int current_second = passed_seconds; //gps.Seconds;

	// Second, Minute and Hour rollover
	while (current_second >= 60) {
		current_minute++;
		current_second -= 60;
	}
	while (current_minute >= 60) {
		current_hour++;
		current_minute -= 60;
	}
	while (current_hour >= 24) {
		current_day += 1;
		current_hour -= 24;
	}

	// Current time in UTC with hour rollover
	float current_utc = (current_hour + TIMEZONE_CORRECTION_H) + (current_minute / 60.0)
			+ (current_second / 3600.0);

	while (current_utc < 0) {
		current_utc += 24;
	}
	while (current_utc >= 24) {
		current_utc -= 24;
	}

	// Julian Days since 2000
	float jul_days_s2k = (((current_minute / 60.0) + (current_hour)) / 24.0)
			+ current_jul_magic_mo + current_day + current_jul_magic_year;
	// END TIMEKEEPING

	// number of Julian centuaries since Jan 1, 2000, 12 UT
	const float jul_centuaries = jul_days_s2k / 36525;

	// calculate the local siderian time (in degrees)
	float local_siderian_time = 100.46 + 0.985647 * jul_days_s2k + current_lng
			+ 15 * current_utc;

	// Ensure that local_siderian_time is greater than 0 degrees
	while (local_siderian_time < 0) {
		local_siderian_time += 360;
	}

	// The hour angle is the difference between the local siderian time and the right ascension in degrees of our target object...
	float hour_angle = local_siderian_time - ra_deg;
	// ...but we need to ensure that it's greater than 0
	while (hour_angle < 0) {
		hour_angle += 360.0; // Maybe this is buggy
	}

	// Convert various values from degrees to radians since the trigonometry functions work with radians
	const float rad_declination = deg2rad(dec_deg);
	const float rad_current_lat = deg2rad(current_lat); // TODO this can be a constant if GPS_FIXED_POS is set
	const float rad_hour_angle = deg2rad(hour_angle);

	// These values are used multiple times throughout calculations, so we pull them out
	const float sin_rad_declination = sin(rad_declination);
	const float sin_rad_current_lat = sin(rad_current_lat);
	const float cos_rad_current_lat = cos(rad_current_lat);

	// Calculate altitude
	const float sin_altitude = sin_rad_declination * sin_rad_current_lat
			+ cos(rad_declination) * cos_rad_current_lat
					* cos(rad_hour_angle);

	const float rad_altitude = asin(sin_altitude);

	// This is our resulting altitude in degrees
	const float deg_altitude = rad2deg(rad_altitude); // RESULT

	// Calculate azimuth
	const float cos_a = (sin_rad_declination - sin(rad_altitude) * sin_rad_current_lat)
			/ (cos(rad_altitude) * cos_rad_current_lat);
	const float rad_a = acos(cos_a);
	const float a = rad2deg(cos_a);
	
	// Azimuth in degrees is either 360-a or a, depending on whether sin(rad_hour_angle) is positive.
	// It be like it is and it's also the second part of the result
	float deg_azimuth;
	// TODO This is buggy. When az=-57.3
	if (sin(rad_hour_angle) > 0) {
		deg_azimuth = 360 - a;
	} else {
		deg_azimuth = a;
	}
	deg_azimuth = a;

	// TODO Is this really necessary to have it this complex? No
	const long desired_az = (long) (
			(deg_azimuth / 360) * AZ_STEPS_PER_REV);
	const long desired_alt = (long) (
			(deg_altitude / 360) * ALT_STEPS_PER_REV);

	if (justHomed || !isHomed) {
		DEBUG_PRINT("Just homed to ");
		DEBUG_PRINT(desired_az);
		DEBUG_PRINT(" / ");
		DEBUG_PRINTLN(desired_alt);

		az_s.setCurrentPosition(desired_az);
		el_s.setCurrentPosition(desired_alt);

		last_desired_az = desired_az;
		last_desired_dec = desired_alt;
		isHomed = true;
	} else {
		az_s.moveTo(desired_az);
		el_s.moveTo(desired_alt);
		//az_s.runToNewPosition(last_desired_az - desired_az);
		//el_s.runToNewPosition(last_desired_dec - desired_alt);

		// From here on only debug outputs happen
		if (last_desired_az - desired_az > 0
				|| last_desired_dec - desired_alt > 0) {
			DEBUG_PRINT_V(
					gps.hasFix() ?
						"GPS: " + String(gps.Satellites, 6) + "S/"
									+ String(gps.Quality) + "Q "
									+ String(gps.Latitude, 6) + "LAT / "
									+ String(gps.Longitude, 6) + "LNG" :
						"GPS: N/A");
			DEBUG_PRINT("; LAT ");
			DEBUG_PRINT(current_lat);
			DEBUG_PRINT(", LNG ");
			DEBUG_PRINT(current_lng);
			DEBUG_PRINT("; RA ");
			DEBUG_PRINT(ra_deg);
			DEBUG_PRINT(" and DEC ");
			DEBUG_PRINT(dec_deg);
			DEBUG_PRINT(" to ALTAZ is: ALT ");
			DEBUG_PRINT(deg_altitude);
			DEBUG_PRINT("° AZ ");
			DEBUG_PRINT(deg_azimuth);
			DEBUG_PRINT("° HA ");
			DEBUG_PRINT(rad_hour_angle);
			DEBUG_PRINT("° sin(HA) ");
			DEBUG_PRINT(sin(rad_hour_angle));
			DEBUG_PRINT("ANG; The time is: ");
			DEBUG_PRINT(current_hour);
			DEBUG_PRINT(":");
			DEBUG_PRINT(current_minute);
			DEBUG_PRINT(":");
			DEBUG_PRINT(current_second);
			DEBUG_PRINT("Steppers: az");
			DEBUG_PRINT(desired_az);
			DEBUG_PRINT("/dec ");
			DEBUG_PRINT(desired_alt);
			DEBUG_PRINT(" diff ");
			DEBUG_PRINT(last_desired_az - desired_az);
			DEBUG_PRINT(" / ");
			DEBUG_PRINT(last_desired_dec - desired_alt);
			DEBUG_PRINT("°; Reported: az");
			DEBUG_PRINT(az_s.currentPosition());
			DEBUG_PRINT("/dec ");
			DEBUG_PRINT(el_s.currentPosition());
		}
	}

	if (last_desired_az - desired_az > 0
			|| last_desired_dec - desired_alt > 0) {
		last_desired_az = desired_az;
		last_desired_dec = desired_alt;
		return true;
	} else {
		return false;
	}
}
//#endif

#endif
