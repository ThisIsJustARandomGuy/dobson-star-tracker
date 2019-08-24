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

int latHH = 47;    // this means 40ยบ North
int latMM = 25;
int latSS = 37;

// enter Pole Star right ascention (AR: HH:MM:SS)
int poleAR_HH = 2;    // this means 2 hours, 52 minutes and 16 seconds
int poleAR_MM = 57;
int poleAR_SS = 06;

// enter Pole Star hour angle (H: HH:MM:SS)
int poleH_HH = 89;
int poleH_MM = 20;
int poleH_SS = 50;

// DESIRED COORDINATES. THIS IS IMPORTANT
float ra_h = 16;
float ra_m = 41.7;
float ra_deg = (ra_h + ra_m / 60) * 15;

float dec_d = 36;
float dec_m = 28;
float dec_deg = dec_d + dec_m / 60;



unsigned long seg_sideral = 1003;
const double pi = 3.14159265358979324;
volatile int lastEncoded1 = 0;
volatile long encoderValue1 = 0;
volatile int lastEncoded2 = 0;
volatile long encoderValue2 = 0;
char input[20];
char txAR[10];    // = "16:41:34#";
char txDEC[11];    // = sprintf(txDEC, "+36d%c28:%02d#", 223, int(dec_m), 0);
long TSL;
unsigned long t_ciclo_acumulado = 0, t_ciclo;
long Az_tel_s, Alt_tel_s;
long AR_tel_s, DEC_tel_s;
long AR_stell_s, DEC_stell_s;
double cos_phi, sin_phi;
double alt, azi;

const byte numChars = 32;
char receivedChars[numChars];

boolean isHomed = false;
boolean isPositiveDeclination = false;

void initConversion() {
	sprintf(txAR, "%02d:%02d:%02d#", int(ra_h), int(ra_m),
			int(0));
	sprintf(txDEC, "%c%02d%c%02d:%02d#", dec_d > 0 ? '+' : '-', int(dec_d), 223,
			int(dec_m), int(0));
	/*cos_phi = cos(
			(((latHH * 3600) + (latMM * 60) + latSS) / 3600.0) * pi / 180.0);
	sin_phi = sin(
			(((latHH * 3600) + (latMM * 60) + latSS) / 3600.0) * pi / 180.0);

	TSL = poleAR_HH * 3600 + poleAR_MM * 60 + poleAR_SS + poleH_HH * 3600
			+ poleH_MM * 60 + poleH_SS;
	while (TSL >= 86400)
	 TSL = TSL - 86400;*/
}

void AZ_to_EQ() {
	double delta_tel, sin_h, cos_h, sin_A, cos_A, sin_DEC, cos_DEC;
	double H_telRAD, h_telRAD, A_telRAD;
	long H_tel;
	long arHH, arMM, arSS;
	long decDEG, decMM, decSS;
	char sDEC_tel;

	float percent_day = ra_deg / 360;

	arHH = percent_day * 24;
	arMM = (percent_day * 24 - arHH) * 60;
	arSS = ((percent_day * 24 - arHH) * 60 - arMM) * 60;

	decDEG = dec_deg;
	decMM = (dec_deg - decDEG) * 60;
	decSS = ((dec_deg - decDEG) * 60 - decMM) * 60;
	sDEC_tel = isPositiveDeclination ? 43 : 45; // 45 IS NEGATIVE; also try sDEC_tel = 43;

	//sprintf(txAR, "%02d:%02d:%02d#", int(arHH), int(arMM), int(arSS));
	//sprintf(txDEC, "%c%02d%c%02d:%02d#", sDEC_tel, int(decDEG), 223, int(decMM),
	//		int(decSS));

#if defined DEBUG && defined DEBUG_SERIAL

	Serial.println(dec_deg);
	Serial.println(decDEG);
	Serial.println(decMM);
	Serial.println(decSS);

	Serial.println(sDEC_tel);

	Serial.println("DONE");
	Serial.println(txAR);
	Serial.println(txDEC);
#endif
}

bool newData = false; // Gets set to true whenever a complete command is buffered
static boolean recvInProgress = false; // True while a command is being received
static byte ndx = 0; // Number of command character received
const char startMarker = ':'; // Commands begin with this character
const char endMarker = '#'; // Commands end with this character


void receiveCommandChar() {

	char rc;

	if (Serial.available() > 0 && newData == false) {
		rc = Serial.read();

		if (recvInProgress == true) {
			if (rc != endMarker) {
				receivedChars[ndx] = rc;
				ndx++;
				if (ndx >= numChars) {
					ndx = numChars - 1;
				}
			} else {
				receivedChars[ndx] = '\0'; // terminate the string
				recvInProgress = false;
				ndx = 0;
				newData = true;
			}
		} else if (rc == startMarker) {
			recvInProgress = true;
		}
	}
}

float last_ra_deg = ra_deg;
float last_dec_deg = dec_deg;

long positions[2];

/**
 * This gets called whenever
 */
bool parseCommands(MultiStepper &motors, bool homingMode) {
	if (newData == true) {
		if (receivedChars[0] == 'G' && receivedChars[1] == 'R') {
			Serial.print(txAR);
		} else if (receivedChars[0] == 'G' && receivedChars[1] == 'D') {
			Serial.print(txDEC);
		} else if (receivedChars[0] == 'Q') {
			// Stop moving

		} else if (receivedChars[0] == 'M' && receivedChars[1] == 'S') {
			// Move Start
			Serial.print("0");

			// TODO Homing code needs to be better. It has to disable the steppers and there must be some way to enable/disable it
			// If homing mode is true we set isHomed to true
			// and return true to indicate to the loop() function that homing is complete.
			if (homingMode) {
				isHomed = true;
				newData = false;
				return true;
			} else {
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
			ra_deg = 360.
					* (hrs / 24. + mins / (24. * 60.) + secs / (24. * 3600.));

			// Set the desired ascension stepper motor position
			// TODO this is very likely wrong. We need to convert this to altaz
			//positions[0] = ra_deg / 360 * AZ_STEPS_PER_REV;

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

			dec_deg = multi * (deg + mins / 60. + secs / 3600.);
			isPositiveDeclination = multi > 0;

			// Set the desired altitude stepper motor position
			// TODO this is very likely wrong. We need to convert this to altaz
			//positions[1] = dec_deg / 360 * ALT_STEPS_PER_REV;

			Serial.print("1");
		} else {
			Serial.println("ERROR: Unknown command");
			Serial.println(receivedChars);
		}
		newData = false;
	}
	return false;
}

// Returns true if homing is completed
bool communication(MultiStepper &motors, bool homingMode) {
	receiveCommandChar();
	return parseCommands(motors, homingMode);
}


void read_sensors(AccelStepper &az, AccelStepper &el) {
	long h_deg, h_min, h_seg, A_deg, A_min, A_seg;

	encoderValue1 = az.currentPosition() % AZ_STEPS_PER_REV;
	encoderValue2 = el.currentPosition() % ALT_STEPS_PER_REV;

	int enc1 = encoderValue1 / 1500;
	long encoder1_temp = encoderValue1 - (enc1 * 1500);
	long map1 = enc1 * map(1500, 0, AZ_STEPS_PER_REV, 0, 324000);
	int enc2 = encoderValue2 / 1500;
	long encoder2_temp = encoderValue2 - (enc2 * 1500);
	long map2 = enc2 * map(1500, 0, ALT_STEPS_PER_REV, 0, 1296000);

	Alt_tel_s = map1 + map(encoder1_temp, 0, AZ_STEPS_PER_REV, 0, 324000);
	Az_tel_s = map2 + map(encoder2_temp, 0, ALT_STEPS_PER_REV, 0, 1296000);

	if (Az_tel_s < 0)
		Az_tel_s = 1296000 + Az_tel_s;
	if (Az_tel_s >= 1296000)
		Az_tel_s = Az_tel_s - 1296000;

	/*Serial.print("Values are (az, el): ");
	Serial.print(encoderValue1);
	Serial.print(", ");
	Serial.print(encoderValue2);
	Serial.print("; alt/az: ");
	Serial.print(Alt_tel_s);
	Serial.print(", ");
	 Serial.println(Az_tel_s);*/
}

void loopConversion() {
	t_ciclo = millis();
	if (t_ciclo_acumulado >= seg_sideral) {
		TSL++;
		t_ciclo_acumulado = t_ciclo_acumulado - seg_sideral;
		if (TSL >= 86400) {
			TSL = TSL - 86400;
		}
	}
	t_ciclo = millis() - t_ciclo;
	t_ciclo_acumulado = t_ciclo_acumulado + t_ciclo;
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

// These are variables that are only needed when debug mode is enabled
//#ifdef DEBUG

// These are variables that require debug mode and serial debug mode to be enabled
//#ifdef DEBUG_SERIAL

int pr = -1;
long last_reported_az = 0;
long last_reported_dec = 0;

//#endif // DEBUG_SERIAL
//#endif // DEBUG

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
 */
void EQ_to_AZ(MultiStepper &motors, AccelStepper &az_s, AccelStepper &el_s,
		FuGPS &gps, Position &pos, bool justHomed) {
	// KEEP TIME
	// TODO Month rollover etc
	// This will be handled by GPS eventually, but we may need a better way to prevent bugs during testing

	current_lat = pos.latitude > 1.00 ? pos.latitude : LAT;
	current_lng = pos.longitude > 1.00 ? pos.longitude : LNG;

	long passed_seconds = (millis() * TIME_FACTOR) / 1000; // Seconds that have passed since exceution started

	int current_day = 24;
	int current_hour = pos.hours; //17; //gps.Hours;
	int current_minute = pos.minutes; //47; //gps.Minutes;
	int current_second = pos.seconds; //passed_seconds; //gps.Seconds;

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
		//Serial.print("Just homed to ");
		//Serial.print(desired_az);
		//Serial.print(" / ");
		//Serial.println(desired_alt);

		az_s.setCurrentPosition(desired_az);
		el_s.setCurrentPosition(desired_alt);

		last_desired_az = desired_az;
		last_desired_dec = desired_alt;
		isHomed = true;
	} else {
		az_s.setCurrentPosition(0);
		el_s.setCurrentPosition(0);
		az_s.runToNewPosition(last_desired_az - desired_az);
		el_s.runToNewPosition(last_desired_dec - desired_alt);


	// From here on only debug outputs happen
#if defined DEBUG && defined DEBUG_SERIAL
		//if (pr == -1 || pr >= 10) {
		Serial.print(
				gps.hasFix() ?
						"GPS: " + String(gps.Satellites, 6) + "S/"
								+ String(gps.Quality) + "Q" :
						"GPS: N/A");
		Serial.print("; LAT ");
		Serial.print(current_lat);
		Serial.print(", LNG ");
		Serial.print(current_lng);
		Serial.print("; RA ");
		Serial.print(ra_deg);
		Serial.print(" and DEC ");
		Serial.print(dec_deg);
		Serial.print(" to ALTAZ is: ALT ");
		Serial.print(deg_altitude);
		Serial.print("ฐ AZ ");
		Serial.print(deg_azimuth);
		Serial.print("ฐ HA ");
		Serial.print(rad_hour_angle);
		Serial.print("ฐ sin(HA) ");
		Serial.print(sin(rad_hour_angle));
		Serial.print("ANG; The time is: ");
		Serial.print(current_hour);
		Serial.print(":");
		Serial.print(current_minute);
		Serial.print(":");
		Serial.print(current_second);
		Serial.print("Steppers: az");
		Serial.print(desired_az);
		Serial.print("/dec ");
		Serial.print(desired_alt);
		Serial.print(" diff ");
		Serial.print(last_desired_az - desired_az);
		Serial.print(" / ");
		Serial.print(last_desired_dec - desired_alt);
		Serial.print("ฐ; Reported: az");
		Serial.print(az_s.currentPosition());
		Serial.print("/dec ");
		Serial.print(el_s.currentPosition());

		last_desired_az = desired_az;
		last_desired_dec = desired_alt;
		//	pr = 0;
		//}
		//pr++;
#endif
	}
}
//#endif

#endif
