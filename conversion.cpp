/*
 * Conversion.cpp
 *
 *  Created on: 03.08.2019
 *      Author: lukas
 */
#ifndef CONVERSION_H_

#include <Arduino.h>
#include <AccelStepper.h>
#include "./conversion.h"

int latHH = 47;    // this means 40º North
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

unsigned long seg_sideral = 1003;
const double pi = 3.14159265358979324;
volatile int lastEncoded1 = 0;
volatile long encoderValue1 = 0;
volatile int lastEncoded2 = 0;
volatile long encoderValue2 = 0;
char input[20];
char txAR[10];
char txDEC[11];
long TSL;
unsigned long t_ciclo_acumulado = 0, t_ciclo;
long Az_tel_s, Alt_tel_s;
long AR_tel_s, DEC_tel_s;
long AR_stell_s, DEC_stell_s;
double cos_phi, sin_phi;
double alt, azi;

long pulses_enc1 = 3200;
long pulses_enc2 = 3200;

const byte numChars = 32;
char receivedChars[numChars];

boolean newData = false;
boolean isHomed = false;

long desiredRaSecs, desiredDecSecs;

void initConversion() {
	cos_phi = cos(
			(((latHH * 3600) + (latMM * 60) + latSS) / 3600.0) * pi / 180.0);
	sin_phi = sin(
			(((latHH * 3600) + (latMM * 60) + latSS) / 3600.0) * pi / 180.0);

	TSL = poleAR_HH * 3600 + poleAR_MM * 60 + poleAR_SS + poleH_HH * 3600
			+ poleH_MM * 60 + poleH_SS;
	while (TSL >= 86400)
		TSL = TSL - 86400;
}

void AZ_to_EQ(AccelStepper &az, AccelStepper &el) {
	double delta_tel, sin_h, cos_h, sin_A, cos_A, sin_DEC, cos_DEC;
	double H_telRAD, h_telRAD, A_telRAD;
	long H_tel;
	long arHH, arMM, arSS;
	long decDEG, decMM, decSS;
	char sDEC_tel;

	A_telRAD = (Az_tel_s / 3600.0) * pi / 180.0;
	h_telRAD = (Alt_tel_s / 3600.0) * pi / 180.0;
	sin_h = sin(h_telRAD);
	cos_h = cos(h_telRAD);
	sin_A = sin(A_telRAD);
	cos_A = cos(A_telRAD);
	delta_tel = asin((sin_phi * sin_h) + (cos_phi * cos_h * cos_A));
	sin_DEC = sin(delta_tel);
	cos_DEC = cos(delta_tel);
	DEC_tel_s = long((delta_tel * 180.0 / pi) * 3600.0);

	while (DEC_tel_s >= 324000) {
		DEC_tel_s = DEC_tel_s - 324000;
	}
	while (DEC_tel_s <= -324000) {
		DEC_tel_s = DEC_tel_s + 324000;
	}

	H_telRAD = acos((sin_h - (sin_phi * sin_DEC)) / (cos_phi * cos_DEC));
	H_tel = long((H_telRAD * 180.0 / pi) * 240.0);

	if (sin_A >= 0) {
		H_tel = 86400 - H_tel;
	}
	AR_tel_s = TSL - H_tel;

	while (AR_tel_s < 0) {
		AR_tel_s = AR_tel_s + 86400;
	}
	while (AR_tel_s >= 86400) {
		AR_tel_s = AR_tel_s - 86400;
	}

	if (!isHomed) {
		az.setCurrentPosition(map(AR_tel_s, 0, 86400, 0, 3200));
		el.setCurrentPosition(map(DEC_tel_s, 0, 324000, 0, 3200));
		isHomed = true;
	}

	arHH = AR_tel_s / 3600;
	arMM = (AR_tel_s - arHH * 3600) / 60;
	arSS = (AR_tel_s - arHH * 3600) - arMM * 60;
	decDEG = abs(DEC_tel_s) / 3600;
	decMM = (abs(DEC_tel_s) - decDEG * 3600) / 60;
	decSS = (abs(DEC_tel_s) - decDEG * 3600) - decMM * 60;
	(DEC_tel_s < 0) ? sDEC_tel = 45 : sDEC_tel = 43;

	sprintf(txAR, "%02d:%02d:%02d#", int(arHH), int(arMM), int(arSS));
	sprintf(txDEC, "%c%02d%c%02d:%02d#", sDEC_tel, int(decDEG), 223, int(decMM),
			int(decSS));

	const float stepsPerSec = (3200 / (24 * 60 * 60));
	const long azPosition = arSS * stepsPerSec + (arMM * stepsPerSec * 60)
			+ (arHH * stepsPerSec * 3600);
	const long elPosition = decDEG * stepsPerSec + (decMM * stepsPerSec * 60)
			+ (decSS * stepsPerSec * 3600);

}

void recvWithStartEndMarkers() {
	static boolean recvInProgress = false;
	static byte ndx = 0;
	char startMarker = ':';
	char endMarker = '#';
	char rc;

	while (Serial.available() > 0 && newData == false) {
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
		}

		else if (rc == startMarker) {
			recvInProgress = true;
		}
	}
}

long desiredAz = 0;
long desiredDec = 0;

void showNewData(AccelStepper &az, AccelStepper &el) {
	if (newData == true) {
		if (receivedChars[0] == 'G' && receivedChars[1] == 'R') {
			Serial.print(txAR);
		}
		if (receivedChars[0] == 'G' && receivedChars[1] == 'D') {
			Serial.print(txDEC);
		}
		if (receivedChars[0] == 'Q') {
			// Stop moving
			az.moveTo(az.currentPosition());
			el.moveTo(el.currentPosition());
		}
		if (receivedChars[0] == 'M' && receivedChars[1] == 'S') {
			// Start moving
			az.moveTo(map(desiredRaSecs, 0, 86400, 0, 3200));
				/*map(abs(abs(desiredRaSecs) - abs(AR_tel_s)), 0, 86400, 0,
				 3200));*/
			el.moveTo(map(desiredDecSecs, -324000, 324000, 0, 32000));
						/*map(abs(abs(desiredDecSecs) - abs(DEC_tel_s)), 0, 324000, 0,
						 3200));*/
			Serial.print("0");
			Serial.println("Moving from: ");
			Serial.println(AR_tel_s);
			Serial.println(DEC_tel_s);
			Serial.println("Moving to: ");
			Serial.println(desiredRaSecs);
			Serial.println(desiredDecSecs);
		}
		if (receivedChars[0] == 'S') {
			if (receivedChars[1] == 'r') {
				// Set target RA

				long hrs = (receivedChars[3] - '0') * 10
						+ (receivedChars[4] - '0');
				long mins = (receivedChars[6] - '0') * 10
						+ (receivedChars[7] - '0');
				long secs = (receivedChars[9] - '0') * 10
						+ (receivedChars[10] - '0');

				desiredRaSecs = hrs * 3600 + mins * 60 + secs;

				float raDeg = 360 * ((desiredRaSecs / 86400));

				desiredAz = map(raDeg, 0, 86400, 0, 3200);
				Serial.print("1");
			}
			if (receivedChars[1] == 'd') {
				// Set target s

				long multi = (receivedChars[3] == '+') ? 1 : -1;

				long deg = multi
						* ((receivedChars[4] - '0') * 10
								+ (receivedChars[5] - '0'));

				long mins = (receivedChars[7] - '0') * 10
						+ (receivedChars[8] - '0');
				long secs = (receivedChars[10] - '0') * 10
						+ (receivedChars[11] - '0');

				/*
				 * 	decDEG = abs(DEC_tel_s) / 3600;
				 *	decMM = (abs(DEC_tel_s) - decDEG * 3600) / 60;
				 *
				 *
				 *
				 *	decSS = (abs(DEC_tel_s) - decDEG * 3600) - decMM * 60;
				 *	decSS + decMM * 60 = DEC_tel_s - decDEG * 3600
				 *	decSS + decMM * 60 + decDEG * 3600 = DEC_tel_s
				 *
				 */

				desiredDecSecs = deg * 3600 + mins * 60 + secs;

				float decDeg = 360 * ((abs(desiredRaSecs) / 86400));

				desiredDec = map(decDeg, 0, 360, 0, 3200);
				Serial.print("1");
			}
		}
		newData = false;
	}
}

void communication(AccelStepper &az, AccelStepper &el) {

	recvWithStartEndMarkers();
	showNewData(az, el);
	/*int i = 0;
	input[i++] = Serial.read();
	delay(5);
	while ((input[i++] = Serial.read()) != '#') {
		delay(5);
	}
	input[i] = '\0';

	if (input[1] == ':' && input[2] == 'G' && input[3] == 'R'
			&& input[4] == '#') {
		Serial.print(txAR);
	}

	if (input[1] == ':' && input[2] == 'G' && input[3] == 'D'
			&& input[4] == '#') {
		Serial.print(txDEC);
	 }*/
}
void read_sensors(AccelStepper &az, AccelStepper &el) {
	long h_deg, h_min, h_seg, A_deg, A_min, A_seg;

	/*Serial.print("Values are (az, el): ");
	Serial.print(encoderValue1);
	Serial.print(", ");
	 Serial.println(encoderValue2);*/
	encoderValue1 = az.currentPosition() % pulses_enc1;
	encoderValue2 = el.currentPosition() % pulses_enc2;

	int enc1 = encoderValue1 / 1500;
	long encoder1_temp = encoderValue1 - (enc1 * 1500);
	long map1 = enc1 * map(1500, 0, pulses_enc1, 0, 324000);
	int enc2 = encoderValue2 / 1500;
	long encoder2_temp = encoderValue2 - (enc2 * 1500);
	long map2 = enc2 * map(1500, 0, pulses_enc2, 0, 1296000);

	Alt_tel_s = map1 + map(encoder1_temp, 0, pulses_enc1, 0, 324000);
	Az_tel_s = map2 + map(encoder2_temp, 0, pulses_enc2, 0, 1296000);

	if (Az_tel_s < 0)
		Az_tel_s = 1296000 + Az_tel_s;
	if (Az_tel_s >= 1296000)
		Az_tel_s = Az_tel_s - 1296000;
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
const long timeLast;
const long LAT = 47.426430;
const long LNG = 12.849180;
const short TIME_FACTOR = 1000;
float start_lat = 52.5;
float start_lng = -1.91666667;

float current_jul_magic_year = -731.5; // 1998 //6938.5;// 2019
float current_jul_magic_mo = 212; // August

const int H_TIMEZONE_CORRECTION = 0;

float deg2rad(float degs) {
	return degs * pi / 180;
}

float rad2deg(float rad) {
	return rad * 180 / pi;
}

volatile float rlyaz = 0;
volatile float rlydec = 0;

int pr = -1;
long last_desired_az = 0;
long last_desired_dec = 0;

void EQ_to_AZ(float ra, float dec, AccelStepper &az_s, AccelStepper &el_s) {
	long current_year = 1998;
	long current_month = 8;
	long current_day = 10;

	// KEEP TIME
	long passed_s = ((millis() * TIME_FACTOR) / 1000);
	long current_h = 23;
	long current_m = 10;
	long current_s = 00 + passed_s;
	//Serial.println(current_s);

	while (current_s >= 60) {
		current_m++;
		current_s -= 60;
	}
	while (current_m >= 60) {
		current_h++;
		current_m -= 60;
	}
	/*float jul_day = 2458699.8513 + passed_s / 86400.0;
	 long jul_day_12h = 2458699;*/
	float jul_day = (((current_m / 60.0) + (current_h)) / 24.0)
			+ current_jul_magic_mo + current_day + current_jul_magic_year;
	
	while (current_h >= 24) {
		jul_day++;
		current_h -= 24;
	}

	float utc_current = (current_h - H_TIMEZONE_CORRECTION) + (current_m / 60.0)
			+ (current_s / 3600.0);

	while (utc_current < 0) {
		utc_current += 24;
	}
	while (utc_current >= 24) {
		utc_current -= 24;
	}

	// END TIMEKEEPING
	
	float jul_days_s2k = jul_day;

	// number of Julian centuaries since Jan 1, 2000, 12 UT
	float jul_centuaries = jul_days_s2k / 36525; // T

	// local siderial time
	float LST = 100.46 + 0.985647 * jul_days_s2k + start_lng + 15 * utc_current;
	while (LST < 0) {
		LST += 360;
	}

	float hour_angle = LST - ra;
	while (hour_angle < 0) {
		hour_angle += 360.0;
	}

	// Calculate altitude
	float sin_alt = sin(deg2rad(dec)) * sin(deg2rad(start_lat))
			+ cos(deg2rad(dec)) * cos(deg2rad(start_lat))
					* cos(deg2rad(hour_angle));
	float alt = rad2deg(asin(sin_alt));

	// Calculate azimuth
	float cos_a = (sin(deg2rad(dec))
			- sin(deg2rad(alt)) * sin(deg2rad(start_lat)))
			/ (cos(deg2rad(alt)) * cos(deg2rad(start_lat)));
	float a = rad2deg(acos(cos_a));
	
	float az;
	if (sin(deg2rad(hour_angle)) > 0) {
		az = 360 - a;
	} else {
		az = a;
	}


	long desired_az = (long) round((az / 360) * 320000);
	long desired_dec = (long) round((alt / 360) * 3200000);




	if (pr == -1 || pr >= 10) {
		Serial.print("RA ");
		Serial.print(ra);
		Serial.print(" and DEC ");
		Serial.print(dec);
		Serial.print(" to ALTAZ is: ALT ");
		Serial.print(alt);
		Serial.print(" AZ ");
		Serial.print(az);
		Serial.print("; Steppers: az");
		Serial.print(desired_az);
		Serial.print("/dec ");
		Serial.print(desired_dec);
		Serial.print(" diff ");
		Serial.print(last_desired_az - desired_az);
		Serial.print(" / ");
		Serial.println(last_desired_dec - desired_dec);
		pr = 0;
	}
	
	
	if (!isHomed) {
		last_desired_az = desired_az;
		last_desired_dec = desired_dec;
		az_s.setCurrentPosition(desired_az);
		el_s.setCurrentPosition(desired_dec);
		isHomed = true;
	} else {
		az_s.move(last_desired_az - desired_az);
		el_s.move(last_desired_dec - desired_dec);

		last_desired_az = desired_az;
		last_desired_dec = desired_dec;
	}
	pr++;
}
//#endif

#endif
