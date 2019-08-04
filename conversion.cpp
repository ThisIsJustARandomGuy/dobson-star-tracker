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
boolean beginMove = false;

int desiredRaSecs, desiredDecSecs;

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
			Serial.println("Moving from: ");
			Serial.println(AR_tel_s);
			Serial.println(DEC_tel_s);
			Serial.println("Moving to: ");
			Serial.println(desiredRaSecs);
			Serial.println(desiredDecSecs);
			az.moveTo(desiredRaSecs);
				/*map(abs(abs(desiredRaSecs) - abs(AR_tel_s)), 0, 86400, 0,
				 3200));*/
			el.moveTo(desiredDecSecs);
						/*map(abs(abs(desiredDecSecs) - abs(DEC_tel_s)), 0, 324000, 0,
						 3200));*/
			Serial.print("0");
		}
		if (receivedChars[0] == 'S') {
			if (receivedChars[1] == 'r') {
				// Set target RA

				int hrs = (receivedChars[3] - '0') * 10
						+ (receivedChars[4] - '0');
				int mins = (receivedChars[6] - '0') * 10
						+ (receivedChars[7] - '0');
				int secs = (receivedChars[9] - '0') * 10
						+ (receivedChars[10] - '0');

				desiredRaSecs = hrs * 3600 + mins * 60 + secs;

				float raDeg = 360 * ((desiredRaSecs / 86400));

				desiredAz = map(raDeg, 0, 360, 0, 3200);
				Serial.print("1");
			}
			if (receivedChars[1] == 'd') {
				// Set target s

				int multi = (receivedChars[3] == '+') ? 1 : -1;

				static char degC[2];
				degC[0] = receivedChars[4];
				degC[1] = receivedChars[5];

				int deg = atoi(degC);

				/*int deg = multi
						* ((receivedChars[4] - '0') * 10
				 + (receivedChars[5] - '0'));*/

				int mins = (receivedChars[7] - '0') * 10
						+ (receivedChars[8] - '0');
				int secs = (receivedChars[10] - '0') * 10
						+ (receivedChars[11] - '0');

				Serial.println("Move to: ");
				Serial.println(multi);

				Serial.println(degC);
				Serial.println(mins);
				Serial.println(secs);

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
				Serial.println(desiredDecSecs);

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
	encoderValue1 = az.currentPosition();
	encoderValue2 = el.currentPosition();


	if (encoderValue2 >= pulses_enc2 || encoderValue2 <= -pulses_enc2) {
		encoderValue2 = 0;
	}
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

//#endif

#endif
