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
int poleAR_HH = 12;    // this means 2 hours, 52 minutes and 16 seconds
int poleAR_MM = 50;
int poleAR_SS = 53;

// enter Pole Star hour angle (H: HH:MM:SS)
int poleH_HH = 8;
int poleH_MM = 32;
int poleH_SS = 7;

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

long pulses_enc1 = 36000;
long pulses_enc2 = 36000;

const byte numChars = 32;
char receivedChars[numChars];

boolean newData = false;

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

	/*if (DEC_tel_s >= 324000) {
		DEC_tel_s = DEC_tel_s % 324000;
	} else if (DEC_tel_s <= 324000) {
		DEC_tel_s = (DEC_tel_s % 324000) * -1;
	 }*/
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

	if (AR_tel_s >= 86400) {
		AR_tel_s = DEC_tel_s % 86400;
	} else if (DEC_tel_s < 0) {

	}
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
	az.moveTo(azPosition);
	el.moveTo(elPosition);

	/*if (Serial.available()) {
		Serial.print("Moving to az ");
		Serial.print(azPosition);
		Serial.print(", el ");
		Serial.print(elPosition);
	 }*/
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

void showNewData() {
	if (newData == true) {
		if (receivedChars[0] == 'G' && receivedChars[1] == 'R') {
			Serial.print(txAR);
		}
		if (receivedChars[0] == 'G' && receivedChars[1] == 'D') {
			Serial.print(txDEC);
		}
		newData = false;
	}
}

void communication() {

	recvWithStartEndMarkers();
	showNewData();
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
