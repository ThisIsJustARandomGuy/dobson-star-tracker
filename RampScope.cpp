#include <Arduino.h>
#include <AccelStepper.h>
#include <TimerOne.h>
//#include <MultiStepper.h>

#include "./Pins.h"
#include "./Moon.h"

#include "./conversion.h"


AccelStepper azimuth(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper elevation(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);

//MultiStepper axes;

Moon dummyMoon;

float ra_h = 16;
float ra_m = 41.7;
float ra_deg = (ra_h + ra_m / 60) * 15;

float dec_d = 36;
float dec_m = 28;
float dec_deg = dec_d + dec_m / 60;

float start_y = 2019;
float start_mo = 8;
float start_d = 5;

float start_h = 23;
float start_m = 10;
float start_s = 00;


void setup() {
	Serial.begin(9600);

	//initConversion();

	// Enable azimuth
	pinMode(X_ENABLE_PIN, OUTPUT);
	digitalWrite(X_ENABLE_PIN, LOW); // Enable

	// Enable elevation
	pinMode(Y_ENABLE_PIN, OUTPUT);
	digitalWrite(Y_ENABLE_PIN, LOW);

	azimuth.setMaxSpeed(30000);
	azimuth.setAcceleration(500);

	elevation.setMaxSpeed(30000);
	elevation.setAcceleration(500);

	Serial.println(ra_deg);
	Serial.println(dec_deg);
	//Timer1.initialize(100);
	//Timer1.attachInterrupt(stepperMove);

	// Add steppers to mutlistepper
	//axes.addStepper(azimuth);
	//axes.addStepper(elevation);

	/*azimuth.moveTo(12000);
	 elevation.moveTo(6400);*/
}

int calc = -1;

void loop() {

	azimuth.run();
	elevation.run();

//	loopConversion();
//	read_sensors(azimuth, elevation);

	if (calc >= 5000 || calc == -1 || Serial.available() > 0) {
		//azimuth.setCurrentPosition(random(0, 3200));
		//elevation.setCurrentPosition(random(0, 6400));

		//AZ_to_EQ(azimuth, elevation);
		long millis_start = micros();
		EQ_to_AZ(ra_deg, dec_deg, azimuth, elevation);
		long calc_time = micros() - millis_start;
		if (DEBUG == true) {
			Serial.print("Calc took ");
			Serial.print(calc_time / 1000.);
			Serial.println("ms");
		}
		//Serial.println(azimuth.targetPosition());
		
		//Serial.println("done...");
		calc = 0;
	}

	/*if (Serial.available() > 0)
	 communication(azimuth, elevation);*/

	calc++;

	/*long positions[2]; // 0 = azimuth; 1 = elevation

	positions[0] = 5000;
	positions[1] = dummyMoon.getElevation();
	axes.moveTo(positions);
	axes.runSpeedToPosition();
	delay(1000);

	positions[0] = -azimuth.currentPosition();
	positions[1] = -elevation.currentPosition();
	axes.moveTo(positions);
	axes.runSpeedToPosition();
	delay(1000);

	if(azimuth.distanceToGo() == 0 && elevation.distanceToGo() == 0) {
		delay(1000);
	}*/
	// If at the end of travel go to the other end
	/*if (azimuth.distanceToGo() == 0) {
		azimuth.moveTo(-azimuth.currentPosition());
	}

	if (elevation.distanceToGo() == 0) {
		elevation.moveTo(-elevation.currentPosition());
	 }*/
}
