#include <Arduino.h>
#include <AccelStepper.h>
//#include <MultiStepper.h>

#include "./Pins.h"
#include "./Moon.h"

#include "./conversion.h"


AccelStepper azimuth(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper elevation(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);

//MultiStepper axes;

Moon dummyMoon;

void setup() {
	Serial.begin(9600);

	initConversion();

	// Enable azimuth
	pinMode(X_ENABLE_PIN, OUTPUT);
	digitalWrite(X_ENABLE_PIN, LOW); // Enable

	// Enable elevation
	pinMode(Y_ENABLE_PIN, OUTPUT);
	digitalWrite(Y_ENABLE_PIN, LOW);

	azimuth.setMaxSpeed(30000);
	azimuth.setAcceleration(500);

	elevation.setMaxSpeed(20000);
	elevation.setAcceleration(2000);

	// Add steppers to mutlistepper
	//axes.addStepper(azimuth);
	//axes.addStepper(elevation);

	azimuth.moveTo(1200);
	elevation.moveTo(6400);
}

void loop() {
	//loopConversion();

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
	if (azimuth.distanceToGo() == 0) {
		azimuth.moveTo(-azimuth.currentPosition());
	}

	if (elevation.distanceToGo() == 0) {
		elevation.moveTo(-elevation.currentPosition());
	}

	azimuth.run();
	elevation.run();
}
