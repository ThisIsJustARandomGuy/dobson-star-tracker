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

	elevation.setMaxSpeed(30000);
	elevation.setAcceleration(500);

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

	loopConversion();
	read_sensors(azimuth, elevation);

	if (calc >= 10000 || calc == -1) {
		//azimuth.setCurrentPosition(random(0, 3200));
		//elevation.setCurrentPosition(random(0, 6400));

		AZ_to_EQ(azimuth, elevation);
		//Serial.println(azimuth.targetPosition());
		
		//Serial.println("done...");
		calc = 0;
	}

	if (Serial.available() > 0)
		communication(azimuth, elevation);
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
