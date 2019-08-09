#include <Arduino.h>
#include <AccelStepper.h>
#include <TimerOne.h>
//#include <MultiStepper.h>

#include "./config.h"
#include "./Moon.h"

#include "./conversion.h"


AccelStepper azimuth(AccelStepper::DRIVER, AZ_STEP_PIN, AZ_DIR_PIN);
AccelStepper elevation(AccelStepper::DRIVER, ALT_STEP_PIN, ALT_DIR_PIN);

#define OPMODE_INITIALIZING 0
#define OPMODE_HOMING 1
#define OPMODE_TRACKING 2

byte operating_mode = OPMODE_INITIALIZING;

//MultiStepper axes;

Moon dummyMoon;



float start_y = 2019;
float start_mo = 8;
float start_d = 5;

float start_h = 23;
float start_m = 10;
float start_s = 00;



void setupSteppers() {
	// Set stepper pins
	pinMode(AZ_ENABLE_PIN, OUTPUT);  // Azimuth pin
	pinMode(ALT_ENABLE_PIN, OUTPUT); // Altitude pin

	azimuth.setMaxSpeed(AZ_MAX_SPEED);
	azimuth.setAcceleration(AZ_MAX_ACCEL);

	elevation.setMaxSpeed(30000);
	elevation.setAcceleration(500);

	// Check for debug constants and enable the stepper drivers
#ifdef AZ_ENABLE
	digitalWrite(AZ_ENABLE_PIN, LOW); // Enable azimuth stepper
#endif
#ifdef ALT_ENABLE
	digitalWrite(ALT_ENABLE_PIN, LOW); // Enable altitude stepper
#endif

#ifdef DEBUG_SERIAL
#ifdef AZ_ENABLE
	Serial.println("DBG Az  ON");
#else
	Serial.println("DBG Az  OFF");
#endif

#ifdef ALT_ENABLE
	Serial.println("DBG Alt ON");
#else
	Serial.println("DBG Alt OFF");
#endif
#endif
}


void setup() {
	Serial.begin(9600);

	// This sets up communication and conversion values
	initConversion();

	setupSteppers();

	// Set the telescope to homing mode (see above for what it does)
	operating_mode = OPMODE_HOMING;

}


int calc = -1;

void loop() {

	azimuth.run();
	elevation.run();

	//loopConversion();
	//read_sensors(azimuth, elevation);

	if (Serial.available() > 0) {
		if (communication(azimuth, elevation, operating_mode == OPMODE_HOMING))
			operating_mode = OPMODE_TRACKING;
	}

#if defined DEBUG && defined DEBUG_SERIAL
		long millis_start = micros();
#endif
		//AZ_to_EQ();
		EQ_to_AZ(azimuth, elevation);

#if defined DEBUG && defined DEBUG_SERIAL
		long calc_time = micros() - millis_start;
		/*Serial.print("Calc took ");
			Serial.print(calc_time / 1000.);
		 Serial.println("ms");*/
#endif
	//calc = 0;


	//calc++;
}
