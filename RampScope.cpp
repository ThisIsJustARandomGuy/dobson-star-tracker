#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

#include "./config.h"
#include "./location.h"

#include "./conversion.h"


AccelStepper azimuth(AccelStepper::DRIVER, AZ_STEP_PIN, AZ_DIR_PIN);
AccelStepper elevation(AccelStepper::DRIVER, ALT_STEP_PIN, ALT_DIR_PIN);

#define OPMODE_INITIALIZING 0
#define OPMODE_HOMING 1
#define OPMODE_TRACKING 2

byte operating_mode = OPMODE_INITIALIZING;

MultiStepper axes;

//Moon dummyMoon;



void setupSteppers() {
	// Set stepper pins
	pinMode(AZ_ENABLE_PIN, OUTPUT);  // Azimuth pin
	pinMode(ALT_ENABLE_PIN, OUTPUT); // Altitude pin

	azimuth.setPinsInverted(true, false, false);
	azimuth.setMaxSpeed(AZ_MAX_SPEED);
	azimuth.setAcceleration(AZ_MAX_ACCEL);

	elevation.setPinsInverted(true, false, false);
	elevation.setMaxSpeed(ALT_MAX_SPEED);
	elevation.setAcceleration(ALT_MAX_ACCEL);

	axes.addStepper(azimuth);
	axes.addStepper(elevation);

	// Check for debug constants and enable the stepper drivers
#ifdef AZ_ENABLE
	//digitalWrite(AZ_ENABLE_PIN, LOW); // Enable azimuth stepper
#endif
#ifdef ALT_ENABLE
	//digitalWrite(ALT_ENABLE_PIN, LOW); // Enable altitude stepper
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

bool motorsEnabled = false;
void handleSteppersOnOff() {
	if (digitalRead(STEPPERS_ON_PIN) == HIGH) {
		//Serial.println("BTN ON");
		//if (!motorsEnabled) {
		//Serial.println("switching ON");
		//motorsEnabled = true;
			// Motors ON
			digitalWrite(AZ_ENABLE_PIN, LOW);
			digitalWrite(ALT_ENABLE_PIN, LOW);
		//}
	} else {
		//if (motorsEnabled) {
		//Serial.println("switching OFF");
		//motorsEnabled = false;
			// Motors OFF
			digitalWrite(AZ_ENABLE_PIN, HIGH);
			digitalWrite(ALT_ENABLE_PIN, HIGH);
		//}
	}
}


FuGPS gps(Serial1);

void setup() {
	Serial.begin(115200);
	initGPS (gps);
	//return; // TODO remove

	// This sets up communication and conversion values
	initConversion();

	setupSteppers();

	pinMode(STEPPERS_ON_PIN, INPUT);

	// Set the telescope to homing mode (see above for what it does)
	operating_mode = OPMODE_HOMING;
}


unsigned int calc = 0;

void loop() {
	Position pos = handleGPS(gps);
	//return;
	//loopConversion();
	//read_sensors(azimuth, elevation);
	handleSteppersOnOff();
	bool justHomed = communication(axes, operating_mode == OPMODE_HOMING);
	if (justHomed)
		operating_mode = OPMODE_TRACKING;
#ifdef DEBUG_SERIAL
	// In Serial debug mode we always home first thing
	if (calc == 0) {
		Serial.println("Set home");
		justHomed = true;
		operating_mode = OPMODE_TRACKING;
	}
#endif

	//delay(100);
	if (calc >= 10000 || calc == 0) {
#if defined DEBUG && defined DEBUG_SERIAL
		long micros_start = micros();
#endif
		//AZ_to_EQ();
		//delay(1000);
		bool didMove = EQ_to_AZ(axes, azimuth, elevation, gps, pos, justHomed);

#if defined DEBUG && defined DEBUG_SERIAL
		if (didMove) {
		long calc_time = micros() - micros_start;
		Serial.print("; Move took ");
		Serial.print(calc_time / 1000.);
		Serial.println("ms");
		}
#endif
		calc = 0;
	}

	calc++;
}
