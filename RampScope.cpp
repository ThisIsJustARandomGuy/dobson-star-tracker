#include <AccelStepper.h>
#include <Arduino.h>
#include <FuGPS.h>
#include <HardwareSerial.h>
#include <MultiStepper.h>

#include "config.h"
#include "conversion.h"
#include "location.h"


AccelStepper azimuth(AccelStepper::DRIVER, AZ_STEP_PIN, AZ_DIR_PIN);
AccelStepper elevation(AccelStepper::DRIVER, ALT_STEP_PIN, ALT_DIR_PIN);

#define OPMODE_INITIALIZING 0
#define OPMODE_HOMING 1
#define OPMODE_TRACKING 2

byte operating_mode = OPMODE_INITIALIZING;

MultiStepper axes;

bool motorsEnabled = false; // True when steppers are enabled

#ifdef DEBUG_HOME_IMMEDIATELY
	const bool homeImmediately = true;
#else
const bool homeImmediately = false;
#endif

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

#ifdef AZ_ENABLE
	DEBUG_PRINTLN("DBG Az  ON");
#else
	DEBUG_PRINTLN("DBG Az  OFF");
#endif

#ifdef ALT_ENABLE
	DEBUG_PRINTLN("DBG Alt ON");
#else
	DEBUG_PRINTLN("DBG Alt OFF");
#endif
} // setupSteppers

// This function turns stepper motor drivers on/off
void handleSteppersOnOff() {
	const bool steppersSwitchOn = digitalRead(STEPPERS_ON_PIN) == HIGH;
	const bool isHoming = operating_mode == OPMODE_HOMING;

	if (steppersSwitchOn && !isHoming) {
		// Motors on
		motorsEnabled = true;
#ifdef AZ_ENABLE
		digitalWrite(AZ_ENABLE_PIN, LOW);
#endif
#ifdef ALT_ENABLE
		digitalWrite(ALT_ENABLE_PIN, LOW);
#endif
	} else {
		// Motors OFF
		motorsEnabled = false;
#ifdef AZ_ENABLE
		digitalWrite(AZ_ENABLE_PIN, HIGH);
#endif
#ifdef ALT_ENABLE
		digitalWrite(ALT_ENABLE_PIN, HIGH);
#endif
	}
}


// TODO Config and wrap in ifdef
FuGPS gps(Serial1); // GPS module

void setup() {
	Serial.begin(115200);

	// This initializes the GPS module
	// TODO Wrap in ifdef
	initGPS (gps);

	// This sets up communication and conversion values
	initCommunication();


	setupSteppers();

	pinMode(STEPPERS_ON_PIN, INPUT);

	// Set the telescope to homing mode (see README for what it does)
	operating_mode = OPMODE_HOMING;
}



unsigned int calc = 0;
void loop() {
	// Get the current position from our GPS module. If no GPS is installed
	// or no fix is available values from EEPROM are used
	Position pos = handleGPS(gps);

	bool requiresHoming = operating_mode == OPMODE_HOMING;
	// Handle serial serial communication. Returns true if homing was just performed
	bool justHomed = communication(axes, requiresHoming);

	// If DEBUG_HOME_IMMEDIATELY is defined, homing is performed on first loop iteration.
	// Otherwise a serial command or BUTTON_HOME are required
	if (justHomed || (homeImmediately && calc == 0)) {
		DEBUG_PRINTLN("Set home");

		justHomed = true;
		operating_mode = OPMODE_TRACKING;
	}

	// Turn the stepper motors on or off, depending on state of STEPPERS_ON_PIN
	handleSteppersOnOff();

	// Every 10.000 loop iterations we handle motor movements. This should be dynamic. based on how long calculations/serial comms took
	if (calc >= 10000 || calc == 0) {
		calc = 0;

#if defined DEBUG && defined DEBUG_SERIAL
		long micros_start = micros();
#endif
		// This function converts the coordinates and sends motor move commands
		bool didMove = EQ_to_AZ(axes, azimuth, elevation, gps, pos, justHomed);

#if defined DEBUG
		if (didMove) {
			long calc_time = micros() - micros_start;
			DEBUG_PRINT("; Move took ");
			DEBUG_PRINT(calc_time / 1000.);
			DEBUG_PRINTLN("ms");
		}
#endif
	}

	calc++;
}
