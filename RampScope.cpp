#include <AccelStepper.h>
#include <Arduino.h>
#include <TimerOne.h>
#include <FuGPS.h>
#include <HardwareSerial.h>
#include <MultiStepper.h>

#include "config.h"
#include "conversion.h"
#include "location.h"


AccelStepper azimuth(AccelStepper::DRIVER, AZ_STEP_PIN, AZ_DIR_PIN); // Azimuth stepper
AccelStepper altitude(AccelStepper::DRIVER, ALT_STEP_PIN, ALT_DIR_PIN); // Altitude stepper
MultiStepper axes;  // This holds both our axes
FuGPS gps(Serial1); // GPS module

// OPMODE_INITIALIZING
// This is only on when initializing critical hardware such as the stepper drivers.
// When this mode is active, you can not rely on any value reported by the telescope (stepper pos, GPS pos, desired pos etc.)
//
// Motors are on: YES
// Tracking active: NO
// Desired/Reported stepper position updates: YES
#define OPMODE_INITIALIZING 0 // Motors off; Tracking NOT active; Desired/Reported stepper position NOT ACCURATE

// OPMODE_HOMING
// While this mode is on, the telescope stands still and assumes that it is correctly pointing at the desired position
//
// Motors are on: YES
// Tracking active: NO
// Desired/Reported stepper position updates: YES
#define OPMODE_HOMING 1

// OPMODE_TRACKING
// While this mode is on, the telescope tracks the desired position
//
// Motors are on: YES
// Tracking active: YES
// Desired/Reported stepper position updates: YES

#define OPMODE_TRACKING 2 // Motors on; Tracking IS active; Desired/Reported stepper position DOES update


// Start with OPMODE_INITIALIZING
byte operating_mode = OPMODE_INITIALIZING;


// Stores whether stepper drivers are enabled
bool motorsEnabled = false;

// This increases every loop iteration and gets reset at 10.000
unsigned int loopIteration = 0;

#ifdef DEBUG_HOME_IMMEDIATELY
	const bool homeImmediately = true;
#else
const bool homeImmediately = false;
#endif


/**
 * This is attached to timer interrupt 1. It gets called every STEPPER_INTERRUPT_FREQ / 1.000.000 seconds and moves our steppers
 */
void moveSteppers() {
	azimuth.run();
	altitude.run();
} // moveSteppers

/**
 * Initialize stepper drivers. Sets pin inversion config, max speed and acceleration per stepper
 */
void setupSteppers() {
	// Set stepper pins
	pinMode(AZ_ENABLE_PIN, OUTPUT);  // Azimuth pin
	pinMode(ALT_ENABLE_PIN, OUTPUT); // Altitude pin

	azimuth.setPinsInverted(true, false, false);
	azimuth.setMaxSpeed(AZ_MAX_SPEED);
	azimuth.setAcceleration(AZ_MAX_ACCEL);

	altitude.setPinsInverted(true, false, false);
	altitude.setMaxSpeed(ALT_MAX_SPEED);
	altitude.setAcceleration(ALT_MAX_ACCEL);

	axes.addStepper(azimuth);
	axes.addStepper(altitude);

	Timer1.initialize(STEPPER_INTERRUPT_FREQ);
	Timer1.attachInterrupt(&moveSteppers);

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


/*
 * This function turns stepper motor drivers on, if these conditions are met
 * STEPPERS_ON_PIN reads HIGH
 * operating mode is not OPMODE_HOMING
 * TODO Maybe the conditions need to change (esp. the opmode one)
 */
void setSteppersOnOffState() {
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

/**
 * Run various tasks required to initialize
 */
void setup() {
	Serial.begin(9600);

	// This initializes the GPS module
	// TODO Wrap in ifdef
	initGPS (gps);

	// This sets up communication and conversion values
	initCommunication();

	// Initialize stepper motor drivers
	setupSteppers();

	// Button pins
	pinMode(STEPPERS_ON_PIN, INPUT);
	pinMode(HOME_NOW_PIN, INPUT);

	// Set the telescope to homing mode (see README for what it does)
	operating_mode = OPMODE_HOMING;
}


void loop() {
	// If the HOME button is pressed/switched on start homing mode
	const bool currentlyHoming = digitalRead(HOME_NOW_PIN) == HIGH;

	if (currentlyHoming) {
		operating_mode = OPMODE_HOMING;
	}

	// Get the current position from our GPS module. If no GPS is installed
	// or no fix is available values from EEPROM are used
	Position pos = handleGPS(gps);

	// Homing needs to be performed in HOMING mode
	bool requiresHoming = operating_mode == OPMODE_HOMING;

	// Handle serial serial communication. Returns true if homing was just performed
	bool justHomed = handleSerialCommunication(axes, requiresHoming);

	// If DEBUG_HOME_IMMEDIATELY is defined, homing is performed on first loop iteration.
	// Otherwise a serial command or HOME_NOW Button are required
	if (justHomed || currentlyHoming
			|| (homeImmediately && loopIteration == 0)) {
		DEBUG_PRINTLN("Set home position");

		justHomed = true;

		// Start tracking after homing
		operating_mode = OPMODE_TRACKING;
	}

	// Turn the stepper motors on or off, depending on state of STEPPERS_ON_PIN
	setSteppersOnOffState();

	// Every 10.000 loop iterations: Handle motor movements.
	// TODO This should be dynamic, based on how long calculations/serial comms took and/or if a new command is available
	if (loopIteration >= 10000 || loopIteration == 0) {
		loopIteration = 0;

		// Start timing the calculation
		long micros_start = micros();

		// This function converts the coordinates and sends motor move commands
		bool stepper_pos_changed = handleMovement(axes, azimuth, altitude, gps, pos, justHomed);

		// Debug: If a move took place, output how long it took from beginning to end of the calculation
		if (stepper_pos_changed) {
			long calc_time = micros() - micros_start;
			DEBUG_PRINT("; Move took ");
			DEBUG_PRINT(calc_time / 1000.);
			DEBUG_PRINTLN("ms");
		}
	}

	loopIteration++;
}
