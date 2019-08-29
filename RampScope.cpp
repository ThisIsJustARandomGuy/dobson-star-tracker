#include <AccelStepper.h>
#include <Arduino.h>
#include <FuGPS.h>
#include <HardwareSerial.h>
#include <MultiStepper.h>

#include "config.h"
#include "conversion.h"
#include "location.h"
#include "Mounts/Dobson.h"

#ifdef BOARD_ARDUINO_MEGA
#include <TimerOne.h>
#elif defined BOARD_ARDUINO_UNO
#include <DueTimer.h>
#endif


AccelStepper azimuth(AccelStepper::DRIVER, AZ_STEP_PIN, AZ_DIR_PIN); // Azimuth stepper
AccelStepper altitude(AccelStepper::DRIVER, ALT_STEP_PIN, ALT_DIR_PIN); // Altitude stepper
MultiStepper axes;  // This holds both our axes
FuGPS gps(Serial1); // GPS module

Dobson scope(azimuth, altitude, gps);

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

	// Setup the stepper interrupt. Depends on the selected board
#ifdef BOARD_ARDUINO_MEGA
	Timer1.initialize(STEPPER_INTERRUPT_FREQ);
	Timer1.attachInterrupt(&moveSteppers);
#elif defined BOARD_ARDUINO_UNO
	Timer.getAvailable().attachInterrupt(&moveSteppers).start(
			STEPPER_INTERRUPT_FREQ);
#endif

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
	const bool isHoming = operating_mode == OPMODE_HOMING;

#ifdef STEPPERS_ON_PIN
	// If the STEPPERS_ON switch is installed, check its state
	const bool steppersSwitchOn = digitalRead(STEPPERS_ON_PIN) == HIGH;
#else
	// If no STEPPERS_ON switch is installed, we define its state as enabled
#define steppersSwitchOn true
#endif

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
#ifdef BOARD_ARDUINO_MEGA
	Serial.begin(9600);
#elif defined BOARD_ARDUINO_UNO
	Serial.begin(115200);
#endif

	// This initializes the GPS module
	// TODO Wrap in ifdef
	initGPS (gps);

	// This sets up communication and conversion values
	initCommunication();

	// Initialize stepper motor drivers
	setupSteppers();

	// Button pins
#ifdef BUZZER_PIN
	pinMode(BUZZER_PIN, OUTPUT);
	digitalWrite(BUZZER_PIN, HIGH);
	delay(1000);
	digitalWrite(BUZZER_PIN, LOW);
#endif
#ifdef STEPPERS_ON_PIN
	pinMode(STEPPERS_ON_PIN, INPUT);
#endif
#ifdef HOME_NOW_PIN
	pinMode(HOME_NOW_PIN, INPUT);
#endif

	// Set the telescope to homing mode (see README for what it does)
	operating_mode = OPMODE_HOMING;
//	tone(BUZZER_PIN, 2000, 500);

	scope.setTarget( { 16.7, 36.5 });
}


void loop() {
#ifdef HOME_NOW_PIN
	// If the HOME button is installed and pressed/switched on start homing mode
	const bool currentlyHoming = digitalRead(HOME_NOW_PIN) == HIGH;
#else
	// If no HOME button exists, it never gets pressed, so we define it as false
#define currentlyHoming false
#endif

	if (currentlyHoming) {
		operating_mode = OPMODE_HOMING;
	}

	// Get the current position from our GPS module. If no GPS is installed
	// or no fix is available values from EEPROM are used
	TelescopePosition pos = handleGPS(gps);

	scope.updateGpsPosition(pos);

	// Homing needs to be performed in HOMING mode
	bool requiresHoming = operating_mode == OPMODE_HOMING;

	// Handle serial serial communication. Returns true if homing was just performed
	bool justHomed = handleSerialCommunication(scope, gps, requiresHoming);

	// If DEBUG_HOME_IMMEDIATELY is defined, homing is performed on first loop iteration.
	// Otherwise a serial command or HOME_NOW Button are required
	if (justHomed || currentlyHoming
			|| (homeImmediately && loopIteration == 0)) {
		justHomed = true;

		// Start tracking after homing
		operating_mode = OPMODE_TRACKING;

		scope.setHomed();
	}

	// Turn the stepper motors on or off, depending on state of STEPPERS_ON_PIN
	setSteppersOnOffState();

	// Every 10.000 loop iterations: Handle motor movements.
	// TODO This should be dynamic, based on how long calculations/serial comms took and/or if a new command is available
	if (loopIteration >= 10000 || loopIteration == 0) {
		loopIteration = 0;

		// Start timing the calculation
		long micros_start = micros();

		// This function converts the coordinates
		scope.calculateMotorTargets();

		// This actually makes the motors move to their desired target positions
		scope.move();

#ifdef DEBUG
		// Debug: If a move took place, output how long it took from beginning to end of the calculation
		if (scope._didMove) {
			long calc_time = scope._lastCalcMicros - micros_start;
			long dbg_time = micros() - scope._lastCalcMicros;
			DEBUG_PRINT("; Calc: ");
			DEBUG_PRINT(calc_time / 1000.);
			DEBUG_PRINT("ms; DbgComms: ");
			DEBUG_PRINT(dbg_time / 1000.);
			DEBUG_PRINTLN("ms");
		}
#endif
	}

	loopIteration++;
}
