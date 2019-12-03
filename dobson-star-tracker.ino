#include <TimeLib.h>
#include <AccelStepper.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <MultiStepper.h>

#include "config.h"
#include "conversion.h"
//#include "location.h"

//Load the timer library, depending on the selected BOARD_TYPE
#ifdef BOARD_ARDUINO_MEGA
	#include <TimerOne.h>
#elif defined BOARD_ARDUINO_DUE
	#include <DueTimer.h>
#endif

// Load the display unit if it's enabled
#ifdef SERIAL_DISPLAY_ENABLED
#include "display_unit.h"
#endif

// Load the required observer class
#ifdef GPS_FIXED_POS
	#include "./FixedObserver.h"
#else
	#include "./GpsObserver.h"
#endif

// Include the mount, depending on the selected MOUNT_TYPE
#ifdef MOUNT_TYPE_DOBSON
	#include "./Dobson.h"
#elif defined MOUNT_TYPE_EQUATORIAL
	#include "./Equatorial.h" // NOT IMPLEMENTED!
#elif defined MOUNT_TYPE_DIRECT
	#include "./DirectDrive.h"
#endif


// Initialize the Steppers
AccelStepper azimuth(AccelStepper::DRIVER, AZ_STEP_PIN, AZ_DIR_PIN);    // Azimuth stepper
AccelStepper altitude(AccelStepper::DRIVER, ALT_STEP_PIN, ALT_DIR_PIN); // Altitude stepper

// Initialize the Observer (either fixed or GPS)
#ifdef GPS_FIXED_POS
	FixedObserver observer(ALT, LAT, LNG, INITIAL_YEAR, INITIAL_MONTH, INITIAL_DAY, INITIAL_HOUR, INITIAL_MINUTE, INITIAL_SECOND);
#else
	GpsObserver observer;
#endif

// Initialize the Mount
#ifdef MOUNT_TYPE_DOBSON
	Dobson scope(azimuth, altitude, observer);
#elif defined MOUNT_TYPE_EQUATORIAL
	Equatorial scope(azimuth, altitude, observer); // NOT IMPLEMENTED!
#elif defined MOUNT_TYPE_DIRECT
	DirectDrive scope(azimuth, altitude, observer);
#endif

// Are the stepper drivers currently enabled?
bool motorsEnabled = false;

// This increases every loop iteration and resets at 10.000
unsigned int loopIteration = 0;

// Is the loop() function being run for the first time?
bool first_loop_run = true;

// millis() at the time of the latest call to Mount::calculateMotorTargets()
long last_motor_update = 0;

#ifdef DEBUG_HOME_IMMEDIATELY
	const bool homeImmediately = true;
#else
	const bool homeImmediately = false;
#endif


/**
 * Motor Interrupt handler
 * This is attached to timer interrupt 1. It gets called every STEPPER_INTERRUPT_FREQ / 1.000.000 seconds and moves our steppers
 */
void moveSteppers() {
	azimuth.run();
	altitude.run();
}


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

	// Setup the stepper interrupt. Depends on the selected board
#ifdef BOARD_ARDUINO_MEGA
	Timer1.initialize(STEPPER_INTERRUPT_FREQ);
	Timer1.attachInterrupt(&moveSteppers);
#elif defined BOARD_ARDUINO_DUE
	Timer.getAvailable()
		.attachInterrupt(&moveSteppers)
		.start(STEPPER_INTERRUPT_FREQ);
#endif

	DEBUG_PRINT("  Steppers enabled:  ");
#ifdef AZ_ENABLE
	DEBUG_PRINT("Az=ON");
#else
	DEBUG_PRINT("Az=OFF");
#endif

#ifdef ALT_ENABLE
	DEBUG_PRINTLN("  Alt=ON");
#else
	DEBUG_PRINTLN("  Alt=OFF");
#endif
} // setupSteppers


/*
 * This function turns stepper motor drivers on, if these conditions are met:
 * STEPPERS_ON_PIN reads HIGH
 * operating mode is not Mode::INITIALIZING
 * TODO Maybe the conditions need to change (esp. the opmode one)
 */
void setSteppersOnOffState() {
	const bool isTracking = scope.getMode() == Mode::TRACKING;

#ifdef STEPPERS_ON_PIN
	// If the STEPPERS_ON switch is installed, check its state
	const bool steppersSwitchOn = digitalRead(STEPPERS_ON_PIN) == HIGH;
#else
	// If no STEPPERS_ON switch is installed, define its state as enabled
#define steppersSwitchOn true
#endif

	if (steppersSwitchOn && isTracking) {
		// Motors on
		motorsEnabled = true;
#ifdef AZ_ENABLE
		digitalWrite(AZ_ENABLE_PIN, LOW);
#endif
#ifdef ALT_ENABLE
		digitalWrite(ALT_ENABLE_PIN, LOW);
#endif
	}
	else {
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

#ifndef REMOVE_SPLASH
	void greet() {
		DEBUG_PRINTLN("                      __          __  _                          ");
		DEBUG_PRINTLN("                      \\ \\        / / | |                         ");
		DEBUG_PRINTLN("                       \\ \\  /\\  / /__| | ___ ___  _ __ ___   ___ ");
		DEBUG_PRINTLN("                        \\ \\/  \\/ / _ \\ |/ __/ _ \\| '_ ` _ \\ / _ \\");
		DEBUG_PRINTLN("                         \\  /\\  /  __/ | (_| (_) | | | | | |  __/");
		DEBUG_PRINTLN("                          \\/  \\/ \\___|_|\\___\\___/|_| |_| |_|\\___|");
		DEBUG_PRINTLN("                                                                 ");

		DEBUG_PRINTLN(" _____        _                          _____ _                _______             _             ");
		DEBUG_PRINTLN("|  __ \\      | |                        / ____| |              |__   __|           | |            ");
		DEBUG_PRINTLN("| |  | | ___ | |__  ___  ___  _ __     | (___ | |_ __ _ _ __      | |_ __ __ _  ___| | _____ _ __ ");
		DEBUG_PRINTLN("| |  | |/ _ \\| '_ \\/ __|/ _ \\| '_ \\     \\___ \\| __/ _` | '__|     | | '__/ _` |/ __| |/ / _ \\ '__|");
		DEBUG_PRINTLN("| |__| | (_) | |_) \\__ \\ (_) | | | |    ____) | || (_| | |        | | | | (_| | (__|   <  __/ |   ");
		DEBUG_PRINTLN("|_____/ \\___/|_.__/|___/\\___/|_| |_|   |_____/ \\__\\__,_|_|        |_|_|  \\__,_|\\___|_|\\_\\___|_|   ");
		DEBUG_PRINTLN("                                                                                               ");

		DEBUG_PRINTLN("Welcome to the dobson-star-tracker serial console.");
		DEBUG_PRINTLN("Before continuing, please set up the config.h file.");
		DEBUG_PRINTLN("Type :HLP# and press Enter to see a list of available commands");
		DEBUG_PRINTLN();
		DEBUG_PRINTLN();
		DEBUG_PRINTLN();
	}
#endif


/**
 * Performs a few sanity checks on the config. If there are any obvious errors,
 * warn the user. Depending on what is set in config.h this may enter an
 * endless loop to prevent damage to the telescope hardware. Have a look at
 * DEBUG_STOP_ON_CONFIG_INSANITY in config.h
 */
void config_sanity_check() {
	bool failed = false;
	bool can_continue = true;

	// Begin checks
	if (AZ_STEPS_PER_REV <= 0) {
		DEBUG_PRINTLN("  Warning: AZ_STEPS_PER_REV should probably be > 0");
		failed = true;
	}
	if (ALT_STEPS_PER_REV <= 0) {
		DEBUG_PRINTLN("  Warning: ALT_STEPS_PER_REV should probably be > 0");
		failed = true;
	}

	#if defined BUZZER_PIN
		#if BUZZER_PIN == STEPPERS_ON_PIN
			DEBUG_PRINTLN("  Error: BUZZER_PIN and STEPPERS_ON_PIN are set to the same pin number.");
			failed = true; can_continue = false;
		#endif
		#if BUZZER_PIN == HOME_NOW_PIN
			DEBUG_PRINTLN("  Error: BUZZER_PIN and HOME_NOW_PIN are set to the same pin number.");
			failed = true; can_continue = false;
		#endif
		#if BUZZER_PIN == TARGET_SELECT_PIN
			DEBUG_PRINTLN("  Error: BUZZER_PIN and TARGET_SELECT_PIN are set to the same pin number.");
			failed = true; can_continue = false;
		#endif
	#endif

	#if defined STEPPERS_ON_PIN
		#if STEPPERS_ON_PIN == HOME_NOW_PIN
			DEBUG_PRINTLN("  Error: STEPPERS_ON_PIN and HOME_NOW_PIN are set to the same pin number.");
			failed = true; can_continue = false;
		#endif
		#if STEPPERS_ON_PIN == TARGET_SELECT_PIN
			DEBUG_PRINTLN("  Error: STEPPERS_ON_PIN and TARGET_SELECT_PIN are set to the same pin number.");
			failed = true; can_continue = false;
		#endif
	#endif

	#if defined HOME_NOW_PIN && HOME_NOW_PIN == TARGET_SELECT_PIN
		DEBUG_PRINTLN("  Error: HOME_NOW_PIN and TARGET_SELECT_PIN are set to the same pin number.");
		failed = true; can_continue = false;
	#endif

	#ifndef GPS_FIXED_POS
		#if GPS_MIN_SATELLITES <= 0
			DEBUG_PRINTLN("  Warning: GPS_MIN_SATELLITES should probably be > 0");
			failed = true;
		#endif

		#if GPS_MIN_SATELLITES_TIME <= 0
			DEBUG_PRINTLN("  Warning: GPS_MIN_SATELLITES_TIME should probably be > 0");
			failed = true;
		#endif
	#endif

	#if UPDATE_MOTOR_POS_MS <= 0
		DEBUG_PRINTLN("  Warning: UPDATE_MOTOR_POS_MS should probably be > 0");
		failed = true;
	#endif
		

	#if STEPPER_INTERRUPT_FREQ <= 0
		DEBUG_PRINTLN("  Warning: STEPPER_INTERRUPT_FREQ should probably be > 0");
		failed = true;
	#endif

	// End checks

	if (failed) {
		DEBUG_PRINTLN("  Config sanity check has failed.");
		DEBUG_PRINTLN("  Check the output above for errors and review config.h");
		DEBUG_PRINTLN("  Do not continue until you know why this message appeared!");
		#ifdef DEBUG_STOP_ON_CONFIG_INSANITY
			bool stop = true;
			while (stop) {
				// Buzzer is installed. Start buzzing and continue endless loop
				#ifdef BUZZER_PIN
					digitalWrite(BUZZER_PIN, HIGH);
				#endif

				// DEBUG_SERIAL is enabled. Wait for "ok" input
				#ifdef DEBUG_SERIAL
					DEBUG_PRINTLN("  Press 1 and Enter to continue");
					while (!Serial.available()) {}
					if (Serial.read() == '1') {
						if (can_continue) {
							stop = false;
						}
						else {
							DEBUG_PRINTLN("  I'm sorry Dave, I'm afraid I can't do that. Please fix the errors first");
						}
					}
				#endif
			}
			// Buzzer is installed. Stop buzzing after the continue command
			#ifdef BUZZER_PIN
				digitalWrite(BUZZER_PIN, LOW);
			#endif
		#endif
	}
}

/**
 * Run various tasks required to initialize the following:
 * Serial connection
 * Serial communication
 * Stepper motors
 * If enabled:
 *     Display module
 *     GPS module
 *     Buzzer
 *     Steppers On/Off Button
 *     Home Now Button
 *     Target Select Button
 *
 * Finally, telescope.initialize() is called. This sets an initial target (depending on the MOUNT_TYPE) and the initial scope operating mode
 */
void setup() {
	Serial.begin(SERIAL_BAUDRATE);
	#if defined DEBUG && defined DEBUG_SERIAL
		delay(500);
		#ifndef REMOVE_SPLASH
			greet();
		#endif
	#endif

	DEBUG_PRINTLN("> Performing config sanity check");
	config_sanity_check();
	
	// If the serial display is enabled, begin communicating with it
	#ifdef SERIAL_DISPLAY_ENABLED
		DEBUG_PRINT("> Initializing Display module ... ");
		initDisplayCommunication(scope);
		DEBUG_PRINTLN("done");
	#else
		DEBUG_PRINTLN("Display module is disabled");
	#endif

	DEBUG_PRINT("> Initializing Observer module ... ");
	// This initializes the Observer (including the GPS module, if required)
	observer.initialize();
	DEBUG_PRINTLN("done");
	#if defined DEBUG && defined DEBUG_SERIAL
		observer.printDebugInfo();
	#endif

	DEBUG_PRINTLN("> Initializing Telescope module ... ");
	// Initialize the telescope
	scope.initialize();

	DEBUG_PRINTLN();
	DEBUG_PRINTLN("> Initializing Serial communications module ... ");
	// This sets up communication with Stellarium / Serial console
	initCommunication(scope);

	DEBUG_PRINTLN();
	DEBUG_PRINTLN("> Initializing Stepper drivers ... ");
	// Initialize stepper motor drivers
	setupSteppers();

	DEBUG_PRINTLN("> Initializing Buttons and Output ... ");
	DEBUG_PRINT("    Buzzer...............");
	// Button pins
	#ifdef BUZZER_PIN
		pinMode(BUZZER_PIN, OUTPUT);
		digitalWrite(BUZZER_PIN, HIGH);
		//delay(1000);
		digitalWrite(BUZZER_PIN, LOW);
		DEBUG_PRINLN("done");
	#else
		DEBUG_PRINTLN("not connected");
	#endif

	DEBUG_PRINT("    Steppers on/off......");
	// Steppers on/off switch
	#ifdef STEPPERS_ON_PIN
		pinMode(STEPPERS_ON_PIN, INPUT);
		DEBUG_PRINLN("done");
	#else
		DEBUG_PRINTLN("not connected");
	#endif


	DEBUG_PRINT("    Home now.............");
	// Home now switch
	#ifdef HOME_NOW_PIN
		pinMode(HOME_NOW_PIN, INPUT);

		DEBUG_PRINLN("done");
	#else
		DEBUG_PRINTLN("not connected");
	#endif


	DEBUG_PRINT("    Debug target.........");
	// Target select pin
	#ifdef TARGET_SELECT_PIN
		// Input mode for the select pin is INPUT with PULLUP enabled, so that we can use a longer cable
		pinMode(TARGET_SELECT_PIN, INPUT_PULLUP);
		DEBUG_PRINLN("done");
	#else
		DEBUG_PRINTLN("not connected");
	#endif

	// If the serial display is enabled, send the telescope status
	#ifdef SERIAL_DISPLAY_ENABLED
		DEBUG_PRINTLN("> Sending status to Display .... ");
		display_statusUpdate(scope);
	#endif

	DEBUG_PRINTLN("> Initialization done");
	DEBUG_PRINTLN();
	DEBUG_PRINTLN();
	DEBUG_PRINTLN();

}


/*
 * Main program loop. It does roughly the following (in order)
 * 1) Check whether the Home Button is pressed. If so, set the scope back to homing mode
 * 2) Get updates from the GPS module and tell the telescope instance about it
 * 3) Handle communication over serial
 * 4) Check whether homing mode should end (TODO This should move to the telescope class)
 * 5) Turn the stepper drivers on/off
 * 6) Every 10.000 loop iterations:
 *    - Run the necessary calculations to update the telescope target position
 *    - Update the target values for the steppers
 * 7) Debug communications, if enabled
*/
void loop() {
	#ifdef HOME_NOW_PIN
		// If the HOME button is installed and pressed/switched on start homing mode
		const bool currentlyAligning = digitalRead(HOME_NOW_PIN) == HIGH;

		if (currentlyAligning) {
			// The telescope stands still and waits for the next target selection (which it then assumes it is pointing at)
			// Setting homed to false also sets the telescope to operating mode Mode::ALIGNING
			scope.setHomed(false);
		}
	#else
		// If no HOME button exists, it never gets pressed, so we define it as false
		#define currentlyAligning false
	#endif

	// Get the current position from our GPS module. If no GPS is installed
	// or no fix is available values from EEPROM / config.h are used.
	// For more details look at the implementations of the Observer class
	observer.updatePosition();

	// Handle serial serial communication. Returns true if homing was just performed
	bool justAligned = handleSerialCommunication(scope, observer);

	#ifdef SERIAL_DISPLAY_ENABLED
		handleDisplayCommunication(scope, observer);
	#endif

	// If DEBUG_HOME_IMMEDIATELY is defined, homing is performed on first loop iteration.
	// Otherwise a serial command or HOME_NOW Button are required
	if (justAligned || currentlyAligning
		|| (homeImmediately && loopIteration == 0)) {
		justAligned = true;

		// Sets the telescope to operating mode Mode::TRACKING
		scope.setHomed(true);
	}

	// Turn the stepper motors on or off, depending on state of STEPPERS_ON_PIN
	setSteppersOnOffState();

	// Every UPDATE_MOTOR_POS_MS milliseconds, or when this is run the first time, 
	// we calculate and update the target motor position of the telescope
	if (millis() - last_motor_update >= UPDATE_MOTOR_POS_MS || first_loop_run) {
		first_loop_run = false;
		last_motor_update = millis();

		#if defined(DEBUG) && defined(DEBUG_SERIAL_STEPPER_MOVEMENT) && defined(DEBUG_TIMING)
			// Start timing the calculation
			long micros_start = micros();
		#endif

		// This function converts the coordinates
		scope.calculateMotorTargets();

		// This actually makes the motors move to their desired target positions
		#ifdef MOUNT_STOP_UNTIL_GPS_POS_VALID
			if (observer.hasValidPosition()) {
				scope.move();
			}
		#else
			scope.move();
		#endif

		#if defined(DEBUG) && defined(DEBUG_SERIAL_STEPPER_MOVEMENT) && defined(DEBUG_TIMING)
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