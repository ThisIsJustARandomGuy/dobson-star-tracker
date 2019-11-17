#pragma once

/*
 * Board type
 * Uncomment only one of these, depending on the board you are running the firmware on
 */
#define BOARD_ARDUINO_MEGA
//#define BOARD_ARDUINO_DUE

// Baudrate to use when communicating over serial connection
#define SERIAL_BAUDRATE 9600

/*
 * Mount type
 * The default is "Direct Drive". Please see notes below for why
 * 
 * This setting defines which motion system is used by the telescope. Currently, there are three planned, and two supported mount systems:
 * Dobson, Equatorial (NOT IMPLEMENTED!), Direct Drive
 * You can find more information about Dobson on Equatorial mounts here: https://science.howstuffworks.com/telescope5.htm
 * 
 * Notes on the Direct Drive "mount"
 *    Directly moves the steppers to the provided angles without any coordinate system conversion
 *    Does NOT TRACK
 *    It's useful to see whether the axes are set up correctly (connection, direction and steps per revolution)
 *    Because it doesn't track, it won't get confused by unreliable times/GPS positions
 *    If you're using a Dobson mount, but are exclusively controlling it with a Joystick/IR Remote (NOT IMPLEMENTED) this appears to be the most intuitive control method. Others will work as well, though.
 * 
 * IMPORTANT: Define the same mount type on the display, if you are using it.
 * If you're using the wrong combination, the display unit should show a warning, but still allow you to change the angles.
 * The telescope will, at the least, not be able to point to the correct location if you select the wrong mount type on either the display unit or here.
 */
#define MOUNT_TYPE_DOBSON     // Target should be in Ra/Dec. Supports tracking
//#define MOUNT_TYPE_EQUATORIAL // NOT IMPLEMENTED! Target should be in Ra/Dec. Supports tracking
//#define MOUNT_TYPE_DIRECT       // Target should be in degrees for both axis1 and axis2. Does NOT support tracking

/*
 * Azimuth Stepper
 * Direct Drive: Axis 1
 * Dobson: Azimuth (horizontal) axis
 * Equatorial: Not implemented; Right ascension
 */
#define AZ_ENABLE_PIN       38     // RAMPS 1.4 X stepper
#define AZ_STEP_PIN         54     // RAMPS 1.4
#define AZ_DIR_PIN          55     // RAMPS 1.4
#define AZ_STEPS_PER_REV   119467.  // How many steps the stepper motor needs to complete for one a horizontal 360degree revolution of the telescope (my setup: 3200 : 1 and 560 : 15)
#define AZ_MAX_ACCEL       300     // Maximum acceleration for the azimuth stepper
#define AZ_MAX_SPEED       4000    // Maximum speed for the azimuth stepper

/*
 * Altitude stepper
 * Direct Drive: Axis 2
 * Dobson: Altitude (vertical) axis
 * Equatorial: Not implemented; Declination axis
 */
#define ALT_ENABLE_PIN       56    // RAMPS 1.4 Y stepper
#define ALT_STEP_PIN         60    // RAMPS 1.4
#define ALT_DIR_PIN          61    // RAMPS 1.4
#define ALT_STEPS_PER_REV    73920. // for 90degrees //147840 for a full rotation // How many steps the stepper motor needs to complete for a vertical 360degree revolution of the telescope (my setup: 5.18:1 and 3200 : 1 and 105 : 12)
#define ALT_MAX_ACCEL        400   // Maximum acceleration for the altitude stepper
#define ALT_MAX_SPEED        10000 // Maximum speed for the altitude stepper

#define AZ_STEPS_PER_DEG   (AZ_STEPS_PER_REV / 360.0)
#define ALT_STEPS_PER_DEG  (ALT_STEPS_PER_REV / 360.0)

/*
 * Control via Display
 * The DirectDrive telescope can be controlled via an Arduino+Display unit connected via serial port.
 * Eventually, I hope to get the Dobson one working as well
 */
#define SERIAL_DISPLAY_ENABLED              // Uncomment this line to enable/disable control via the display unit
#define SERIAL_DISPLAY_PORT Serial3			// Which Serial port to use. Serial 2 is 16(RX) and 17(TX) on the Mega/Due or 
#define SERIAL_DISPLAY_BAUDRATE       9600  // The baudrate which is used to communicate with the display unit. On the display unit SERIAL_BAUDRATE must have the same value

/*
 * Other Pins
 * Attention: These only apply to my setup. Best to disable all of them and add them back one by one.
 *
 * Comments for myself:
 * y+ is dead
 * y- buzzer
 * x+ stepper on
 * x- pushbutton
 */
//#define BUZZER_PIN      14  // Uncomment if you have a buzzer installed. Default pin for RAMPS 1.4 is Y_MIN
//#define STEPPERS_ON_PIN    3  // Uncomment if you have a stepper switch installed. Default pin for RAMPS 1.4 is  X_MIN. If this pin is HIGH, the steppers are turned on.
//#define HOME_NOW_PIN     3  // Uncomment if you have a homing mode switch installed. Default pin for RAMPS 1.4 is X_MAX. If this pin is HIGH, Telescope::setHomed(true) is called. Ignored when using MOUNT_TYPE_DIRECT
//#define TARGET_SELECT_PIN  2  // Debug button which cycles through a few different targets

/**
 * ----------------
 * Debug section
 * Use settings in this section to set up your telescope.
 * You can speed up time, enable debug messages over serial etc.
 * ----------------
 */

// Uncomment the following line to enable various debug features that would otherwise not get compiled into the firmware
#define DEBUG

// Uncomment the following line to enable sending debug statements via the serial port
#define DEBUG_SERIAL

// When initializing, a sanity check is performed on the constants set in config.h
// If the sanity check fails, the telescope will output a warning.
// If this is uncommented, the telescope will also stop and wait.
// If DEBUG_SERIAL is enabled, the telescope will wait until the user enters a continue command.
// If BUZZER_PIN is enabled, it will continuously beep until powered off or a continue command is received.
#define DEBUG_STOP_ON_CONFIG_INSANITY

// Uncomment to enable debug statements about how long various tasks take to execute
//#define DEBUG_TIMING
// Uncomment to enable debug statements regarding stepper movement
//#define DEBUG_SERIAL_STEPPER_MOVEMENT
// Uncomment to enable verbose debug statements regarding stepper movement (overrides DEBUG_SERIAL_STEPPER_MOVEMENT)
//#define DEBUG_SERIAL_STEPPER_MOVEMENT_VERBOSE

// Uncomment to enable debug statements regarding position calculations
//#define DEBUG_SERIAL_POSITION_CALC

// Uncomment the following line to enable debug messages of the GPS module
//#define DEBUG_GPS

// If this is uncommented the telescope assumes that it is homed on startup to whatever position is set in conversion.cpp. Ignored when using MOUNT_TYPE_DIRECT
//#define DEBUG_HOME_IMMEDIATELY

//#define DEBUG_DISABLE_ALL_STEPPERS     // Uncomment this to disable ALL stepper drivers
//#define DEBUG_DISABLE_AZIMUTH_STEPPER  // Uncomment this to disable only the azimuth stepper motor
//#define DEBUG_DISABLE_ALTITUDE_STEPPER // Uncomment this to disable only the altitude stepper

/**
 * ----------------
 * Position / GPS section
 *
 * ----------------
 */

// This location is used while initializing the GPS module, or if no GPS module is connected (GPS_FIXED_POS)
#define ALT 700.0     // Observer altitude in meters
#define LAT 47.0      // Observer latitude in degrees
#define LNG 12.0      // Observer longitude in degrees

// The initial date and time that is used when GPS_FIXED_POS is enabled
#define INITIAL_YEAR 2019
#define INITIAL_MONTH 11
#define INITIAL_DAY 10
#define INITIAL_HOUR 0
#define INITIAL_MINUTE 0
#define INITIAL_SECOND 0

// Timezone correction to convert FROM your current time to UTC. We could use GPS to get this value, but it would be difficult
// TODO Invert this value as it is confusing to have to set a negative X for the timezone UTC+X
#define TIMEZONE_CORRECTION_H (-1)

// Updates from the GPS module are ignored if you uncomment the next line
#define GPS_FIXED_POS

// Serial 1 TX on Arduino is connected to RX on the GPS module. Z_MIN on the RAMPS shield
#define GPS_SERIAL_PORT Serial1

// Uncomment this to have the telescope wait for a GPS fix before moving
// TODO Does not work currently
//#define GPS_WAIT_FOR_FIX

// Set the minimum number of GPS satellites to consider enough for returning the position
#define GPS_MIN_SATELLITES 3

// Set the minimum number of GPS satellites to consider enough for returning the time. Comment out to use GPS_MIN_SATELLITES
#define GPS_MIN_SATELLITES_TIME 3

// Should the telescope actually wait for the GPS fix and the min_satellites before moving?
//#define MOUNT_STOP_UNTIL_GPS_POS_VALID

// END GPS SECTION

/**
 * -------------------
 * Timing Section
 *
 * -------------------
 */
// Update the motor positions (e.g. call Mount::calculateMotorTargets()) every X ms
#define UPDATE_MOTOR_POS_MS 500
// The stepper interrupts get called every STEPPER_INTERRUPT_FREQ microseconds.
// 1.000.000 means the interrupt gets called every second. 1.000 means every ms
// The values below are reasonable for the default motor speeds and the respective boards
#ifdef BOARD_ARDUINO_MEGA
#define STEPPER_INTERRUPT_FREQ 500 // every 0.5ms
#endif
#ifdef BOARD_ARDUINO_DUE
#define STEPPER_INTERRUPT_FREQ 100 // every 0.1ms
#endif




// ----------------------------------------------------------------------
// ---------------------------- DEBUG MACROS ----------------------------
// ---------- Only change these if you know what you are doing ----------
// ----------------------------------------------------------------------

#if defined DEBUG && defined DEBUG_SERIAL

/* This part uses the code from https://blog.galowicz.de/2016/02/20/short_file_macro/
 * to set the __FILENAME__ constant which is used in the debug macros.
 */
using cstr = const char* const;

static constexpr cstr past_last_slash(cstr str, cstr last_slash)
{
	return
		*str == '\0' ? last_slash :
		(*str == '/'  || *str == '\\') ? past_last_slash(str + 1, str + 1) :
		past_last_slash(str + 1, last_slash);
}

static constexpr cstr past_last_slash(cstr str)
{
	return past_last_slash(str, str);
}

// The filename part of __FILE__ excluding the path
#define __FILENAME__ ({constexpr cstr sf__ {past_last_slash(__FILE__)}; sf__;})

// Prints a debug message
#define DEBUG_PRINT(x)    Serial.print(x)

// Prints a debug message line
#define DEBUG_PRINTLN(x)  Serial.println(x)

// Prints a debug message with time, file name and line number
#define DEBUG_PRINT_V(x)   \
		   Serial.print(String(millis() / 1000., 2)); \
		   Serial.print(": ");       \
		   Serial.print(__FILENAME__);\
		   Serial.print(':');          \
		   Serial.print(__LINE__);      \
		   Serial.print(' ');            \
		   Serial.print(x);

// Prints a debug message line with timestamp, file name and line number
#define DEBUG_PRINTLN_V(x)   \
		   Serial.print(String(millis() / 1000., 2)); \
		   Serial.print(": ");       \
		   Serial.print(__FILENAME__);\
		   Serial.print(':');          \
		   Serial.print(__LINE__);      \
		   Serial.print(' ');            \
		   Serial.println(x);

// Prints a debug message with timestamp, function name, file name and line number
#define DEBUG_PRINT_VV(x)   \
		   Serial.print(String(millis() / 1000., 2)); \
		   Serial.print(": ");     \
		   Serial.print(__PRETTY_FUNCTION__); \
		   Serial.print(' ');        \
		   Serial.print(__FILENAME__);\
		   Serial.print(':');          \
		   Serial.print(__LINE__);      \
		   Serial.print(' ');            \
		   Serial.print(x);

// Prints a debug message line with timestamp, function name, file name and line number
#define DEBUG_PRINTLN_VV(x) \
		   Serial.print(String(millis() / 1000., 2)); \
		   Serial.print(": ");     \
		   Serial.print(__PRETTY_FUNCTION__); \
		   Serial.print(' ');        \
		   Serial.print(__FILENAME__);\
		   Serial.print(':');          \
		   Serial.print(__LINE__);      \
		   Serial.print(' ');            \
		   Serial.println(x);
#else
// (Disabled) Prints a debug message. Define the DEBUGand DEBUG_SERIAL constants to enable
#define DEBUG_PRINT(x)

// (Disabled) Prints a debug message line. Define the DEBUGand DEBUG_SERIAL constants to enable
#define DEBUG_PRINTLN(x)

// (Disabled) Prints a debug message with timestamp, file name and line number. Define the DEBUG and DEBUG_SERIAL constants to enable
#define DEBUG_PRINT_V(x)

// (Disabled) Prints a debug message line with timestamp, file name and line number. Define the DEBUG and DEBUG_SERIAL constants to enable
#define DEBUG_PRINTLN_V(x)

// (Disabled) Prints a debug message with timestamp, function name, file name and line number. Define the DEBUG and DEBUG_SERIAL constants to enable
#define DEBUG_PRINT_VV(x)

// (Disabled) Prints a debug message line with timestamp, function name, file name and line number. Define the DEBUG and DEBUG_SERIAL constants to enable
#define DEBUG_PRINTLN_VV(x)
#endif


// These define whether the stepper drivers are enabled at all. Please do not modify
#if !defined DEBUG || !(defined DEBUG_DISABLE_ALL_STEPPERS || defined DEBUG_DISABLE_AZIMUTH_STEPPER)
#define AZ_ENABLE
#endif

#if !defined DEBUG || !(defined DEBUG_DISABLE_ALL_STEPPERS || defined DEBUG_DISABLE_ALTITUDE_STEPPER)
#define ALT_ENABLE
#endif