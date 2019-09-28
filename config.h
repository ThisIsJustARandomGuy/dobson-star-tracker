#pragma once

// BOARD config
// Select
// Only define one of these
//#define BOARD_ARDUINO_MEGA
#define BOARD_ARDUINO_UNO


/*
 * Azimuth Stepper
 * This is the stepper that rotates the mount horizontally
 */
#define AZ_ENABLE_PIN       38     // RAMPS 1.4 X stepper
#define AZ_STEP_PIN         54     // RAMPS 1.4
#define AZ_DIR_PIN          55     // RAMPS 1.4
#define AZ_STEPS_PER_REV   119467  // How many steps the stepper motor needs to complete for one a horizontal 360degree revolution of the telescope (my setup: 3200 : 1 and 560 : 15)
#define AZ_MAX_ACCEL       300     // Maximum acceleration for the azimuth stepper
#define AZ_MAX_SPEED       4000    // Maximum speed for the azimuth stepper

/*
 * Altitude stepper
 * This is the stepper that rotates the telescope vertically
 */
#define ALT_ENABLE_PIN       56    // RAMPS 1.4 Y stepper
#define ALT_STEP_PIN         60    // RAMPS 1.4
#define ALT_DIR_PIN          61    // RAMPS 1.4
#define ALT_STEPS_PER_REV    36960 // for 90degrees //147840 for a full rotation // How many steps the stepper motor needs to complete for a vertical 360degree revolution of the telescope (my setup: 5.18:1 and 3200 : 1 and 105 : 12)
#define ALT_MAX_ACCEL        400   // Maximum acceleration for the altitude stepper
#define ALT_MAX_SPEED        10000 // Maximum speed for the altitude stepper

/*
 * Other Pins
 * y+ is dead
 * y- buzzer
 * x+ stepper on
 * x- pushbutton
 */
#define BUZZER_PIN         14 // Uncomment if you have a buzzer installed. Default pin for RAMPS 1.4 is Y_MIN
#define STEPPERS_ON_PIN    3  // Uncomment if you have a stepper switch installed. Default pin for RAMPS 1.4 is  X_MIN. If this pin is HIGH, the steppers are turned on.
//#define HOME_NOW_PIN       3  // Uncomment if you have a homing mode switch installed. Default pin for RAMPS 1.4 is X_MAX. If this pin is HIGH, homing is performed
#define TARGET_SELECT_PIN  2  // Debug button which cycles through a few different targets

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

// Uncomment to enable debug statements regarding stepper movement
#define DEBUG_SERIAL_STEPPER_MOVEMENT
// Uncomment to enable verbose debug statements regarding stepper movement (overrides DEBUG_SERIAL_STEPPER_MOVEMENT)
//#define DEBUG_SERIAL_STEPPER_MOVEMENT_VERBOSE

// Uncomment to enable debug statements regarding position calculations
#define DEBUG_SERIAL_POSITION_CALC

// Uncomment the following line to enable debug messages of the GPS module
//#define DEBUG_GPS

// If this is uncommented the telescope assumes that it is homed on startup to whatever position is set in conversion.cpp
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

// This location is used while initializing the GPS module, or if no GPS module is connected.
#define LAT 47.425011 // Observer latitude in degrees
#define LNG 12.846258 // Observer longitude in degrees

// Timezone correction to convert FROM your current time to UTC. We could use GPS to get this value, but it would be difficult
// TODO Invert this value as it is confusing to have to set a negative X for the timezone UTC+X
#define TIMEZONE_CORRECTION_H -2

// Updates from the GPS module are ignored if you uncomment the next line
// TODO Does not work currently
#define GPS_FIXED_POS

// Serial 1 TX on Arduino is connected to RX on the GPS module. Z_MIN on the RAMPS shield
#define GPS_SERIAL_PORT Serial1

// END GPS SECTION

// The stepper interrupts get called every STEPPER_INTERRUPT_FREQ microseconds.
// 1.000.000 means the interrupt gets called every second. 1.000 means every ms
// The values below are reasonable for the default motor speeds and the respective boards
#ifdef BOARD_ARDUINO_MEGA
#define STEPPER_INTERRUPT_FREQ 500 // every 0.5ms
#endif
#ifdef BOARD_ARDUINO_UNO
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