#ifndef CONFIG_H_
#define CONFIG_H_

#define STEPPER_INTERRUPT_FREQ 100   // Unit is microseconds; 1.000.000 is one second

/*
 * Azimuth Stepper
 * This is the stepper that rotates the mount horizontally
 */
#define AZ_ENABLE_PIN       38    // RAMPS 1.4
#define AZ_STEP_PIN         54    // RAMPS 1.4
#define AZ_DIR_PIN          55    // RAMPS 1.4
#define AZ_STEPS_PER_REV  119467   // How many steps the stepper motor needs to complete for one a horizontal 360degree revolution of the telescope. 3200 : 1 and 560 : 15
#define AZ_MAX_ACCEL       300    // Maximum acceleration for the azimuth stepper
#define AZ_MAX_SPEED       4000    // Maximum speed for the azimuth stepper

/*
 * Altitude stepper
 * This is the stepper that rotates the telescope vertically
 */
#define ALT_ENABLE_PIN       56  // RAMPS 1.4
#define ALT_STEP_PIN         60  // RAMPS 1.4
#define ALT_DIR_PIN          61  // RAMPS 1.4
#define ALT_STEPS_PER_REV   36960 // 90degrees //147840 // How many steps the stepper motor needs to complete for a vertical 360degree revolution of the telescope 5.18:1 and 3200 : 1 and 105 : 12
#define ALT_MAX_ACCEL       400 // Maximum acceleration for the altitude stepper
#define ALT_MAX_SPEED       10000  // Maximum speed for the altitude stepper

#define STEPPERS_ON_PIN    34 // RAMPS Q_DIR_PIN. If this pin is HIGH, the steppers are turned on
#define HOME_NOW_PIN       36 // RAMPS Q_STEP_PIN. If this pin is HIGH, homing is performed
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

// Uncomment the following line to enable debug messages of the GPS module
//#define DEBUG_GPS

// If this is uncommented the telescope assumes that it is homed on startup to whatever position is set in conversion.cpp
#define DEBUG_HOME_IMMEDIATELY

//#define DEBUG_DISABLE_ALL_STEPPERS     // Uncomment this to disable ALL stepper drivers
//#define DEBUG_DISABLE_AZIMUTH_STEPPER  // Uncomment this to disable only the azimuth stepper motor
//#define DEBUG_DISABLE_ALTITUDE_STEPPER // Uncomment this to disable only the altitude stepper

// Please only modify this if you know what you are doing
#if !defined DEBUG || !(defined DEBUG_DISABLE_ALL_STEPPERS || defined DEBUG_DISABLE_AZIMUTH_STEPPER)
#define AZ_ENABLE
#endif

#if !defined DEBUG || !(defined DEBUG_DISABLE_ALL_STEPPERS || defined DEBUG_DISABLE_ALTITUDE_STEPPER)
#define ALT_ENABLE
#endif
// Continue modifying

// Time multiplication factor. 1 means real-time. 2 means time passes twice as fast
// Do NOT set to something ridiculously high if your motors are connected or rapid unplanned disassembly of setup may occur
// DO use this to test your setup, but start with sensible values like 1
// Negative values can be used to reverse the passing of time. Caution: This does _not_ rewind actual time. We're actively working on that feature (PR #1)
const short TIME_FACTOR = 1;

// For a successful build you have to either
// 1) uncomment this and set the LAT and LNG in the next lines or
// 2) have a GPS module connected and setup (see GPS section)
#define GPS_FIXED_POS

#ifdef GPS_FIXED_POS
#define LAT 47.425011 // Observer latitude in degrees
#define LNG 12.846258 // Observer longitude in degrees

#define TIMEZONE_CORRECTION_H -2 // Timezone correction to convert to UTC
#endif /* !GPS_FIXED_POS */

/**
 * ----------------
 * GPS section
 * This whole section is ignored, if GPS_FIXED_POS is enabled in the debug section above.
 * ----------------
 */
#ifdef GPS_FIXED_POS

#define GPS_SERIAL_PORT Serial1 // Serial 1 TX on Arduino is connected to RX on the GPS module

#endif /* GPS_FIXED_POS */

/*
 * Debug macros. These can be ignored
 */
#if defined DEBUG && defined DEBUG_SERIAL
#define DEBUG_PRINT(x)    Serial.print(x)
#define DEBUG_PRINTLN(x)  Serial.println(x)
#define DEBUG_PRINT_V(x)   \
		   Serial.print(millis());\
		   Serial.print(": ");     \
		   Serial.print(__FILE__);  \
		   Serial.print(':');        \
		   Serial.print(__LINE__);    \
		   Serial.print(' ');          \
		   Serial.print(x);
#define DEBUG_PRINTLN_V(x)   \
		   Serial.print(millis());\
		   Serial.print(": ");     \
		   Serial.print(__FILE__);  \
		   Serial.print(':');        \
		   Serial.print(__LINE__);    \
		   Serial.print(' ');          \
		   Serial.println(x);
#define DEBUG_PRINT_VV(x)   \
		   Serial.print(millis());\
		   Serial.print(": ");     \
		   Serial.print(__PRETTY_FUNCTION__); \
		   Serial.print(' ');        \
		   Serial.print(__FILE__);    \
		   Serial.print(':');          \
		   Serial.print(__LINE__);      \
		   Serial.print(' ');            \
		   Serial.print(x);
#define DEBUG_PRINTLN_VV(x) \
		   Serial.print(millis());\
		   Serial.print(": ");     \
		   Serial.print(__PRETTY_FUNCTION__); \
		   Serial.print(' ');        \
		   Serial.print(__FILE__);    \
		   Serial.print(':');          \
		   Serial.print(__LINE__);      \
		   Serial.print(' ');            \
		   Serial.println(x);
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINT_V(x)
#define DEBUG_PRINT_VV(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTLN_V(x)
#define DEBUG_PRINTLN_VV(x)
#endif

#endif /* CONFIG_H_ */
