#ifndef CONFIG_H_
#define CONFIG_H_

/*
 * Azimuth Stepper
 * This is the stepper that rotates the mount horizontally
 */
#define AZ_ENABLE_PIN       38    // RAMPS 1.4
#define AZ_STEP_PIN         54    // RAMPS 1.4
#define AZ_DIR_PIN          55    // RAMPS 1.4
#define AZ_STEPS_PER_REV  119467   // How many steps the stepper motor needs to complete for one a horizontal 360degree revolution of the telescope
#define AZ_MAX_ACCEL       70    // Maximum acceleration for the azimuth stepper
#define AZ_MAX_SPEED       2000    // Maximum speed for the azimuth stepper

/*
 * Altitude stepper
 * This is the stepper that rotates the telescope vertically
 */
#define ALT_ENABLE_PIN       56  // RAMPS 1.4
#define ALT_STEP_PIN         60  // RAMPS 1.4
#define ALT_DIR_PIN          61  // RAMPS 1.4
#define ALT_STEPS_PER_REV  140000 // How many steps the stepper motor needs to complete for a vertical 360degree revolution of the telescope
#define ALT_MAX_ACCEL       1000 // Maximum acceleration for the altitude stepper
#define ALT_MAX_SPEED       20000  // Maximum speed for the altitude stepper

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

// Uncomment the following line to disable the stepper drivers
//#define DEBUG_DISABLE_ALL_STEPPERS
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

#endif /* CONFIG_H_ */
