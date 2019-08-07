#ifndef CONFIG_H_
#define CONFIG_H_

/*
 * Azimuth Stepper
 * This is the stepper that rotates the mount horizontally
 */
#define X_ENABLE_PIN       38    // RAMPS 1.4
#define X_STEP_PIN         54    // RAMPS 1.4
#define X_DIR_PIN          55    // RAMPS 1.4
#define X_STEPS_PER_REV 32000    // How many steps the stepper motor needs to complete for one a horizontal 360degree revolution of the telescope

/*
 * Altitude stepper
 * This is the stepper that rotates the telescope horizontally
 */
#define Y_ENABLE_PIN       56    // RAMPS 1.4
#define Y_STEP_PIN         60    // RAMPS 1.4
#define Y_DIR_PIN          61    // RAMPS 1.4
#define Y_STEPS_PER_REV 32000    // How many steps the stepper motor needs to complete for a vertical 360degree revolution of the telescope


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

// TIME_FACTOR
// 1 means real-time. 2 means time passes twice as fast
// Do NOT set to something ridiculously high if your motors are connected or rapid unplanned disassembly of setup may occur
// Use this to test your setup
// Negative values can be used to reverse the passing of time. Caution: This does _not_ rewind actual tims. We're actively working on that feature (PR #1)
const short TIME_FACTOR = 1;

// For a successful build you have to either
// 1) uncomment this and set the LAT and LNG in the next lines or
// 2) have a GPS module connected and setup (see GPS section)
#define GPS_FIXED_POS

#ifdef GPS_FIXED_POS
#define LAT 47.426430 // Observer latitude in degrees
#define LNG 12.849180 // Observer longitude in degrees

#define TIMEZONE_CORRECTION_H -2 // Timezone correction to convert to UTC
#endif

/**
 * ----------------
 * GPS section
 * ----------------
 */
#ifndef GPS_FIXED_POS
// GPS setup will be here
#endif

#endif /* CONFIG_H_ */
