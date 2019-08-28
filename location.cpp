#include <Arduino.h>
#include <FuGPS.h>
#include <Time.h>

#include "./config.h"

#ifdef BOARD_ARDUINO_MEGA
#include <EEPROM.h>
#endif

#include "./location.h"

bool gpsAlive = false;

TelescopePosition gpsPosition = { 0, LAT, LNG }; // This stores the latest reported GPS position. Defaults to 0, LAT, LNG as set in config.h

// These are the EEPROM addresses
const unsigned int eeprom_alt = 0;
const unsigned int eeprom_lat = eeprom_alt + sizeof(float);
const unsigned int eeprom_lng = eeprom_lat + sizeof(float);
const unsigned int eeprom_hours = eeprom_lng + sizeof(int);
const unsigned int eeprom_minutes = eeprom_hours + sizeof(int);
const unsigned int eeprom_seconds = eeprom_minutes + sizeof(int);


/**
 * Loads position data from EEPROM and stores it in the global gpsPosition variable
 */
void loadFromEEPROM() {
	float altitude, latitude, longitude;
	int hours, minutes, seconds;


#ifdef BOARD_ARDUINO_MEGA
	EEPROM.get(eeprom_alt, altitude);
	EEPROM.get(eeprom_lat, latitude);
	EEPROM.get(eeprom_lng, longitude);
	EEPROM.get(eeprom_hours, hours);
	EEPROM.get(eeprom_minutes, minutes);
	EEPROM.get(eeprom_seconds, seconds);
#else
	altitude = 0;
	latitude = 0;
	longitude = 0;
	hours = 0;
	seconds = 0;
#endif

	gpsPosition.altitude = altitude;
	gpsPosition.latitude = latitude;
	gpsPosition.longitude = longitude;
	//gpsPosition.hours = hours;
	//gpsPosition.minutes = minutes;
	//gpsPosition.seconds = seconds;
}

/**
 * Updates position data in the EEPROM
 */
void updateEEPROM(float altitude, float latitude, float longitude, int hours,
		int minutes, int seconds) {
#ifdef BOARD_ARDUINO_MEGA
	EEPROM.update(eeprom_alt, altitude);
	EEPROM.update(eeprom_lat, latitude);
	EEPROM.update(eeprom_lng, longitude);
	EEPROM.update(eeprom_hours, hours);
	EEPROM.update(eeprom_minutes, minutes);
	EEPROM.update(eeprom_seconds, seconds);
#endif

	gpsPosition.altitude = altitude;
	gpsPosition.latitude = latitude;
	gpsPosition.longitude = longitude;
	//gpsPosition.hours = hours;
	//gpsPosition.minutes = minutes;
	//gpsPosition.seconds = seconds;
}

/**
 * Initializes GPS
 */
TelescopePosition& initGPS(FuGPS &fuGPS) {
	setTime(23, 38, 0, 28, 8, 2019);

	// Begin communicating with the GPS module
	Serial1.begin(9600);

	// We use the builtin LED to indicate whether GPS fix is available
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);

	// Load stored position and time from the EEPROM
	loadFromEEPROM();

	// Wait a little bit so that the module can initialize
	delay(500);

	// Set up the module
	fuGPS.sendCommand(FUGPS_PMTK_SET_NMEA_BAUDRATE_9600);
	fuGPS.sendCommand(FUGPS_PMTK_SET_NMEA_UPDATERATE_1HZ);
	//fuGPS.sendCommand(FUGPS_PMTK_API_SET_NMEA_OUTPUT_DEFAULT);
	fuGPS.sendCommand(FUGPS_PMTK_API_SET_NMEA_OUTPUT_RMCGGA);

	return gpsPosition;
}

bool didSetTimeWithoutFix = false;
bool didSetTimeWithFix = false;

TelescopePosition& handleGPS(FuGPS &fuGPS) {

	if (fuGPS.read()) {
		// We don't know, which message was came first (GGA or RMC).
		// Thats why some fields may be empty.

		gpsAlive = true;

		if (fuGPS.hasFix() == true) {
			digitalWrite(LED_BUILTIN, HIGH);

			if (fuGPS.Altitude > 1.0 && fuGPS.Latitude > 1.0
					&& fuGPS.Longitude > 1.0)
				updateEEPROM(fuGPS.Altitude, fuGPS.Latitude, fuGPS.Longitude,
						fuGPS.Hours / 255.000 * 24.000 + 21.000,
						fuGPS.Minutes / 255.000 * 60.000,
						fuGPS.Seconds / 255.000 * 60.000);

			if (!didSetTimeWithFix) {
				didSetTimeWithFix = true;
				setTime(fuGPS.Hours - TIMEZONE_CORRECTION_H, fuGPS.Minutes, fuGPS.Seconds, fuGPS.Days, fuGPS.Months,
						fuGPS.Years);
			}
#ifdef DEBUG_GPS
			// Data from GGA or RMC
			DEBUG_PRINTLN("Location (decimal degrees): https://www.google.com/maps/search/?api=1&query="
							+ String(fuGPS.Latitude, 6) + ","
							+ String(fuGPS.Longitude, 6));

#endif
			//storePosition(fuGPS.Altitude, fuGPS.Latitude, fuGPS.Longitude);
		} else {
			digitalWrite(LED_BUILTIN, LOW);

			// Set time even if we do not have a GPS fix. At least 3 Satellites are required though
			if (!didSetTimeWithFix && !didSetTimeWithoutFix && fuGPS.Satellites > 3) {
				didSetTimeWithoutFix = true; // Only set time once
				setTime(fuGPS.Hours - TIMEZONE_CORRECTION_H, fuGPS.Minutes, fuGPS.Seconds, fuGPS.Days, fuGPS.Months,
						fuGPS.Years);
			}
			//Serial.println("No GPS fix: , Satellites: " + String(fuGPS.Satellites, 6));
			//loadPosition();
		}

#ifdef DEBUG_GPS
		//Serial.println(String(fuGPS.Hours, 6));
		//Serial.println((fuGPS.Minutes / 255.00 * 60.00 - 30));
		//Serial.println(fuGPS.Seconds / 255.00 * 60.00);
		DEBUG_PRINTLN("Quality: " + String(fuGPS.Quality, 6) + ", Satellites: "
						+ String(fuGPS.Satellites, 6) + "; Time: "
						+ String(fuGPS.Hours) + "hh " +
						String(fuGPS.Minutes) + "mm " +
						String(fuGPS.Seconds, 6) + "ss; StoredAlt: "
						+ String(fuGPS.Altitude, 6) + "; StoredLat: "
						+ String(fuGPS.Latitude, 6) + "; StoredLng: "
						+ String(fuGPS.Longitude, 6));
#endif
	} else {
		if (millis() % 1000 > 700) {
			digitalWrite(LED_BUILTIN, HIGH);
		} else {
			digitalWrite(LED_BUILTIN, LOW);
		}
	}

	// Default is 10 seconds
	if (fuGPS.isAlive() == false) {
		//Serial.println("Cannot read");
		if (gpsAlive == true) {
			gpsAlive = false;

#ifdef DEBUG_GPS
			DEBUG_PRINTLN("GPS module not responding with valid data.");
			DEBUG_PRINTLN("Check wiring or restart.");
#endif
		} else {
			if (millis() % 200 > 100) {
				digitalWrite(LED_BUILTIN, HIGH);
			} else {
				digitalWrite(LED_BUILTIN, LOW);
			}
		}
	}

	return gpsPosition;
}
