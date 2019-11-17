#include <Arduino.h>
#include <FuGPS.h>
#include <Time.h>

#include "./config.h"

#ifdef BOARD_ARDUINO_MEGA
	#include <EEPROM.h>
#endif

#include "./location.h"

bool gpsAlive = false;

ObserverPosition gpsPosition = { 0, LAT, LNG }; // This stores the latest reported GPS position. Defaults to 0, LAT, LNG as set in config.h

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


#ifdef BOARD_ARDUINO_MEGA
	EEPROM.get(eeprom_alt, altitude);
	EEPROM.get(eeprom_lat, latitude);
	EEPROM.get(eeprom_lng, longitude);
#else
	altitude = 0;
	latitude = 0;
	longitude = 0;
#endif

	gpsPosition = { altitude,latitude, longitude };
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

	gpsPosition = { altitude,latitude, longitude };
}


// The current local sidereal time at the specified longitude and the current time
// Simply said, it's the right ascension of the point in the sky directly above the observer
// More information and an explanation of the algorithm can be found at: http://www.astro.sunysb.edu/fwalter/AST443/times.html
double get_local_sidereal_time(const double degrees_longitude) {
	const unsigned long day_seconds = hour() * 3600UL + minute() * 60 + second();
	const unsigned long timestamp = now();

	const double julian_day = timestamp / 86400.0;
	const double T = (julian_day - 10957.5) / 36525.0;
	const double G0 = 24110.548 + (8640184.812866 * T) + (0.93104 * T * T) - (0.0000062 * T * T * T);
	const double a = day_seconds * 1.00273790934;

	const double GST = fmod((((G0 + a) / 3600.0) + 9600), 24);

	return GST - (degrees_longitude / 15.);
}
