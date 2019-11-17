#include "./Observer.h"
#include "./GpsObserver.h"
#include "./config.h"

#include <FuGPS.h>
#include <Time.h>

GpsObserver::GpsObserver() :
	_gps(GPS_SERIAL_PORT) {
	setPosition({ ALT, LAT, LNG });
}

void GpsObserver::initialize() {
	// Begin communicating with the GPS module
	GPS_SERIAL_PORT.begin(9600);

	// We use the builtin LED to indicate whether GPS fix is available
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);

	// Load stored position and time from the EEPROM
	//loadFromEEPROM(); // Disabled

	// Wait a little bit so that the module can initialize
	delay(500);

	// Set up the module
	_gps.sendCommand(FUGPS_PMTK_SET_NMEA_BAUDRATE_9600);
	_gps.sendCommand(FUGPS_PMTK_SET_NMEA_UPDATERATE_1HZ);
	//fuGPS.sendCommand(FUGPS_PMTK_API_SET_NMEA_OUTPUT_DEFAULT);
	_gps.sendCommand(FUGPS_PMTK_API_SET_NMEA_OUTPUT_RMCGGA);

	setPosition({
		_gps.Altitude,
		_gps.Latitude,
		_gps.Longitude
	});
}

void GpsObserver::updatePosition() {
	// Has the GPS module sent any updates?
	if (_gps.read()) {
		// If there are updates, it's connected and running, obviously
		_gpsAlive = true;

		// Are there enough satellites in view to determine the position/time?
		if (_gps.hasFix() == true) {
			digitalWrite(LED_BUILTIN, HIGH);

			// Update the current position
			setPosition({
				_gps.Altitude,
				_gps.Latitude,
				_gps.Longitude
			});

			if (!_didSetTimeWithFix) {
				_didSetTimeWithFix = true;
				setTime(_gps.Hours - TIMEZONE_CORRECTION_H, _gps.Minutes, _gps.Seconds, _gps.Days, _gps.Months, _gps.Years);
			}

			#ifdef DEBUG_GPS
				// Data from GGA or RMC
				DEBUG_PRINTLN("Location (decimal degrees): https://www.google.com/maps/search/?api=1&query="
					+ String(fuGPS.Latitude, 6) + ","
					+ String(fuGPS.Longitude, 6));
			#endif

			// Store the current positon in permanent memory
			//storePosition(fuGPS.Altitude, fuGPS.Latitude, fuGPS.Longitude); // Disabled
		}
		else {
			digitalWrite(LED_BUILTIN, LOW);
			#ifdef GPS_MIN_SATELLITES_TIME
				const unsigned int min_satellites = GPS_MIN_SATELLITES_TIME;
			#elif defined GPS_MIN_SATELLITES
				const unsigned int min_satellites = GPS_MIN_SATELLITES;
			#else
				const unsigned int min_satellites = 3;
			#endif
			// Set time even if we do not have a GPS fix. At least 3 Satellites are required though
			if (!_didSetTimeWithFix && !_didSetTimeWithoutFix && _gps.Satellites > min_satellites) {
				_didSetTimeWithoutFix = true; // Only set time once
				setTime(_gps.Hours - TIMEZONE_CORRECTION_H, _gps.Minutes, _gps.Seconds, _gps.Days, _gps.Months, _gps.Years);
			}
		}

		#ifdef DEBUG_GPS
			DEBUG_PRINTLN("Quality: " + String(_gps.Quality, 6) + ", Satellites: "
				+ String(_gps.Satellites, 6) + "; Time: "
				+ String(_gps.Hours) + "hh " +
				String(_gps.Minutes) + "mm " +
				String(_gps.Seconds, 6) + "ss; StoredAlt: "
				+ String(_gps.Altitude, 6) + "; StoredLat: "
				+ String(_gps.Latitude, 6) + "; StoredLng: "
				+ String(_gps.Longitude, 6));
		#endif
	}
	else {
		// TODO Rewrite the LED status code. At the moment it's rather useless
		if (millis() % 1000 > 700) {
			digitalWrite(LED_BUILTIN, HIGH);
		}
		else {
			digitalWrite(LED_BUILTIN, LOW);
		}
	}

	// Default is 10 seconds
	if (_gps.isAlive() == false) {
		if (_gpsAlive == true) {
			_gpsAlive = false;

			#ifdef DEBUG_GPS
				DEBUG_PRINTLN("GPS module not responding with valid data.");
				DEBUG_PRINTLN("Check wiring or restart.");
			#endif
		}
		else {
			if (millis() % 200 > 100) {
				digitalWrite(LED_BUILTIN, HIGH);
			}
			else {
				digitalWrite(LED_BUILTIN, LOW);
			}
		}
	}
}

bool GpsObserver::hasValidPosition() {
	if (_gpsAlive) {
		bool hasFix = true;
		bool enoughSatellites = true;
		#ifdef GPS_WAIT_FOR_FIX
			// We should wait for a GPS fix
			hasFix = _gps.hasFix();
		#endif

		#ifdef GPS_MIN_SATELLITES
			// Do we have enough satellites?
			enoughSatellites = _gps.Satellites >= GPS_MIN_SATELLITES;
		#endif

		return hasFix && enoughSatellites;
	}

	return false;
}

void GpsObserver::printDebugInfo() {
	Serial.println("GPS Status: ");
	Serial.print("Alive      ... ");
	Serial.println(_gps.isAlive() ? "Yes" : "No");
	Serial.print("Fix        ... ");
	Serial.println((_gps.hasFix() ? "Yes" : "No"));
	Serial.print("Satellites ... ");
	Serial.println(String(_gps.Satellites, 6));
	Serial.print("Acceptable ... ");
	Serial.println(hasValidPosition() ? "Yes" : "No");
	Serial.print("Quality    ... ");
	Serial.println(String(_gps.Quality, 6));
	Serial.print("Altitude   ... ");
	Serial.println(String(_gps.Altitude, 6));
	Serial.print("Latitude   ... ");
	Serial.println(String(_gps.Latitude, 6));
	Serial.print("Longitude  ... ");
	Serial.println(String(_gps.Longitude, 6));
}