#include <Arduino.h>
#include <EEPROM.h>
#include <FuGPS.h>
#include "./config.h"

#include "./location.h"


bool gpsAlive = false;

Position gpsPosition; // This stores the latest reported GPS position. Defaults to 0, LAT, LNG as set in config.h

// These are the EEPROM addresses
const unsigned int eeprom_alt = 0;
const unsigned int eeprom_lat = eeprom_alt + sizeof(float);
const unsigned int eeprom_lng = eeprom_lat + sizeof(float);


/**
 * This loads position data from EEPROM and stores it in the global gpsPosition variable
 */
void loadFromEEPROM() {
	float altitude;
	float latitude;
	float longitude;
	EEPROM.get(eeprom_alt, altitude);
	EEPROM.get(eeprom_lat, latitude);
	EEPROM.get(eeprom_lng, longitude);
	gpsPosition.altitude = altitude;
	gpsPosition.latitude = latitude;
	gpsPosition.longitude = longitude;
}

/**
 * Updates position data in the EEPROM
 */
void updateEEPROM(float altitude, float latitude, float longitude) {
	EEPROM.update(eeprom_alt, altitude);
	EEPROM.update(eeprom_lat, latitude);
	EEPROM.update(eeprom_lng, longitude);
	gpsPosition.altitude = altitude;
	gpsPosition.latitude = latitude;
	gpsPosition.longitude = longitude;
}

/**
 * Initializes GPS
 */
Position& initGPS(FuGPS &fuGPS) {
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);
	Serial1.begin(9600);
	delay(100);
	fuGPS.sendCommand(FUGPS_PMTK_SET_NMEA_BAUDRATE_9600);
	fuGPS.sendCommand(FUGPS_PMTK_SET_NMEA_UPDATERATE_1HZ);
	fuGPS.sendCommand(FUGPS_PMTK_API_SET_NMEA_OUTPUT_RMCGGA);
	loadFromEEPROM();
	//fuGPS.sendCommand(FUGPS_PMTK_API_SET_NMEA_OUTPUT_RMCGGA);
	//Serial.println("Beginning");
	return gpsPosition;

}

Position& handleGPS(FuGPS &fuGPS) {
	/*while (Serial1.available() > 0) {
		// get the byte data from the GPS
		byte gpsData = Serial1.read();
		Serial.write(gpsData);
	}
	 return;*/
	if (fuGPS.read()) {
		// We don't know, which message was came first (GGA or RMC).
		// Thats why some fields may be empty.

		gpsAlive = true;

		if (fuGPS.hasFix() == true) {
			digitalWrite(LED_BUILTIN, HIGH);

#ifdef DEBUG_SERIAL
			Serial.print("Accuracy (HDOP): ");
			Serial.println(fuGPS.Accuracy);
			Serial.print("Altitude (above sea level): ");
			Serial.println(fuGPS.Altitude);

			// Data from GGA or RMC
			Serial.print("Location (decimal degrees): ");
			Serial.println(
					"https://www.google.com/maps/search/?api=1&query="
							+ String(fuGPS.Latitude, 6) + ","
							+ String(fuGPS.Longitude, 6));
			
			if (fuGPS.Altitude != 0 && fuGPS.Latitude != 0
					&& fuGPS.Longitude != 0)
			updateEEPROM(fuGPS.Altitude, fuGPS.Latitude, fuGPS.Longitude);

#endif
			//storePosition(fuGPS.Altitude, fuGPS.Latitude, fuGPS.Longitude);
		} else {
			digitalWrite(LED_BUILTIN, LOW);
			//Serial.println("No GPS fix");
			//loadPosition();
		}

#ifdef DEBUG_SERIAL
		Serial.println(
				"Quality: " + String(fuGPS.Quality, 6) + ", Satellites: "
						+ String(fuGPS.Satellites, 6) + "; Alt: "
						+ String(fuGPS.Altitude, 6) + "; Lat: "
						+ String(fuGPS.Latitude, 6) + "; Lng: "
						+ String(fuGPS.Longitude, 6) + "; StroedAlt: "
						+ String(gpsPosition.altitude, 6) + "; StoredLat: "
						+ String(gpsPosition.latitude, 6) + "; StoredLng: "
						+ String(gpsPosition.longitude, 6));
#endif
		/*Serial.print(fuGPS.Quality);
		Serial.print(",");
		Serial.print(fuGPS.Satellites);
		Serial.print(",");
		//Serial.print(latestPos.altitude);
		Serial.print(",");
		//Serial.print(latestPos.latitude);
		Serial.println(",");
		 //Serial.println(latestPos.longitude);*/
	}

	// Default is 10 seconds
	if (fuGPS.isAlive() == false) {
		if (gpsAlive == true) {
			gpsAlive = false;

#ifdef DEBUG_SERIAL
			Serial.println("GPS module not responding with valid data.");
			Serial.println("Check wiring or restart.");
#endif
		}
	}

	return gpsPosition;
}

