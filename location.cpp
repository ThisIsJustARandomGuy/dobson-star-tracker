#include <Arduino.h>
#include <FuGPS.h>
#include "./config.h"

#include "./location.h"

FuGPS fuGPS(Serial1);
bool gpsAlive = false;
#define GPS Serial2
#define DBG Serial


void initGPS() {

	Serial1.begin(9600);
	Serial.println("Beginning");
}

void handleGPS() {
	while (Serial1.available() > 0) {
		// get the byte data from the GPS
		byte gpsData = Serial1.read();
		Serial.write(gpsData);
	}
	return;
	/*if (fuGPS.read()) {
		// We don't know, which message was came first (GGA or RMC).
		// Thats why some fields may be empty.

		gpsAlive = true;

		if (fuGPS.hasFix() == true) {
			digitalWrite(LED_BUILTIN, HIGH);
			// Data from GGA message
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

			//storePosition(fuGPS.Altitude, fuGPS.Latitude, fuGPS.Longitude);
		} else {
			digitalWrite(LED_BUILTIN, LOW);
			Serial.println("No GPS fix");
			//loadPosition();
		}

		//Serial.println("Quality: " + String(fuGPS.Quality, 6) + ", Satellites: " + String(fuGPS.Satellites, 6) + "; Alt: " + String(latestPos.altitude, 6) + "; Lat: " + String(latestPos.latitude, 6) + "; Lng: " + String(latestPos.longitude) );
		Serial.print(fuGPS.Quality);
		Serial.print(",");
		Serial.print(fuGPS.Satellites);
		Serial.print(",");
		//Serial.print(latestPos.altitude);
		Serial.print(",");
		//Serial.print(latestPos.latitude);
		Serial.println(",");
		//Serial.println(latestPos.longitude);
	} else {
		Serial.println("Nothing reported");
	}

	// Default is 10 seconds
	if (fuGPS.isAlive() == false) {
		if (gpsAlive == true) {
			gpsAlive = false;
			Serial.println("GPS module not responding with valid data.");
			Serial.println("Check wiring or restart.");
		}
	 }*/
}

