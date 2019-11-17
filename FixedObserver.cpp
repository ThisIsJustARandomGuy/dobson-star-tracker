#include "FixedObserver.h"

#include <Time.h>


FixedObserver::FixedObserver(const float altitude, const float latitude, const float longitude, const int year, const int month, const int day, const int hour, const int minute, const int second) {
	// Initialize the position
	setPosition({ altitude, latitude, longitude });
	setTime(hour, minute, second, day, month, year);
}

void FixedObserver::initialize() {
	// Nothing to do
}

void FixedObserver::updatePosition() {
	// Nothing to do here
}

bool FixedObserver::hasValidPosition() {
	// The position is always valid
	return true;
}

void FixedObserver::printDebugInfo() {
	Serial.println("Fixed position used (GPS_FIXED_POS)");
	Serial.print("Altitude   ... ");
	Serial.println(String(altitude(), 6));
	Serial.print("Latitude   ... ");
	Serial.println(String(latitude(), 6));
	Serial.print("Longitude  ... ");
	Serial.println(String(longitude(), 6));
}