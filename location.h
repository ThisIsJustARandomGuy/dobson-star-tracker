#pragma once

#include <FuGPS.h>

// altitude, latitude, longitude
struct ObserverPosition {
	double altitude;
	double latitude;
	double longitude;
};

struct RaDecPosition {
	double rightAscension; // Azimuth in case of the DirectDrive
	double declination;    // Altitude in case of the DirectDrive
};

double get_local_sidereal_time(const double degrees_longitude);

// TODO These should not be in here, but in a separate file
void loadFromEEPROM();
void updateEEPROM(float altitude, float latitude, float longitude, int hours,
		int minutes, int seconds);