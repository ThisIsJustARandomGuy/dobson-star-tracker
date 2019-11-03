#pragma once

#include <FuGPS.h>

struct TelescopePosition {
	float altitude;
	float latitude;
	float longitude;
};


struct RaDecPosition {
	double rightAscension; // Azimuth in case of the DirectDrive
	double declination;    // Altitude in case of the DirectDrive
};

const double pi = 3.14159265358979324;

double green_sidereal_time(double jd_ut);
double get_local_sidereal_time(const double degrees_longitude);

void loadFromEEPROM();
void updateEEPROM(float altitude, float latitude, float longitude, int hours,
		int minutes, int seconds);
TelescopePosition& initGPS(FuGPS &gps);
TelescopePosition& handleGPS(FuGPS &gps);
