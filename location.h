#ifndef LOCATION_H
#define LOCATION_H

#import <FuGPS.h>

struct TelescopePosition {
	float altitude;
	float latitude;
	float longitude;
};


struct RaDecPosition {
	float rightAscension;
	float declination;
};

float days_since_j2k(int year);
int days_to_beginning_of_month(int year, int month);

float deg2rad(float degs);

float rad2deg(float rad);

void loadFromEEPROM();
void updateEEPROM(float altitude, float latitude, float longitude, int hours,
		int minutes, int seconds);
TelescopePosition& initGPS(FuGPS &gps);
TelescopePosition& handleGPS(FuGPS &gps);

#endif // LOCATION_H
