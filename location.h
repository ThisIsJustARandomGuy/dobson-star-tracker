#ifndef LOCATION_H
#define LOCATION_H

#import <FuGPS.h>

struct Position {
	float altitude;
	float latitude;
	float longitude;
	int hours;
	int minutes;
	int seconds;
};

void loadFromEEPROM();
void updateEEPROM(float altitude, float latitude, float longitude, int hours,
		int minutes, int seconds);
Position& initGPS(FuGPS &gps);
Position& handleGPS(FuGPS &gps);

#endif // LOCATION_H
