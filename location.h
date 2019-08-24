#ifndef LOCATION_H
#define LOCATION_H

#import <FuGPS.h>

struct Position {
	float altitude;
	float latitude;
	float longitude;
};

void loadFromEEPROM();
void updateEEPROM(float altitude, float latitude, float longitude);
Position& initGPS(FuGPS &gps);
Position& handleGPS(FuGPS &gps);

#endif // LOCATION_H
