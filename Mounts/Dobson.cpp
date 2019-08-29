/*
 * Dobson.cpp
 *
 *  Created on: 27.08.2019
 *      Author: lukas
 */

#include <AccelStepper.h>
#include <FuGPS.h>
#include <Time.h>

#import "../config.h"
#include "../location.h"

#import "./Dobson.h"


Dobson::Dobson(AccelStepper &azimuthStepper, AccelStepper &altitudeStepper, FuGPS &gps) :
		_azimuthStepper(azimuthStepper), _altitudeStepper(altitudeStepper), _gps(gps) {
	// no code here?
}

// Calculate new motor targets. This does not yet execute the move
void Dobson::calculateMotorTargets() {
	long passed_seconds = (millis() * TIME_FACTOR) / 1000; // Seconds that have passed since execution started

	// Current time in UTC with hour rollover
	float current_utc = (hour() + TIMEZONE_CORRECTION_H) + (minute() / 60.0) + (second() / 3600.0);

	while (current_utc < 0) {
		current_utc += 24;
	}
	while (current_utc >= 24) {
		current_utc -= 24;
	}

	// Julian Days since 2000
	long jul_days_s2k = (((minute() / 60.0) + hour()) / 24.0) + days_to_beginning_of_month(year(), month()) + day()
			+ days_since_j2k(year());
	// END TIMEKEEPING

	// number of Julian centuaries since Jan 1, 2000, 12 UT
	const float jul_centuaries = jul_days_s2k / 36525;

	// calculate the local siderian time (in degrees)
	double local_siderian_time = 100.46 + 0.985647 * jul_days_s2k + _gpsPosition.longitude + 15 * current_utc;

	// Ensure that local_siderian_time is greater than 0 degrees
	while (local_siderian_time < 0) {
		local_siderian_time += 360;
	}

	// The hour angle is the difference between the local siderian time and the right ascension in degrees of our target object...
	float hour_angle = local_siderian_time - _target.rightAscension;
	// ...but we need to ensure that it's greater than 0
	while (hour_angle < 0) {
		hour_angle += 360.0; // Maybe this is buggy
	}

	// Convert various values from degrees to radians since the trigonometry functions work with radians
	const float rad_declination = radians(_target.declination);
	const float rad_current_lat = radians(_gpsPosition.latitude); // TODO this can be a constant if GPS_FIXED_POS is set
	const float rad_hour_angle = radians(hour_angle);

	// These values are used multiple times throughout calculations, so we pull them out
	const float sin_rad_declination = sin(rad_declination);
	const float sin_rad_current_lat = sin(rad_current_lat);
	const float cos_rad_current_lat = cos(rad_current_lat);

	// Calculate altitude
	const float sin_altitude = sin_rad_declination * sin_rad_current_lat
			+ cos(rad_declination) * cos_rad_current_lat * cos(rad_hour_angle);

	const float rad_altitude = asin(sin_altitude);

	// This is our resulting altitude in degrees
	_altitudeDegrees = degrees(rad_altitude); // RESULT

	// Calculate azimuth
	const float cos_azimuth = (sin_rad_declination - sin(rad_altitude) * sin_rad_current_lat)
			/ (cos(rad_altitude) * cos_rad_current_lat);
	const float azimuth_degrees = degrees(acos(cos_azimuth));

	// Azimuth in degrees is either 360-a or a, depending on whether sin(rad_hour_angle) is positive.
	// It be like it is and it's also the second part of the result
	// TODO This is buggy. When az=-57.3
	if (sin(rad_hour_angle) > 0) {
		_azimuthDegrees = 360.0 - azimuth_degrees;
	} else {
		_azimuthDegrees = azimuth_degrees;
	}

	// TODO Is it really necessary to have it this complex? No
	_steppersTarget = { (long) ((_azimuthDegrees / 360.0) * AZ_STEPS_PER_REV), (long) ((_altitudeDegrees / 360.0)
			* ALT_STEPS_PER_REV) };




#ifdef DEBUG
	_lastCalcMicros = micros();
#endif
}

void Dobson::move() {
	long micros_after_move = 0;
	if (!_isHomed) {
		DEBUG_PRINT("Just homed to ");
		DEBUG_PRINT(_steppersTarget.azimuth);
		DEBUG_PRINT(" / ");
		DEBUG_PRINTLN(_steppersTarget.altitude);

		// If not homed, or if homing was performed in this loop iteration just
		// set the current stepper position to the current target position without moving them
		_azimuthStepper.setCurrentPosition(_steppersTarget.azimuth);
		_altitudeStepper.setCurrentPosition(_steppersTarget.altitude);

		_steppersHomed = { _steppersTarget.azimuth, _steppersTarget.altitude };
		_steppersLastTarget = { _steppersTarget.azimuth, _steppersTarget.altitude };

			// Homing was performed
		_isHomed = true;
		_homedLastIteration = true;
	} else {
		_homedLastIteration = false;
		// Move the steppers to their target positions
		_azimuthStepper.moveTo(_steppersTarget.azimuth);
		_altitudeStepper.moveTo(_steppersTarget.altitude);

		// From here on only debug outputs happen
		if (_steppersLastTarget.azimuth - _steppersTarget.azimuth != 0
				|| _steppersLastTarget.altitude - _steppersTarget.altitude != 0) {
			micros_after_move = micros();

			DEBUG_PRINT_V(
					_gps.hasFix() ?
							"GPS: " + String(_gps.Satellites, 6) + "S/" + String(_gps.Quality) + "Q "
									+ String(_gps.Latitude, 6) + "LAT / " + String(_gps.Longitude, 6) + "LNG" :
							"GPS: N/A");
			DEBUG_PRINT("; LAT ");
			DEBUG_PRINT(_gpsPosition.latitude);
			DEBUG_PRINT(", LNG ");
			DEBUG_PRINT(_gpsPosition.longitude);
			DEBUG_PRINT("; RA ");
			DEBUG_PRINT(_target.rightAscension);
			DEBUG_PRINT(" and DEC ");
			DEBUG_PRINT(_target.declination);
			DEBUG_PRINT(" to ALTAZ is: AZ ");
			DEBUG_PRINT(_azimuthDegrees);
			DEBUG_PRINT("° ALT ");
			DEBUG_PRINT(_altitudeDegrees);
			DEBUG_PRINT("°; The time is: ");
			DEBUG_PRINT(hour());
			DEBUG_PRINT(":");
			DEBUG_PRINT(minute());
			DEBUG_PRINT(":");
			DEBUG_PRINT(second());
			DEBUG_PRINT("Steppers: az");
			DEBUG_PRINT(_steppersTarget.azimuth);
			DEBUG_PRINT("/dec ");
			DEBUG_PRINT(_steppersTarget.altitude);
			DEBUG_PRINT(" diff ");
			DEBUG_PRINT(_steppersTarget.azimuth - _steppersLastTarget.azimuth);
			DEBUG_PRINT(" / ");
			DEBUG_PRINT(_steppersTarget.altitude - _steppersLastTarget.altitude);
			DEBUG_PRINT("°; Reported: az");
			DEBUG_PRINT(_azimuthStepper.currentPosition());
			DEBUG_PRINT("/dec ");
			DEBUG_PRINT(_altitudeStepper.currentPosition());
		}
	}

	if (_steppersLastTarget.azimuth - _steppersTarget.azimuth != 0
			|| _steppersLastTarget.altitude - _steppersTarget.altitude != 0) {
		_steppersLastTarget.azimuth = _steppersTarget.azimuth;
		_steppersLastTarget.altitude = _steppersTarget.altitude;
		_didMove = true;
	} else {
		_didMove = false;
	}
}
