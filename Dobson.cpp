/*
 * Dobson.cpp
 *
 *  Created on: 27.08.2019
 *      Author: lukas
 */

#include <AccelStepper.h>
#include <FuGPS.h>
#include <Time.h>

#import "./config.h"
#include "./location.h"

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
		current_utc += 24.;
	}
	while (current_utc >= 24) {
		current_utc -= 24.;
	}

	// Julian Days since 2000
	// TODO What happens to this, if current_utc gets modified by one of the loops above? day() would stay the same
	double jul_days_s2k = (((second() / 3600.0) + (minute() / 60.0) + hour()) / 24.0) + days_to_beginning_of_month(year(), month()) + day() + days_since_j2k(year());
	// END TIMEKEEPING

	// number of Julian centuaries since Jan 1, 2000, 12 UT
	const double jul_centuaries = jul_days_s2k / 36525.;

	// calculate the local siderian time (in degrees)
	double local_siderian_time = 100.46061837 + (15. * 0.06570982441908) * jul_days_s2k + _gpsPosition.longitude + (15. * 1.00273790935) * current_utc + (15. * 0.000026 * jul_centuaries * jul_centuaries);

	// Ensure that local_siderian_time is greater than 0 degrees
	while (local_siderian_time < 0) {
		local_siderian_time += 360.0;
	}
	while (local_siderian_time >= 0) {
		local_siderian_time -= 360.0;
	}

	// The hour angle is the difference between the local siderian time and the right ascension in degrees of our target object...
	double hour_angle = local_siderian_time - _target.rightAscension;
	// ...but we need to ensure that it's greater than 0
	while (hour_angle < 0) {
		hour_angle += 360.0; // Maybe this is buggy
	}
	while (hour_angle >= 360) {
		hour_angle -= 360.0; // Maybe this is buggy
	}


	// Trust me, this works
	_altitudeDegrees = degrees(asin(sin(radians(_target.declination)) * sin(radians(_gpsPosition.latitude)) + cos(radians(_target.declination)) * cos(degrees(_gpsPosition.latitude)) * cos(radians(hour_angle))));
	_azimuthDegrees = degrees( acos(( sin(radians(_target.declination)) - sin(radians(_gpsPosition.latitude)) * sin(radians(_altitudeDegrees)) ) / (cos(radians(_gpsPosition.latitude)) * cos(radians(_altitudeDegrees))) ));

	_steppersTarget = {
		(long)((_azimuthDegrees / 360.0) * AZ_STEPS_PER_REV), // Azimuth
		(long)((_altitudeDegrees / 360.0) * ALT_STEPS_PER_REV) // Altitude
	};

	return;


	// Convert various values from degrees to radians since the trigonometry functions work with radians
	const double rad_declination = radians(_target.declination);
	const double rad_current_lat = radians(_gpsPosition.latitude); // TODO this can be a constant if GPS_FIXED_POS is set
	const double rad_hour_angle = radians(hour_angle);

	// These values are used multiple times throughout calculations, so we pull them out
	const double sin_rad_declination = sin(rad_declination);
	const double sin_rad_current_lat = sin(rad_current_lat);
	const double cos_rad_current_lat = cos(rad_current_lat);

	// Calculate altitude
	const double sin_altitude = sin_rad_declination * sin_rad_current_lat
			+ cos(rad_declination) * cos_rad_current_lat * cos(rad_hour_angle);

	const double rad_altitude = asin(sin_altitude);

	// This is our resulting altitude in degrees
	_altitudeDegrees = degrees(rad_altitude); // RESULT

	// Calculate azimuth
	const double cos_azimuth = (sin_rad_declination - sin(rad_altitude) * sin_rad_current_lat)
			/ (cos(rad_altitude) * cos_rad_current_lat);
	const double azimuth_degrees = degrees(acos(cos_azimuth));

	// Azimuth in degrees is either 360-a or a, depending on whether sin(rad_hour_angle) is positive.
	// It be like it is and it's also the second part of the result
	// TODO This is buggy. When az=-57.3
	/*if (sin(rad_hour_angle) > 0) {
		_azimuthDegrees = 360.0 - azimuth_degrees;
	} else {*/
		_azimuthDegrees = azimuth_degrees;
	//}

	// TODO Is it really necessary to have it this complex? No
	_steppersTarget = { 
		(long) ((_azimuthDegrees / 360.0) * AZ_STEPS_PER_REV), // Azimuth
		(long) ((_altitudeDegrees / 360.0) * ALT_STEPS_PER_REV) // Altitude
	};


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

		// Caclulate a new interpolated position
		interpolatePosition();

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


// Calculate new motor targets. This does not yet execute the move
void Dobson::interpolatePosition() {
	long passed_seconds = (millis() * TIME_FACTOR) / 1000; // Seconds that have passed since execution started

	// Current time in UTC with hour rollover
	double current_utc = (hour() + TIMEZONE_CORRECTION_H) + (minute() / 60.0) + (second() / 3600.0);

	while (current_utc < 0) {
		current_utc += 24;
	}
	while (current_utc >= 24) {
		current_utc -= 24;
	}

	// Julian Days since 2000
	// TODO What happens to this, if current_utc gets modified by one of the loops above? day() would stay the same
	double jul_days_s2k = (((second() / 3600.0) + (minute() / 60.0) + hour()) / 24.0) + days_to_beginning_of_month(year(), month()) + day()
		+ days_since_j2k(year());
	// END TIMEKEEPING

	// number of Julian centuaries since Jan 1, 2000, 12 UT
	const double jul_centuaries = jul_days_s2k / 36525.;

	// calculate the local siderian time (in degrees)
	double local_siderian_time = 100.46061837 + (15. * 0.06570982441908) * jul_days_s2k + _gpsPosition.longitude + (15. * 1.00273790935) * current_utc + (15. * 0.000026 * jul_centuaries * jul_centuaries);

	// Ensure that local_siderian_time is greater than 0 degrees
	while (local_siderian_time < 0.) {
		local_siderian_time += 360.;
	}
	while (local_siderian_time >= 360.) {
		local_siderian_time -= 360.;
	}

	// Calculate Altitude in Degrees
	double Altitude = _altitudeDegrees;
	while (Altitude < 0) {
		Altitude += 360.;
	}
	while (Altitude >= 360.) {
		Altitude -= 360.;
	}

	// Calculate Azimuth in Degrees
	double Azimuth = _azimuthDegrees;
	while (Azimuth < 0.) {
		Azimuth += 360.;
	}
	while (Azimuth >= 360.) {
		Azimuth -= 360.;
	}
	
	DEBUG_PRINT("Altitude: ");
	DEBUG_PRINT(Altitude);
	DEBUG_PRINT("; Azimuth: ");
	DEBUG_PRINT(Azimuth);

	// Convert Altitude and Azimuth to Radians
	Altitude = radians(Altitude);
	Azimuth = radians(Azimuth);


	double Declination = degrees(asin(sin(Altitude) * sin(radians(_gpsPosition.latitude)) + cos(Altitude) * cos(radians(_gpsPosition.latitude)) * cos(Azimuth)));
	//double HourAngle = asin(-sin(Azimuth) * cos(Altitude) / cos(Declination));
	double HourAngle = degrees(acos((sin(Altitude) - sin(Declination) * sin(radians(_gpsPosition.latitude))) / (cos(Declination) * cos(radians(_gpsPosition.latitude)))));
	double RightAscension = local_siderian_time - HourAngle;

	DEBUG_PRINT("; RightAscension: ");
	DEBUG_PRINT(String(RightAscension, 2));
	DEBUG_PRINT("; Declination: ");
	DEBUG_PRINTLN(String(Declination, 2));
	
	_currentPosition = {
		RightAscension,
		Declination
	};

	return;
}