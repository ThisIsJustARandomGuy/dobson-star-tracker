/*
 * Dobson.cpp
 *
 *  Created on: 27.08.2019
 *      Author: lukas
 */

#include <AccelStepper.h>
#include <FuGPS.h>
#include <Time.h>

#include "./config.h"
#include "./location.h"

#include "./Dobson.h"


Dobson::Dobson(AccelStepper &azimuthStepper, AccelStepper &altitudeStepper, FuGPS &gps) :
		_azimuthStepper(azimuthStepper), _altitudeStepper(altitudeStepper), _gps(gps) {
	// no code here?
}

// Calculate new motor targets. This does not yet execute the move
void Dobson::calculateMotorTargets() {
	long passed_seconds = (millis() * TIME_FACTOR) / 1000; // Seconds that have passed since execution started

	// Get the local sidereal time and store it in a private value (it gets referenced by the interpolatePosition method)
	_currentLocalSiderealTime = get_local_sidereal_time(_gpsPosition.longitude);
	
	// The hour angle is the difference between the local sidereal timeand the right ascension in degrees of the target object.
	// This immediately calculates this cosine, because that's all we need later on.
	const double CosHourAngle = cos(radians(get_hour_angle(_currentLocalSiderealTime, _target.rightAscension)));

	// Target declination in radians
	const double RadTargetDeclination = radians(_target.declination);
	// Cosine of the target declination
	const double CosTargetDeclination = cos(RadTargetDeclination);
	// Sine of the target declination
	const double SinTargetDeclination = sin(RadTargetDeclination);

	// Current latitude in radians
	const double RadGpsLatitude = radians(_gpsPosition.latitude);
	// Cosine of the current latitude
	const double CosGpsLatitude = cos(RadGpsLatitude);
	// Sine of the current latitude
	const double SinGpsLatitude = sin(RadGpsLatitude);

	// Trust me, this works
	const double targetAltitudeDegrees = degrees(asin( SinTargetDeclination * SinGpsLatitude + CosTargetDeclination * CosGpsLatitude * CosHourAngle ));

	_targetDegrees = {
		degrees( acos( ( SinTargetDeclination - SinGpsLatitude * sin(radians(targetAltitudeDegrees)) ) / (CosGpsLatitude * cos(radians(targetAltitudeDegrees))) ) ), // Azimuth
		targetAltitudeDegrees // Altitude
	};

	_steppersTarget = {
		(long)((_targetDegrees.azimuth / 360.0) * AZ_STEPS_PER_REV), // Azimuth
		(long)((_targetDegrees.altitude / 360.0) * ALT_STEPS_PER_REV) // Altitude
	};

#ifdef DEBUG
	_lastCalcMicros = micros();
#endif
}


/*
 * This method gets executed every 10.000 loop iterations right after Dobson::calculateMotorTargets() was called.
 * It checks whether the telescope is homed. If it is NOT homed, it sets the target as its current motor positions and sets _isHomed to true.
 * The next time the method gets called, _isHomed is true , and the stepper motors are actually moved to their new required position.
 */
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

		// Store where the motors were homed, in case we need to move back to the original position.
		_steppersHomed = { _steppersTarget.azimuth, _steppersTarget.altitude };
		// Store the last target for later comparisons.
		_steppersLastTarget = { _steppersTarget.azimuth, _steppersTarget.altitude };

		// Homing was performed
		_isHomed = true;
		// Homing was performed in this iteration. In the next loop iteration this value can be used, but then it gets set to false again
		_homedLastIteration = true;
	} else {
		_homedLastIteration = false;
		// Move the steppers to their target positions
		_azimuthStepper.moveTo(_steppersTarget.azimuth);
		_altitudeStepper.moveTo(_steppersTarget.altitude);

		// Caclulate the new RA/DEC position
		interpolatePosition();

		// This if statement is wholly for debug debug reasons. There is more code at the end of the method
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
			DEBUG_PRINT(_targetDegrees.azimuth);
			DEBUG_PRINT("° ALT ");
			DEBUG_PRINT(_targetDegrees.altitude);
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
		} // End of debug if statement
	}

	// If a move was performed, store the last target value and set _didMove to true to indicate that a move took place this loop iteration
	if (_steppersLastTarget.azimuth - _steppersTarget.azimuth != 0
			|| _steppersLastTarget.altitude - _steppersTarget.altitude != 0) {
		_steppersLastTarget.azimuth = _steppersTarget.azimuth;
		_steppersLastTarget.altitude = _steppersTarget.altitude;
		_didMove = true;
	} else {
		_didMove = false;
	}
}


/*
 * This method calculates the current position in RA/DEC, based on the position of the stepper motors.
 * // TODO Document this a bit better
 */
void Dobson::interpolatePosition() {
	long passed_seconds = (millis() * TIME_FACTOR) / 1000; // Seconds that have passed since execution started

	// Calculate Altitude in Degrees
	const double altsteps = ALT_STEPS_PER_REV;
	double Altitude = (_altitudeStepper.currentPosition() / ALT_STEPS_PER_REV) * 360.;

	while (Altitude < 0) {
		Altitude += 360.;
	}
	while (Altitude >= 360.) {
		Altitude -= 360.;
	}

	// Azimuth in Degrees
	double Azimuth = (_azimuthStepper.currentPosition() / AZ_STEPS_PER_REV) * 360.;

	// Ensure that it is 
	while (Azimuth < 0.) {
		Azimuth += 360.;
	}
	while (Azimuth >= 360.) {
		Azimuth -= 360.;
	}

	// Convert Altitude and Azimuth to Radians
	Altitude = radians(Altitude);
	Azimuth = radians(Azimuth);

	// Store some values that are needed later on

	// Sine of the current altitude
	const double SinAltitude = sin(Altitude);

	// Current latitude in radians
	const double RadGpsLatitude = radians(_gpsPosition.latitude);
	// Cosine of the current latitude
	const double CosGpsLatitude = cos(RadGpsLatitude);
	// Sine of the current latitude
	const double SinGpsLatitude = sin(RadGpsLatitude);

	// These are the important calculations. 
	double Declination = degrees(asin(SinAltitude * SinGpsLatitude + cos(Altitude) * CosGpsLatitude * cos(Azimuth)));
	//double HourAngle = asin(-sin(Azimuth) * cos(Altitude) / cos(Declination));
	double HourAngle = degrees(acos((SinAltitude - sin(Declination) * SinGpsLatitude) / (cos(Declination) * CosGpsLatitude)));
	double RightAscension = _currentLocalSiderealTime - HourAngle;

	// This stores the current position so that it can get reported correctly.
	_currentPosition = {
		RightAscension,
		Declination
	};

	// From here on only debug outputs happen in this method
	DEBUG_PRINT("Altitude: ");
	DEBUG_PRINT(degrees(Altitude));
	DEBUG_PRINT("; Azimuth: ");
	DEBUG_PRINT(degrees(Azimuth));

	DEBUG_PRINT("; RightAscension: ");
	DEBUG_PRINT(String(RightAscension, 2));
	DEBUG_PRINT("; Declination: ");
	DEBUG_PRINTLN(String(Declination, 2));
}