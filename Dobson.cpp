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
	// Get the local sidereal time and store it in a private value (it gets referenced by the azAltToRaDec method)
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
	const double targetAzimuthDegrees = degrees(acos((SinTargetDeclination - SinGpsLatitude * sin(radians(targetAltitudeDegrees))) / (CosGpsLatitude * cos(radians(targetAltitudeDegrees)))));

	_targetDegrees = {
		targetAzimuthDegrees, // Azimuth
		targetAltitudeDegrees // Altitude
	};

	_steppersTarget = {
		(_targetDegrees.azimuth / 360.0) * (double)AZ_STEPS_PER_REV,  // Azimuth
		(_targetDegrees.altitude / 360.0) * (double)ALT_STEPS_PER_REV // Altitude
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
	if ((micros() < 5000)) {
		DEBUG_PRINTLN("Ignore move for first 5 seconds");
		return;
	}

	double currentAzimuthDeg = (_azimuthStepper.currentPosition() / (double)AZ_STEPS_PER_REV) * 360.0;
	double currentAltitudeDeg = (_altitudeStepper.currentPosition() / (double)ALT_STEPS_PER_REV) * 360.0;
	
	// Caclulate and store the current position
	_currentPosition = azAltToRaDec({
		currentAzimuthDeg,
		currentAltitudeDeg
	});


	if (_ignoreMoves || !_isHomed) {
		// If not homed, or if homing was performed in this loop iteration just
		// set the current stepper position to the current target position without moving them
		_azimuthStepper.setCurrentPosition(_steppersTarget.azimuth);
		_altitudeStepper.setCurrentPosition(_steppersTarget.altitude);

		// Store where the motors targeted before this operation, in case we need to move back to the original position.
		_steppersHomed = { _steppersTarget.azimuth, _steppersTarget.altitude };
		// Store the last target for later comparisons.
		_steppersLastTarget = { _steppersTarget.azimuth, _steppersTarget.altitude };

		// Homing was performed in this iteration. In the next loop iteration this value can be used, but then it gets set to false again
		_ignoredMoveLastIteration = true;

		DEBUG_PRINT("Stepper pos to Az / Alt ");
		DEBUG_PRINT(_steppersTarget.azimuth);
		DEBUG_PRINT(" / ");
		DEBUG_PRINTLN(_steppersTarget.altitude);
	} else {
		_ignoredMoveLastIteration = false;
		// Move the steppers to their target positions
		_azimuthStepper.moveTo(_steppersTarget.azimuth);
		_altitudeStepper.moveTo(_steppersTarget.altitude);
	}

	// These store the difference between the last motor target and the current one and only serve debug purposes for now
	long diffAz = 0;
	long diffAlt = 0;

	// If a move was performed, store the last target value and set _didMove to true to indicate that a move took place this loop iteration
	if (_steppersLastTarget.azimuth - _steppersTarget.azimuth != 0.0
			|| _steppersLastTarget.altitude - _steppersTarget.altitude != 0.0) {
		// A move was performed
		_didMove = true;

		// Difference between the last target and the current one
		diffAz = _steppersLastTarget.azimuth - _steppersTarget.azimuth;
		diffAlt = _steppersLastTarget.altitude - _steppersLastTarget.altitude;

		// Store the last target
		_steppersLastTarget.azimuth = _steppersTarget.azimuth;
		_steppersLastTarget.altitude = _steppersTarget.altitude;
	} else {
		// No move was performed
		_didMove = false;
	}

	#ifdef DEBUG_SERIAL_STEPPER_MOVEMENT
		if (_didMove) {
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
			DEBUG_PRINT(diffAz);
			DEBUG_PRINT(" / ");
			DEBUG_PRINT(diffAlt);
			DEBUG_PRINT("°; Reported: az");
			DEBUG_PRINT(_azimuthStepper.currentPosition());
			DEBUG_PRINT("/dec ");
			DEBUG_PRINT(_altitudeStepper.currentPosition());
		}
	#endif
}


/*
 * This method calculates the position in RA/DEC, based on the position given in the parameter "position"
 * // TODO Document this a bit better
 */
RaDecPosition Dobson::azAltToRaDec(AzAltPosition position) {
	// Calculate Altitude in Degrees
	double Altitude = position.altitude;

	// Ensure that Altitude is between 0 and 360
	while (Altitude < 0.0) {
		Altitude += 360.;
	}
	while (Altitude >= 360.0) {
		Altitude -= 360.;
	}

	// Azimuth in Degrees
	double Azimuth = position.azimuth;

	// Ensure that Azimuth is between 0 and 360
	while (Azimuth < 0.0) {
		Azimuth += 360.0;
	}
	while (Azimuth >= 360.0) {
		Azimuth -= 360.0;
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

	// Declination is kept in radians for now, but will be converted to degrees a few lines below
	double Declination = (asin(SinAltitude * SinGpsLatitude + cos(Altitude) * CosGpsLatitude * cos(Azimuth)));
	// Hour angle in degrees
	double HourAngle = degrees(asin(((0.0-sin(Azimuth)) * cos(Altitude)) / cos(Declination)));

	// The resulting Right Ascension is LST - HA
	double RightAscension = _currentLocalSiderealTime - HourAngle;

	// We can also convert Declination to degrees, since it's not needed in calculations anymore
	Declination = degrees(Declination);

	// Ensure that RightAscension and Declination are between 0 and 360
	while (RightAscension < 0.0) {
		RightAscension += 360.0;
	}
	while (RightAscension >= 360.0) {
		RightAscension -= 360.0;
	}
	while (Declination < 0.0) {
		Declination += 360.0;
	}
	while (Declination >= 360.0) {
		Declination -= 360.0;
	}

	// This stores the current position so that it can get reported correctly.
	RaDecPosition result = {
		RightAscension,
		Declination
	};


	// From here on only debug outputs happen in this method
	#ifdef DEBUG_SERIAL_POSITION_CALC
		if (_didMove) {
			DEBUG_PRINT("Altitude: ");
			DEBUG_PRINT(_altitudeStepper.currentPosition());
			DEBUG_PRINT("; Azimuth: ");
			DEBUG_PRINT(_azimuthStepper.currentPosition());

			DEBUG_PRINT("; RightAscension: ");
			DEBUG_PRINT(RightAscension);
			DEBUG_PRINT("; Declination: ");
			DEBUG_PRINTLN(Declination);
		}
	#endif

	return result;
}