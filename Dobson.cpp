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

int createdNrs = 0;

void clamp360(double &value) {
	while (value < 0.0) {
		value += 360;
	}
	while(value >= 360.0) {
		value -= 360;
	}
}
void nclamp360(double& value) {
	while (value <= -360.0) {
		value += 360;
	}
	while (value >= 360.0) {
		value -= 360;
	}
}

Dobson::Dobson(AccelStepper &azimuthStepper, AccelStepper &altitudeStepper, FuGPS &gps) :
		_azimuthStepper(azimuthStepper), _altitudeStepper(altitudeStepper), _gps(gps) {
	createdNrs += 1;
}

void Dobson::initialize() {
	setMode(Mode::TRACKING);
	setTarget({ 36, 36 });
	
	//setTarget({ 0,0 });//250.43, 36.47 });
}

// Calculate new motor targets. This does not yet execute the move
void Dobson::calculateMotorTargets() {
	const float longitude = radians(_gpsPosition.longitude);
	const float latitude = radians(_gpsPosition.latitude);

	const float sinLongitude = sin(longitude);
	const float sinLatitude = sin(latitude);
	const float cosLatitude = cos(latitude);

	// Get the local sidereal time and store it in a private value (it gets referenced by the azAltToRaDec method)
	_currentLocalSiderealTime = get_local_sidereal_time(degrees(longitude));
	
	// The hour angle is the difference between the local sidereal timeand the right ascension in degrees of the target object.
	const double RadHourAngle = radians(get_hour_angle(_currentLocalSiderealTime, _target.rightAscension));

	_targetHourAngle = RadHourAngle;

	const double declination = radians(_target.declination);
	

	//double targetAltitudeDegrees = degrees(asin( SinTargetDeclination * SinGpsLatitude + CosTargetDeclination * CosGpsLatitude * CosHourAngle ));
	const double sinAltitude = sin(declination) * sinLatitude + cos(declination) * cosLatitude * cos(RadHourAngle);
	const double radAltitude = asin(sinAltitude);
	double targetAltitudeDegrees = degrees(radAltitude);

	const double cosAzimuth = (sin(declination) - sinLatitude * sinAltitude) / (cosLatitude * cos(radAltitude));
	const double radAzimuth = acos(cosAzimuth);
	double targetAzimuthDegrees = degrees(radAzimuth);

	//nclamp360(targetAltitudeDegrees);
	//nclamp360(targetAzimuthDegrees);
	if (sin(RadHourAngle) > 0) {
		targetAzimuthDegrees = 360 - targetAzimuthDegrees;
	}

	_targetRad = {
		radians(targetAzimuthDegrees),
		radAltitude
	};

	_targetDegrees = {
		targetAzimuthDegrees, // Azimuth
		targetAltitudeDegrees // Altitude
	};

	_steppersTarget = {
		static_cast<long>((targetAzimuthDegrees / 360.0)  * (long double)AZ_STEPS_PER_REV), // Azimuth
		static_cast<long>((targetAltitudeDegrees / 360.0) * (long double)ALT_STEPS_PER_REV) // Altitude
	};

	// Calculate and store the current position
	_currentPosition = azAltToRaDec({
		(_azimuthStepper.currentPosition() / ((long double)AZ_STEPS_PER_REV)) * 360.0,
		(_altitudeStepper.currentPosition() / ((long double)ALT_STEPS_PER_REV)) * 360.0
	});

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

	//double currentAzimuthDeg  = (_azimuthStepper.currentPosition() / (double)AZ_STEPS_PER_REV) * 360.0;
	//double currentAltitudeDeg = (_altitudeStepper.currentPosition() / (double)ALT_STEPS_PER_REV) * 360.0;
	//double currentAzimuthDeg  = (_steppersTarget.azimuth / (double)AZ_STEPS_PER_REV) * 360.0;
	//double currentAltitudeDeg = (_steppersTarget.altitude / (double)ALT_STEPS_PER_REV) * 360.0;

	if (false){//_ignoreMoves || !_isHomed) {
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

		/*DEBUG_PRINT("Stepper pos to Az / Alt ");
		DEBUG_PRINT(_steppersTarget.azimuth);
		DEBUG_PRINT(" / ");
		DEBUG_PRINTLN(_steppersTarget.altitude);*/
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
		diffAz = _steppersTarget.azimuth - _steppersLastTarget.azimuth;
		diffAlt = _steppersTarget.altitude - _steppersLastTarget.altitude;

		// Store the last target
		_steppersLastTarget.azimuth = _steppersTarget.azimuth;
		_steppersLastTarget.altitude = _steppersTarget.altitude;
	} else {
		// No move was performed
		_didMove = false;
	}
	_didMove = true;
	#ifdef DEBUG_SERIAL_STEPPER_MOVEMENT_VERBOSE
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
			DEBUG_PRINT(", TelescopeObjects: ");
			DEBUG_PRINT(createdNrs);
			DEBUG_PRINT("; RA ");
			DEBUG_PRINT(_target.rightAscension);
			DEBUG_PRINT(" and DEC ");
			DEBUG_PRINT(_target.declination);
			DEBUG_PRINT(" to ALTAZ is: ");
			DEBUG_PRINT(_targetDegrees.altitude);
			DEBUG_PRINT("° / ");
			DEBUG_PRINT(_targetDegrees.azimuth);
			DEBUG_PRINT("°; The time is: ");
			DEBUG_PRINT(day());
			DEBUG_PRINT(".");
			DEBUG_PRINT(month());
			DEBUG_PRINT(".");
			DEBUG_PRINT(year());
			DEBUG_PRINT(" at ");
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
	#elif defined DEBUG_SERIAL_STEPPER_MOVEMENT
		if (_didMove) {
			DEBUG_PRINTLN("-------------------------------------------------------------------------------------------------------------------");
			DEBUG_PRINT("Desired:     ");
			DEBUG_PRINT("LST ");
			DEBUG_PRINT(_currentLocalSiderealTime);

			DEBUG_PRINT("   Ra/Dec ");
			DEBUG_PRINT(_target.rightAscension);
			DEBUG_PRINT("° / ");
			DEBUG_PRINT(_target.declination);
			DEBUG_PRINT("°");

			const double degHourAngle = _targetHourAngle / 15;
			const int hrs = static_cast<int>(degHourAngle);
			const int mins = static_cast<int>((degHourAngle - hrs) * 60);
			const int secs = static_cast<int>((degHourAngle - (hrs + mins / 60.)) * 3600);
			DEBUG_PRINT("   HourAngle ");
			DEBUG_PRINT(hrs); DEBUG_PRINT(":"); DEBUG_PRINT(mins); DEBUG_PRINT(":"); DEBUG_PRINT(secs);
			DEBUG_PRINT(" / ");
			DEBUG_PRINT(_targetHourAngle);

			DEBUG_PRINT("   Az/Alt ");
			DEBUG_PRINT(_targetDegrees.azimuth);
			DEBUG_PRINT("° / ");
			DEBUG_PRINT(_targetDegrees.altitude);
			DEBUG_PRINT("°");

			DEBUG_PRINT("   Steps Az/Alt ");
			DEBUG_PRINT((long)_steppersTarget.azimuth);
			DEBUG_PRINT("° / ");
			DEBUG_PRINT((long)_steppersTarget.altitude);
			DEBUG_PRINT("°");

			DEBUG_PRINT("   Time: ");
			DEBUG_PRINT(day()); DEBUG_PRINT("."); DEBUG_PRINT(month()); DEBUG_PRINT("."); DEBUG_PRINT(year());
			DEBUG_PRINT(" at ");
			DEBUG_PRINT(hour()); DEBUG_PRINT(":"); DEBUG_PRINT(minute()); DEBUG_PRINT(":"); DEBUG_PRINT(second());
			DEBUG_PRINT("hrs");

			#ifndef DEBUG_SERIAL_TIMING
				DEBUG_PRINTLN("");
			#endif

			/*DEBUG_PRINT("   Steps Az/Alt ");
			DEBUG_PRINT(_steppersTarget.azimuth);
			DEBUG_PRINT(" / ");
			DEBUG_PRINT(_steppersTarget.altitude);

			DEBUG_PRINT("   Diff Az/Alt ");
			DEBUG_PRINT(diffAz);
			DEBUG_PRINT(" / ");
			DEBUG_PRINT(diffAlt);*/
		}
	#endif
}


/*
 * This method calculates the position in RA/DEC, based on the position given in the parameter "position"
 * The values in the "position" parameter are expected to be in degrees
 */
RaDecPosition Dobson::azAltToRaDec(AzAltPosition position) {
	const double radAzimuth = radians(position.azimuth);
	const double radAltitude = radians(position.altitude);

	const float longitude = radians(_gpsPosition.longitude);
	const float latitude = radians(_gpsPosition.latitude);

	const float sinLongitude = sin(longitude);
	const float sinLatitude = sin(latitude);
	const float cosLatitude = cos(latitude);

	const double calcSinDeclination = cos(radAzimuth) * cosLatitude * cos(radAltitude) + sinLatitude * sin(radAltitude);
	const double calcDeclination = asin(calcSinDeclination);


	const double cosRadHourAngle = (sin(radAltitude) - calcSinDeclination * sinLatitude) / (cos(calcDeclination) * cosLatitude);
	const double radHourAngle = acos(cosRadHourAngle);
	const double HourAngle = degrees(_targetHourAngle);

	//double radHourAngle = acos((sin(radAltitude) - sin(calcDeclination) * sinLatitude) / (cos(calcDeclination) * cosLatitude));

	//double HourAngle = degrees(radHourAngle);
	
	double RightAscension = _currentLocalSiderealTime - HourAngle;
	double Declination = degrees(calcDeclination);


	_calculatedHourAngle = radHourAngle;
	_calculatedRad = {
		radians(RightAscension),
		calcDeclination
	};


	//sin(Altitude) = sin(declination) * sin(latitude) + cos(declination) * cos(latitude) * cos(HourAngle);
	/*
	double Declination = asin((cos(Azimuth) * cos(latitude) * cos(Altitude)) + (sin(latitude) * sin(Altitude)));

	double HourAngle = acos((sin(Altitude) - sin(Declination) * sin(latitude)) / (cos(Declination) * cos(latitude)));
	
	Declination = degrees(Declination);
	HourAngle = degrees(HourAngle);*/

	//cos(Azimuth) = (sin(declination) - (sin(latitude) * sin(Altitude))) / (cos(latitude) * cos(Altitude));

	// Declination is kept in radians for now, but will be converted to degrees a few lines below
	/*double Declination = asin(sin(radians(position.altitude)) * sin(radians((double)(_gpsPosition.latitude))) + cos(radians(position.altitude)) * cos(radians((double)(_gpsPosition.latitude))) * cos(radians(position.azimuth)));
	
	// Hour angle in degrees
	//double HourAngle = degrees(asin(((0.0-sin(radians(position.azimuth))) * cos(radians(position.altitude))) / cos(Declination)));
	double HourAngle = (sin(radians(position.altitude)) - (sin(Declination) * sin(radians((double)(_gpsPosition.latitude))))) / (cos(Declination) *  cos(radians((double)(_gpsPosition.latitude))));*/

	// Convert Declination to degrees, since it's not needed in radians anymore
	//Declination = degrees(Declination);

	// The resulting Right Ascension is LST - HA

	// Ensure that RightAscension and Declination are between 0 and 360
	//clamp360(RightAscension);
	//nclamp360(Declination);

	// From here on only debug outputs happen in this method
	#ifdef DEBUG_SERIAL_POSITION_CALC
		if (_didMove) {
			DEBUG_PRINT("Calculated:  ");
			DEBUG_PRINT("LST ");
			DEBUG_PRINT(_currentLocalSiderealTime);

			DEBUG_PRINT("   Ra/Dec ");
			DEBUG_PRINT(RightAscension);
			DEBUG_PRINT("° / ");
			DEBUG_PRINT(Declination);
			DEBUG_PRINT("°");

			const double degHourAngle = _calculatedHourAngle / 15.;
			const int hrs = static_cast<int>(degHourAngle);
			const int mins = static_cast<int>((degHourAngle - hrs) * 60);
			const int secs = static_cast<int>((degHourAngle - (hrs + mins / 60.)) * 3600.);
			DEBUG_PRINT("   HourAngle ");
			DEBUG_PRINT(hrs); DEBUG_PRINT(":"); DEBUG_PRINT(mins); DEBUG_PRINT(":"); DEBUG_PRINT(secs);
			DEBUG_PRINT(" / ");
			DEBUG_PRINT(radHourAngle);

			DEBUG_PRINT("   Az/Alt ");
			DEBUG_PRINT(position.azimuth);
			DEBUG_PRINT(" °/ ");
			DEBUG_PRINT(position.altitude);
			DEBUG_PRINT("°");

			DEBUG_PRINT("   Steps Az/Alt ");
			DEBUG_PRINT(_azimuthStepper.currentPosition());
			DEBUG_PRINT("° / ");
			DEBUG_PRINT(_altitudeStepper.currentPosition());
			DEBUG_PRINTLN("°");
		}
	#endif

	return {
		RightAscension,
		Declination
	};
}