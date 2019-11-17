#include <AccelStepper.h>
#include <FuGPS.h>
#include <Time.h>

#include "./config.h"
#include "./location.h"
#include "./Observer.h"

#include "./Dobson.h"

// Adds or subtracts 360 from a value until: 0<=value<360
void clamp360(double &value) {
	while (value < 0.0) {
		value += 360;
	}
	while(value >= 360.0) {
		value -= 360;
	}
}
// Adds or subtracts 360 from a value until: -360<=value<360
void nclamp360(double& value) {
	while (value <= -360.0) {
		value += 360;
	}
	while (value >= 360.0) {
		value -= 360;
	}
}


Dobson::Dobson(AccelStepper &azimuthStepper, AccelStepper &altitudeStepper, Observer &observer) :
		_azimuthStepper(azimuthStepper), _altitudeStepper(altitudeStepper), _observer(observer) {
}

void Dobson::initialize() {
	setMode(Mode::TRACKING);
	setTarget({
		250.425, //15. * (22. + (59.8 / 60.)),
		36.466667//42. + (43. / 60.)
	});
	calculateMotorTargets();
}

/*
 * Calculates motor target angles by converting from Right Ascension and Declination to Azimuth and Altitude.
 * No movement of the motors is performed in this method (see Dobson::move() for that part)
 * The basic formulas are as follows:
 * Altitude = asin( sin(dec) * sin(lat) + cos(dec) * cos(lat) * cos(HourAngle) )
 * Azimuth  = acos( (sin(dec) - sin(lat) * sin(alt)) / (cos(lat) * cos(Altitude)) )
 * If sin(HourAngle) > 0 then Azimuth = 360 - Azimuth
 */
void Dobson::calculateMotorTargets() {
	const float longitude = _observer.longitude();
	const float latitude = _observer.latitude();
	_currentLocalSiderealTime = get_local_sidereal_time(longitude);

	const double HA = _currentLocalSiderealTime - _target.rightAscension;

	const double Altitude = degrees(asin( sin(radians(_target.declination)) * sin(radians(latitude)) + cos(radians(_target.declination)) * cos(radians(latitude)) * cos(radians(HA)) ));
	const double A = degrees(acos((sin(radians(_target.declination)) - sin(radians(latitude)) * sin(radians(Altitude))) / (cos(radians(latitude)) * cos(radians(Altitude)))));

	double Azimuth = sin(radians(HA)) > 0 ? 360. - A : A;
	clamp360(Azimuth);

	_targetDegrees = { Azimuth, Altitude };

	_steppersTarget = {
		(long)(Azimuth * AZ_STEPS_PER_DEG),
		(long)(Altitude * ALT_STEPS_PER_DEG)
	};

	if (_steppersTarget.azimuth != _steppersLastTarget.azimuth
		|| _steppersTarget.altitude != _steppersLastTarget.altitude) {
		// Indicate that the motor targets have changed from the last time this was called
		// move() then uses the difference between _steppersTarget and _steppersLastTarget
		// for debug outputs. The value of _steppersLastTarget is then set to _steppersTarget, so that the comparison
		// can be done again the next time calculateMotorTargets() is called
		_didMove = true;
	}
	else {
		_didMove = false;
	}

	// Azimuth / Altitude to RightAscension / Declination and store the result
	_currentPosition = azAltToRaDec({
		_azimuthStepper.currentPosition() / AZ_STEPS_PER_DEG,
		_altitudeStepper.currentPosition() / ALT_STEPS_PER_DEG
	});

	#ifdef DEBUG_TIMING
		_lastCalcMicros = micros();
	#endif
}


/*
 * This method gets executed every 10.000 loop iterations right after Dobson::calculateMotorTargets() was called.
 * It checks whether the telescope is homed. If it is NOT homed, it sets the target as its current motor positions and sets _isHomed to true.
 * The next time the method gets called, _isHomed is true , and the stepper motors are actually moved to their new required position.
 */
void Dobson::move() {
		// TODO Finally find a relaible way to start the scope. 
	if (millis() < 3000) {//_ignoreMoves || !_isHomed) {
		// If not homed, or if homing was performed in this loop iteration just
		// set the current stepper position to the current target position without moving them
		_azimuthStepper.setCurrentPosition(_steppersTarget.azimuth);
		_altitudeStepper.setCurrentPosition(_steppersTarget.altitude);

		// Store where the motors targeted before this operation, in case we need to move back to the original position.
		_steppersHomed = { _steppersTarget.azimuth, _steppersTarget.altitude };
		// Set the last position to the current one, because there is no actual previous target
		_steppersLastTarget = { _steppersTarget.azimuth, _steppersTarget.altitude };

		// Homing was performed in this iteration. In the next loop iteration this value can be used, but then it gets set to false again
		_ignoredMoveLastIteration = true;
	} else {
		_ignoredMoveLastIteration = false;
		// Move the steppers to their target positions
		_azimuthStepper.moveTo(_steppersTarget.azimuth);
		_altitudeStepper.moveTo(_steppersTarget.altitude);
	}

	// Has the motor position changed since the last time move() was called? If so,
	// store the current target value
	if (_didMove) {

		// Difference between the last target and the current one
		debugMove(
			_steppersTarget.azimuth - _steppersLastTarget.azimuth,
			_steppersTarget.altitude - _steppersLastTarget.altitude
		);

		// Store the current position
		_steppersLastTarget = _steppersTarget;
	}
}


/*
 * This method calculates the position in RA/DEC, based on the position given in the parameter "position"
 * The values in the "position" parameter are expected to be in degrees
 * Declination = asin( sin(alt) * sin(lat) + cos(alt) * cos(lat) * cos(az) )
 * HourAngle = acos( (sin(alt) - sin(lat) * sin(dec)) / (cos(lat) * cos(dec)) )
 * If sin(az) > 0 then HourAngle = 360 - HourAngle
 * RightAscension = LST - HourAngle
 */
RaDecPosition Dobson::azAltToRaDec(AzAlt<double> position) {
	const double latitude = radians(_observer.latitude());

	const double sinLAT = sin(latitude);
	const double cosLAT = cos(latitude);

	
	const double radAZ = radians(position.azimuth);
	const double radALT = radians(position.altitude);

	const double cosAZ = cos(radAZ);
	const double sinAZ = sin(radAZ);

	const double cosALT = cos(radALT);
	const double sinALT = sin(radALT);

	const double sinDEC = sinALT * sinLAT + cosALT * cosLAT * cosAZ;
	const double radDEC = asin(sinDEC);
	const double Declination = degrees(radDEC);
	
	const double ha1 = degrees(acos(((sinALT - sinLAT * sinDEC) / (cosLAT * cos(radDEC)))));
	const double HourAngle = sinAZ > 0 ? 360. - ha1 : ha1;

	double RightAscension = _currentLocalSiderealTime - HourAngle;
	clamp360(RightAscension);
	
	return {
		RightAscension,
		Declination
	};
}


/*
 * This jsut prints various debug statements so that they do not have to happen in calculateMotorTargets(), move(), and azAltToRaDec()
*/
void Dobson::debugMove(long diffAz, long diffAlt) {

	// Debug statements by calculateMotorTargets()
	#ifdef DEBUG_SERIAL_STEPPER_MOVEMENT_VERBOSE
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
	#elif defined DEBUG_SERIAL_STEPPER_MOVEMENT
		DEBUG_PRINTLN("-------------------------------------------------------------------------------------------------------------------");
		DEBUG_PRINT("Desired:     ");

		DEBUG_PRINT("   Ra/Dec ");
		DEBUG_PRINT(_target.rightAscension);
		DEBUG_PRINT("° / ");
		DEBUG_PRINT(_target.declination);
		DEBUG_PRINT("°");

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

		DEBUG_PRINT("   LST ");
		DEBUG_PRINT(_currentLocalSiderealTime);

		#ifndef DEBUG_TIMING
			DEBUG_PRINTLN("");
		#endif
	#endif // End statements by calculateMotorTargets()

	// Debug statements by move()

	// End statements by move()


	// Debug statements by azAltToRaDec()


	// From here on only debug outputs happen in this method
	#ifdef DEBUG_SERIAL_POSITION_CALC
		DEBUG_PRINT("Calculated:  ");

		DEBUG_PRINT("   Ra/Dec ");
		DEBUG_PRINT(_currentPosition.rightAscension);
		DEBUG_PRINT("° / ");
		DEBUG_PRINT(_currentPosition.declination);
		DEBUG_PRINT("°");

		DEBUG_PRINT("   Az/Alt ");
		DEBUG_PRINT(_azimuthStepper.currentPosition() / AZ_STEPS_PER_DEG);
		DEBUG_PRINT("° / ");
		DEBUG_PRINT(_altitudeStepper.currentPosition() / ALT_STEPS_PER_DEG);
		DEBUG_PRINTLN("°");
	#endif
	// End statements by azAltToRaDec()
}