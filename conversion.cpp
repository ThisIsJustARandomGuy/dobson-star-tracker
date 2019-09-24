/*
 * Conversion.cpp
 *
 *  Created on: 03.08.2019
 *      Author: lukas
 */
#include <Arduino.h>
#include <AccelStepper.h>
#include <TimeLib.h>

#include "./config.h"
#include "./conversion.h"
#include "./location.h"


// These are the coordinates that the telescope initially thinks it's pointed at.
// 
const double ra_h = 16.0;
const double ra_m = 41.7;
double ra_deg = (ra_h + ra_m / 60.0) * 15.0;

const double dec_d = 36.0;
const double dec_m = 28.0;
double dec_deg = dec_d + dec_m / 60.0;

boolean isPositiveDeclination = false;

boolean isHomed = false;

char txAR[10]; // Gets reported to stellarium when it asks for right ascension. Example: "16:41:34#"
char txDEC[11]; // Same as above with declination. Example: "+36d%c28:%02d#"

const byte numChars = 32;
char receivedChars[numChars];

bool newData = false; // Gets set to true whenever a complete command is buffered
static boolean recvInProgress = false; // True while a command is being received
static byte ndx = 0; // Number of command character received
const char startMarker = ':'; // Commands begin with this character
const char endMarker = '#'; // Commands end with this character

// These are the positions that can be cycled through with the position switch
const double debugPositions[][2] = {
	{ ra_deg, dec_deg},
	{ 279.2354166, 38.78555556 }, // Vega
	{ 213.9083333 , 19.17038889 },    // Arktur
	{ 37.9624166 , 89.2642777 },    // Polaris
	{ 15.7, 36.5 },
	{ 14.7, 36.5 },
	{ 17.7, 36.5 },
	{ 18.7, 36.5 },
	{ 19.7, 36.5 },
	{ 17.7, 38.5 },
	{ 17.7, 39.5 }
};

const char* positionNames[] = {
	"Start",
	"Polaris",
	"Vega",
	"Arktur",
	"Pos 4",
	"Pos 5",
	"Pos 6",
	"Pos 7",
	"Pos 8",
	"Pos 9"
};

const int maxDebugPos = sizeof(debugPositions) / (sizeof(debugPositions[0]));

void initCommunication() {
	sprintf(txAR, "%02d:%02d:%02d#", int(ra_h), int(ra_m), int(0));
	sprintf(txDEC, "%c%02d%c%02d:%02d#", dec_d > 0 ? '+' : '-', int(dec_d), 223, int(dec_m), int(0));
}

/*
 * Every loop iteration this function checks if a new character is available from the Serial interface. It only reads the
 * character if the variable newData is set to false. It then tries to read a complete command (1 char per loop iteration).
 * When a command is complete, this function sets newData to true. Later on in the loop iteration the parseCommands function,
 * which tries to parse and execute the command, gets called. After command execution the newData variable is reset to false
 */
void receiveCommandChar() {
	if (Serial.available() > 0 && newData == false) {
		// Read the next character
		char rc = Serial.read();

		// Here we check if the command start marker has already been read in a previous loop iteration.
		if (recvInProgress == true) {
			// If the character is NOT the end marker, we treat it as part of the command, regardless of what it is
			if (rc != endMarker) {
				// Here we store the next character in the receivedChars array.
				receivedChars[ndx] = rc;
				ndx++;
				// This prevents the receivedChars array from overflowing
				if (ndx >= numChars) {
					ndx = numChars - 1;
				}
			} else {
				// End marker received, so the command is completely stored in receivedChars
				// Set the last value to \0 to terminate the string
				receivedChars[ndx] = '\0';
				// Set recvInProgress to false, so that next time a character is received we check for the start marker again
				recvInProgress = false;
				// Reset ndx to 0 so that next time we read a command it gets stored at the start of the receivedChars array again
				ndx = 0;
				// This signals to the parseCommands function that a complete command is now stored in receivedChars.
				newData = true;
			}
		} else if (rc == startMarker) {
			// Start marker received. Set recvInProgress to false, so that next time we check for the end marker or a command character
			recvInProgress = true;
		}
	}
}



// These will eventually be placeholders for a better looking command parser. Currently these do nothing
#define SERIAL_CMD_INVALID_COMMAND     -1
#define SERIAL_CMD_GET_RIGHT_ASCENSION 0
#define SERIAL_CMD_GET_DECLINATION     1
#define SERIAL_CMD_MOVE_STOP           2
#define SERIAL_CMD_MOVE_START          3
#define SERIAL_CMD_SET_RIGHT_ASCENSION 4
#define SERIAL_CMD_SET_DECLINATION     5




/**
 * Serial commands start
 * This section contains a function for each serial command that can get handled
 * TODO Refactor this into a class so that we can later support connections to different programs / tools
 */
// Print the possible commands
void printHelp() {
	Serial.println(":HLP# Print available Commands");
	Serial.println(":GR# Get Right Ascension");
	Serial.println(":GD# Get Declination");
	Serial.println(":Sr,HH:MM:SS# Set Right Ascension; Example: :Sr,12:34:56#");
	Serial.println(":Sd,[+/-]DD:MM:SS# Set Declination (DD is degrees) Example: :Sd,+12:34:56#");
	Serial.println(":MS# Start Move; Starts tracking mode if not enabled");
	Serial.println(":TRK0# Disable tracking");
	Serial.println(":TRK1# Enable tracking");
	Serial.print(":DBGM[0-" + String(maxDebugPos - 1) + "]# Move to debug position X");
	Serial.println(":DBGMIA# Increase Ascension by 1 degree");
	Serial.println(":DBGMDA# Decrease Ascension by 1 degree");
	Serial.println(":DBGMID# Increase Declination by 1 degree");
	Serial.println(":DBGMDD# Decrease Declination by 1 degree");
	Serial.println(":DBGDM[00-99]# Disable Motors for XX seconds");
}



// Reports the current right ascension
void getRightAscension(Mount& scope) {
	const double pos = scope.getCurrentPosition().rightAscension / 15;

	const int hrs = static_cast<int>(pos);
	const int mins = static_cast<int>((pos - hrs) * 60);
	const int secs = static_cast<int>((pos - (hrs + mins / 60.)) * 3600);

	sprintf(txAR, "%02d:%02d:%02d#", hrs, mins, secs);

	Serial.print(txAR);
}

// Reports the current declination
void getDeclination(Mount &scope) {
	const double pos = scope.getCurrentPosition().declination;

	const int deg = static_cast<int>(pos);
	const int mins = static_cast<int>((pos - deg) * 60);
	const int secs = static_cast<int>((pos - (deg + mins / 60.)) * 3600);

	sprintf(txDEC, "%c%02d%c%02d:%02d#", deg > 0 ? '+' : '-', deg, 223, mins, secs);

	Serial.print(txDEC);
}

// Quit the current move
// TODO Implement this
// Motor Position to RaDec conversion is required for this to work and not lose the reported position
void moveQuit() {

}

// Start the requested move
// TODO This currently doesn't really work, since moves are immediately executed once
// a new position is requested.
bool moveStart(bool homingMode) {
	// Immediately confirm to Stellarium
	Serial.print("0");

	// TODO Homing code needs to be better. It has to disable the steppers and there must be some way to enable/disable it
	// If homing mode is true we set isHomed to true
	// and return true to indicate to the loop() function that homing is complete.
	if (homingMode) {
		isHomed = true;
		return true;
	}

	return false;
}


// Set Right Ascension (in hours, minutes and seconds)
void setRightAscension(Mount &telescope) {
	// Immediately confirm to Stellarium
	Serial.print("1");

	// Parse the coordinates part of the command to integers
	const int hrs = multi_char_to_int(receivedChars[3], receivedChars[4]);
	const int mins = multi_char_to_int(receivedChars[6], receivedChars[7]);
	const int secs = multi_char_to_int(receivedChars[9], receivedChars[10]);
	
	Serial.println();
	Serial.println("Changing RA");
	Serial.println(ra_deg);
	ra_deg = (hrs + mins / 60. + secs / 3600.) * 15;
	Serial.println(ra_deg);

	// Set the telescope target
	RaDecPosition scopeTarget = telescope.getTarget();
	scopeTarget.rightAscension = ra_deg;
	telescope.setTarget(scopeTarget);
}

// Set target Declination (in +/- degrees, minutes and seconds)
void setDeclination(Mount &telescope) {
	// Immediately confirm to Stellarium
	Serial.print("1");

	// Whether the coordinates are positive (1) or negative (-1)
	const int multi = (receivedChars[3] == '+') ? 1 : -1;

	// Parse the coordinates part of the command to integers
	const int deg = multi_char_to_int(receivedChars[4], receivedChars[5]);
	const int mins = multi_char_to_int(receivedChars[7], receivedChars[8]);
	const int secs = multi_char_to_int(receivedChars[10], receivedChars[11]);

	Serial.println();
	Serial.println("Changing DEC");
	Serial.println(dec_deg);
	dec_deg = (secs / 3600.0 + mins / 60.0 + deg) * multi;
	Serial.println(dec_deg);
	isPositiveDeclination = multi > 0;

	// Set the telescope target
	RaDecPosition scopeTarget = telescope.getTarget();
	scopeTarget.declination = dec_deg;
	telescope.setTarget(scopeTarget);
}


/**
 * This gets called whenever a complete command was received.
 * It parses the received characters and calls the required functions
 * TODO Break this up into small functions so that the code stays maintainable
 */
bool parseCommands(Mount &telescope, FuGPS &gps, bool homingMode) {
	if (newData == true) {
		if (receivedChars[0] == 'G' && receivedChars[1] == 'R') {
			// GR: Get Right Ascension
			getRightAscension(telescope);
		} else if (receivedChars[0] == 'G' && receivedChars[1] == 'D') {
			// GD: Get Declination
			getDeclination(telescope);
		} else if (receivedChars[0] == 'Q') {
			// Quit the current move
			moveQuit();
		} else if (receivedChars[0] == 'M' && receivedChars[1] == 'S') {
			// MS: Move Start
			// The function returning true means that isHomed was set to true.
			if (moveStart(homingMode)) {
				// Ignores the next movement update and treats it as a home command instead
				telescope.ignoreUpdates();

				newData = false;
				return true;
			}
			else {
				telescope.ignoreUpdates(false);
			}
		} else if (receivedChars[0] == 'S' && receivedChars[1] == 'r') {
			// Set Right Ascension (in hours, minutes and seconds)
			telescope.ignoreUpdates();
			setRightAscension(telescope);
		} else if (receivedChars[0] == 'S' && receivedChars[1] == 'd') {
			// Set target Declination (in degrees, minutes and seconds)
			telescope.ignoreUpdates();
			setDeclination(telescope);
		}
		else if (receivedChars[0] == 'T' && receivedChars[1] == 'R' && receivedChars[2] == 'K') {
			// Enable / Disable tracking (= home off / on)
			if (receivedChars[3] == '1') {
				telescope.setHomed(true);
				Serial.println("Enabled tracking");
			}
			else if (receivedChars[3] == '0') {
				telescope.setHomed(false);
				Serial.println("Disabled tracking");
			}
		}
		else if (receivedChars[0] == 'S' && receivedChars[1] == 'T' && receivedChars[2] == 'P') {
			// Enable / Disable stepper motors
			if (receivedChars[3] == '1') {
				digitalWrite(ALT_ENABLE_PIN, LOW);
				digitalWrite(AZ_ENABLE_PIN, LOW);
				Serial.println("Enabled stepper motors. Send :STP0# to disable them");
			}
			else if (receivedChars[3] == '0') {
				digitalWrite(ALT_ENABLE_PIN, HIGH);
				digitalWrite(AZ_ENABLE_PIN, HIGH);
				Serial.println("Disabled stepper motors. Send :STP1# to re-enable them");
			}
		}
		else if (receivedChars[0] == 'D' && receivedChars[1] == 'B'
				&& receivedChars[2] == 'G') {
			// DEBUG messages
			if (receivedChars[3] == 'M') {
				if (receivedChars[4] == 'I' || receivedChars[4] == 'D') {
					// Increase or Decrease Azimuth or Declination
					int add = receivedChars[4] == 'I' ? 1 : -1;
					if (receivedChars[5] == 'A') {
						// DBGMAXXX Move Azimuth to XXX
						ra_deg += add;
						RaDecPosition pos = telescope.getTarget();
						pos.rightAscension += add;
						telescope.setTarget(pos);
						Serial.println(
								add > 0 ?
										"Add 1 deg ascension" :
										"Sub 1 deg ascension");
					} else if (receivedChars[5] == 'D') {
						// DBGMD[+/-]XX Move Declination to +/-XX
						dec_deg += add;
						RaDecPosition pos = telescope.getTarget();
						pos.declination += add;
						telescope.setTarget(pos);
						Serial.println(
								add > 0 ?
										"Add 1 deg declination" :
										"Sub 1 deg declination");
					}
				} else {
					// Debug move to position stored in debugPositions[targetIndex]
					int targetIndex = char_to_int(receivedChars[4]);
					if (targetIndex > maxDebugPos) {
						Serial.println("Invalid index");
					} else {
						Serial.println("");
						Serial.print("Targeting ");
						Serial.println(positionNames[targetIndex]);
						Serial.println("Moving from ALT / DEC");
						Serial.println(ra_deg);
						Serial.println(dec_deg);
						Serial.println("Moving to ALT / DEC");
						Serial.println(debugPositions[targetIndex][0]);
						Serial.println(debugPositions[targetIndex][1]);
						ra_deg = debugPositions[targetIndex][0];
						dec_deg = debugPositions[targetIndex][1];
						RaDecPosition newPos = { debugPositions[targetIndex][0], debugPositions[targetIndex][1] };
						telescope.setTarget(newPos);
					}
				}
			} else if (receivedChars[3] == 'H') {
				// Home the scope
				telescope.setHomed();
			} else if (receivedChars[3] == 'D' && receivedChars[4] == 'M') {
				// Disable Motors and Pause for X seconds
				digitalWrite(ALT_ENABLE_PIN, HIGH);
				digitalWrite(AZ_ENABLE_PIN, HIGH);
				Serial.print("Disabling motors for: ");
				const int disable_seconds = multi_char_to_int(receivedChars[5], receivedChars[6]);
				Serial.println(disable_seconds);
				delay(disable_seconds * 1000);
				Serial.println("Continuing");
				digitalWrite(ALT_ENABLE_PIN, LOW);
				digitalWrite(AZ_ENABLE_PIN, LOW);
			} else if (receivedChars[3] == 'G' && receivedChars[4] == 'P' && receivedChars[5] == 'S') {
				// GPS Debug info
				Serial.println("GPS Status: ");
				Serial.print("Alive      ... ");
				Serial.println(gps.isAlive() ? "Yes" : "No");
				Serial.print("Fix        ... ");
				Serial.println((gps.hasFix() ? "Yes" : "No"));
				Serial.print("Satellites ... ");
				Serial.println(String(gps.Satellites, 6));
				Serial.print("Quality    ... ");
				Serial.println(String(gps.Quality, 6));
				Serial.print("Altitude   ... ");
				Serial.println(String(gps.Altitude, 6));
				Serial.print("Latitude   ... ");
				Serial.println(String(gps.Latitude, 6));
				Serial.print("Longitude  ... ");
				Serial.println(String(gps.Longitude, 6));
			}
		} else if (receivedChars[0] == 'H' && receivedChars[1] == 'L'
				&& receivedChars[2] == 'P') {
			printHelp();
		} else {
			Serial.println("ERROR: Unknown command");
			Serial.println(receivedChars);
		}
		newData = false;
	}
	return false;
}

// Returns true if homing command was sent
int lastDebugIndex = -1;
long lastDbgIndexSwitch = 0;
bool dbgBtnPressed = false;
bool handleSerialCommunication(Mount &telescope, FuGPS &gps, bool homingMode) {
	receiveCommandChar();
	bool ret = parseCommands(telescope, gps, homingMode);
	if (!dbgBtnPressed && digitalRead(TARGET_SELECT_PIN) == LOW) {
		// Only one button klick per 0.5 seconds is valid
		if (lastDbgIndexSwitch + 500 < millis()) {
			lastDbgIndexSwitch = millis();
			Serial.println("Switching target to " + String((lastDebugIndex + 1)));
			dbgBtnPressed = true;
			lastDebugIndex = (lastDebugIndex + 1) % 10;
			ra_deg = debugPositions[lastDebugIndex][0];
			dec_deg = debugPositions[lastDebugIndex][1];
			RaDecPosition newPos = { debugPositions[lastDebugIndex][0], debugPositions[lastDebugIndex][1] };
			telescope.setTarget(newPos);

			// Confirmation buzz
			#ifdef BUZZER_PIN
				digitalWrite(BUZZER_PIN, HIGH);
				delay(500);
				digitalWrite(BUZZER_PIN, LOW);
			#endif
		}
	}
	if (dbgBtnPressed && digitalRead(TARGET_SELECT_PIN) == HIGH) {
		// Button released
		dbgBtnPressed = false;
	}

	return ret;
}