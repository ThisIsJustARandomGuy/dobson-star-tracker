#include "./config.h"
#include "./macros.h"
#include "./display_unit.h"

boolean newDisplayData = false;
const byte numDisplayChars = 32;
char receivedDisplayChars[numDisplayChars];   // an array to store the received data


void initDisplayCommunication(Mount& telescope) {
	SERIAL_DISPLAY_PORT.begin(9600);

	// Set the status to "initializing" on the display unit
	SERIAL_DISPLAY_PORT.println(SERIAL_DISPLAY_CMD_STATUS_INITIALIZING);
	// Send the used MOUNT_TYPE
	//SERIAL_DISPLAY_PORT.println(SERIAL_DISPLAY_CMD_MOUNT_TYPE);
	display_statusUpdate(telescope);
}

/*
 * Every loop iteration this function checks if a new character is available from the Serial interface. It only reads the
 * character if the variable newData is set to false. It then tries to read a complete command (1 char per loop iteration).
 * When a command is complete, this function sets newData to true. Later on in the loop iteration the parseCommands function,
 * which tries to parse and execute the command, gets called. After command execution the newData variable is reset to false
 */
void receiveDisplayCommandChar() {
	static byte ndx = 0;
	char endMarker = '\n';
	char rc;

	while (SERIAL_DISPLAY_PORT.available() > 0 && newDisplayData == false) {
		rc = SERIAL_DISPLAY_PORT.read();

		if (rc != endMarker) {
			receivedDisplayChars[ndx] = rc;
			ndx++;
			if (ndx >= numDisplayChars) {
				ndx = numDisplayChars - 1;
			}
		}
		else {
			receivedDisplayChars[ndx] = '\0'; // terminate the string
			ndx = 0;
			newDisplayData = true;
		}
	}
}


// Set Altitude to ###.# degrees
void display_setAzimuth(Mount& telescope) {
	// New target Azimuth in degrees
	double new_azimuth = char_to_int(receivedDisplayChars[3]) * 100
		+ char_to_int(receivedDisplayChars[4]) * 10
		+ char_to_int(receivedDisplayChars[5]) * 1
		+ char_to_int(receivedDisplayChars[6]) * 0.1
		+ char_to_int(receivedDisplayChars[7]) * 0.01
		+ char_to_int(receivedDisplayChars[8]) * 0.001;

	if (receivedDisplayChars[2] == '-') {
		new_azimuth *= -1.;
	}

	DEBUG_PRINTLN();
	DEBUG_PRINTLN(receivedDisplayChars);
	DEBUG_PRINTLN("Changing Azimuth");
	DEBUG_PRINTLN(new_azimuth);

	// Set the telescope target
	RaDecPosition scopeTarget = telescope.getTarget();
	scopeTarget.rightAscension = new_azimuth;
	telescope.setTarget(scopeTarget);
}


// Set Altitude to ###.# degrees
void display_setAltitude(Mount& telescope) {
	// New target Altitude in degrees
	double new_altitude = char_to_int(receivedDisplayChars[3]) * 100
		+ char_to_int(receivedDisplayChars[4]) * 10
		+ char_to_int(receivedDisplayChars[5]) * 1
		+ char_to_int(receivedDisplayChars[6]) * 0.1
		+ char_to_int(receivedDisplayChars[7]) * 0.01
		+ char_to_int(receivedDisplayChars[8]) * 0.001;

	if (receivedDisplayChars[2] == '-') {
		new_altitude *= -1.;
	}

	DEBUG_PRINTLN();
	DEBUG_PRINTLN(receivedDisplayChars);
	DEBUG_PRINTLN("Changing Altitude");
	DEBUG_PRINTLN(new_altitude);

	// Set the telescope target
	RaDecPosition scopeTarget = telescope.getTarget();
	scopeTarget.declination = new_altitude;
	telescope.setTarget(scopeTarget);
}

void display_statusUpdate(Mount& telescope) {
	char sbuf[10], dbuf[10];

	if (telescope.getMode() == Mode::TRACKING) {
		SERIAL_DISPLAY_PRINTLN(SERIAL_DISPLAY_CMD_STATUS_ONLINE); // Answer with Status: Tracking
	}
	else {
		SERIAL_DISPLAY_PRINTLN(SERIAL_DISPLAY_CMD_STATUS_ONLINE); // Answer with Status: Online
	}

	// Altitude readout
	long angle = telescope.getTarget().declination * 1000L;
	sprintf(sbuf, "%06ld", abs(angle));
	SERIAL_DISPLAY_PRINT("al");
	SERIAL_DISPLAY_PRINT(angle > 0 ? "+" : "-");
	SERIAL_DISPLAY_PRINTLN(sbuf);

	// Azhimuth readout
	angle = telescope.getTarget().rightAscension * 1000L;
	sprintf(dbuf, "%06ld", abs(angle));
	SERIAL_DISPLAY_PRINT("az");
	SERIAL_DISPLAY_PRINT(angle > 0 ? "+" : "-");
	SERIAL_DISPLAY_PRINTLN(dbuf);

	DEBUG_PRINTLN("Answered with status update");
}

/**
 * This gets called whenever a complete command was received from the display unit
 * It parses the received characters and calls the required functions
 */
void parseDisplayCommands(Mount& telescope, FuGPS& gps, bool homingMode) {
	if (newDisplayData == true) {
		if (receivedDisplayChars[0] == 's' && receivedDisplayChars[1] == '?') {
			// s? Get the status.
			display_statusUpdate(telescope);
		}
		else if (receivedDisplayChars[0] == 'a' && receivedDisplayChars[1] == 'z') {
			// az#### Set Azimuth to ###.# degrees
			display_setAzimuth(telescope);
		}
		else if (receivedDisplayChars[0] == 'a' && receivedDisplayChars[1] == 'l') {
			// al#### Set Altitude to ###.# degrees
			display_setAltitude(telescope);
		}
		newDisplayData = false;
	}
}


void handleDisplayCommunication(Mount& telescope, FuGPS& gps, bool homingMode) {
	// Receives the next command character from the display, if available
	receiveDisplayCommandChar();

	// Once a complete command has been received, this parses and runs the command
	parseDisplayCommands(telescope, gps, homingMode);
}