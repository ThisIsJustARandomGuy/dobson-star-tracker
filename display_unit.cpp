#include "./config.h"
#include "./macros.h"
#include "./display_unit.h"
#include "./Observer.h"

boolean newDisplayData = false;
const byte numDisplayChars = 32;
char receivedDisplayChars[numDisplayChars];   // an array to store the received data


void initDisplayCommunication(Mount& telescope) {
	SERIAL_DISPLAY_PORT.begin(9600);

	// Set the status to "initializing" on the display unit
	SERIAL_DISPLAY_PORT.println(SERIAL_DISPLAY_CMD_STATUS_INITIALIZING);
	// Send the used MOUNT_TYPE
	//SERIAL_DISPLAY_PORT.println(SERIAL_DISPLAY_CMD_MOUNT_TYPE);
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

double chars_to_double(const unsigned int first_index) {
	return char_to_int(receivedDisplayChars[first_index]) * 100
		+ char_to_int(receivedDisplayChars[first_index+1]) * 10
		+ char_to_int(receivedDisplayChars[first_index+2]) * 1
		+ char_to_int(receivedDisplayChars[first_index+3]) * 0.1
		+ char_to_int(receivedDisplayChars[first_index+4]) * 0.01
		+ char_to_int(receivedDisplayChars[first_index+5]) * 0.001;
}

double read_angle(const unsigned int first_index) {
	const bool negative = receivedDisplayChars[first_index] == '-';
	const double result = chars_to_double(first_index + 1);

	return (negative ? -1. : 1.) * result;
}


// Set Right Ascension to (+/-)###.### degrees
void display_setRightAscension(Mount& telescope) {
	// New target Right Ascension in degrees
	double right_ascension = read_angle(2);

	DEBUG_PRINTLN();
	DEBUG_PRINTLN(receivedDisplayChars);
	DEBUG_PRINTLN("Changing Right ascension");
	DEBUG_PRINTLN(right_ascension);

	// Set the telescope target
	RaDecPosition scopeTarget = telescope.getTarget();
	scopeTarget.rightAscension = right_ascension;
	telescope.setTarget(scopeTarget);
}


// Set Declination to (+/-)##.### degrees
void display_setDeclination(Mount& telescope) {
	// New target Declination in degrees
	double declination = read_angle(2);

	DEBUG_PRINTLN();
	DEBUG_PRINTLN(receivedDisplayChars);
	DEBUG_PRINTLN("Changing Declination");
	DEBUG_PRINTLN(declination);

	// Set the telescope target
	RaDecPosition scopeTarget = telescope.getTarget();
	scopeTarget.declination = declination;
	telescope.setTarget(scopeTarget);
}

// Starts alignment mode. Puts the telescope back into Mode::INITIALIZING which disables the stepper motors
// algn
void display_startAlignment(Mount& telescope) {
	DEBUG_PRINTLN();
	DEBUG_PRINTLN("Starting alignment.");
	
	telescope.setMode(Mode::ALIGNING);

	DEBUG_PRINTLN("Confirm alignment on the display unit");
}

// Takes two arguments (right ascension and declination) and sets the telescope to be aligned to them
// sa(+/-)######(+/-)#####
void display_setAlignment(Mount& telescope) {
	double right_ascension = read_angle(2);
	double declination = read_angle(9);

	DEBUG_PRINTLN();
	DEBUG_PRINTLN(receivedDisplayChars);
	DEBUG_PRINTLN("Set aligned to ra/dec:");
	DEBUG_PRINTLN(right_ascension);
	DEBUG_PRINTLN(declination);

	// Set the alignment of the telescope so it will start tracking the correct point
	RaDecPosition alignment = { right_ascension, declination };
	telescope.setAlignment(alignment);
}

// Stops alignment mode. Puts the telescope into Mode::TRACKING which enables the stepper motors
// salgn
void display_stopAlignment(Mount& telescope) {
	DEBUG_PRINTLN();
	DEBUG_PRINTLN("Alignment done");

	telescope.setMode(Mode::TRACKING);
}


void display_statusUpdate(Mount& telescope) {
	char sbuf[10], dbuf[10];

	if (telescope.getMode() == Mode::INITIALIZING) {
		SERIAL_DISPLAY_PRINTLN(SERIAL_DISPLAY_CMD_STATUS_INITIALIZING); // Answer with Status: Initializing
	}
	else if(telescope.getMode() == Mode::ALIGNING) {
		SERIAL_DISPLAY_PRINTLN(SERIAL_DISPLAY_CMD_STATUS_ALIGNING); // Answer with Status: Aligning
	}
	else if (telescope.getMode() == Mode::TRACKING) {
		SERIAL_DISPLAY_PRINTLN(SERIAL_DISPLAY_CMD_STATUS_TRACKING); // Answer with Status: Tracking
	}

	// Altitude readout
	long angle = telescope.getTarget().declination * 1000L;
	sprintf(sbuf, "%06ld", abs(angle));
	SERIAL_DISPLAY_PRINT("dc");
	SERIAL_DISPLAY_PRINT(angle > 0 ? "+" : "-");
	SERIAL_DISPLAY_PRINTLN(sbuf);

	// Azhimuth readout
	angle = telescope.getTarget().rightAscension * 1000L;
	sprintf(dbuf, "%06ld", abs(angle));
	SERIAL_DISPLAY_PRINT("ra");
	SERIAL_DISPLAY_PRINT(angle > 0 ? "+" : "-");
	SERIAL_DISPLAY_PRINTLN(dbuf);

	DEBUG_PRINTLN("Answered with status update");
}

void display_ping() {

}


/**
 * This gets called whenever a complete command was received from the display unit
 * It parses the received characters and calls the required functions
 */
void parseDisplayCommands(Mount& telescope, Observer& observer) {
	if (newDisplayData == true) {
		if (receivedDisplayChars[0] == 's' && receivedDisplayChars[1] == '?') {
			// s? Get the status.
			display_statusUpdate(telescope);
		}
		else if (receivedDisplayChars[0] == 'r' && receivedDisplayChars[1] == 'a') {
			// az###### Set Azimuth to ###.### degrees
			display_setRightAscension(telescope);
		}
		else if (receivedDisplayChars[0] == 'd' && receivedDisplayChars[1] == 'c') {
			// al###### Set Altitude to ###.### degrees
			display_setDeclination(telescope);
		}
		else if (receivedDisplayChars[0] == 'a' && receivedDisplayChars[1] == 'l' && receivedDisplayChars[2] == 'g' && receivedDisplayChars[3] == 'n') {
			// algn Start alignment (Set scope to Mode::ALIGNING)
			display_startAlignment(telescope);
		}
		else if (receivedDisplayChars[0] == 's' && receivedDisplayChars[1] == 'a') {
			if (receivedDisplayChars[2] == 'l') {
				// salgn Stop alignment (Set scope to Mode::TRACKING)
				display_stopAlignment(telescope);
			}
			else {
				// sa(+/-)######(+/-)###### Set scope aligned to (+/-)###.### degrees right ascension, (+/-)###.### degrees declination
				display_setAlignment(telescope);
			}
		}
		newDisplayData = false;
	}
}


void handleDisplayCommunication(Mount& telescope, Observer& observer) {
	// Receives the next command character from the display, if available
	receiveDisplayCommandChar();

	// Once a complete command has been received, this parses and runs the command
	parseDisplayCommands(telescope, observer);
}