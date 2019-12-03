#pragma once

#include "config.h"
#include "./Mount.h"
#include "./Observer.h"

void display_statusUpdate(Mount& telescope);
void initDisplayCommunication(Mount& telescope);
void handleDisplayCommunication(Mount& telescope, Observer& observer);

// Status commands
#define SERIAL_DISPLAY_CMD_STATUS_INITIALIZING "s:1"
#define SERIAL_DISPLAY_CMD_STATUS_ALIGNING "s:2"
#define SERIAL_DISPLAY_CMD_STATUS_TRACKING "s:3"

// These lines define the mount type command, depending on which MOUNT_TYPE is selected in config.h
#ifdef MOUNT_TYPE_DOBSON
#define SERIAL_DISPLAY_CMD_MOUNT_TYPE "m:1"
#endif

#ifdef MOUNT_TYPE_EQUATORIAL
#define SERIAL_DISPLAY_CMD_MOUNT_TYPE "m:2"
#endif

#ifdef MOUNT_TYPE_DIRECT
#define SERIAL_DISPLAY_CMD_MOUNT_TYPE "m:3"
#endif



// Macros
#define SERIAL_DISPLAY_PRINT(x) SERIAL_DISPLAY_PORT.print(x); DEBUG_PRINT(x)
#define SERIAL_DISPLAY_PRINTLN(x) SERIAL_DISPLAY_PORT.println(x); DEBUG_PRINTLN(x)