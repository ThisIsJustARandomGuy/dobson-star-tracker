# dobson-star-tracker
This project aims to enable makers to motorize their dobson-style mounted telescopes using easily sourced hardware.

## Hardware

+ 2 Stepper Motors + drivers. I found that the size of my telescope (as pictured badly below) requires at least NEMA 17 with 1.2A current per phase. The 
+ Arduino with at least 6 Digital Pins. The instructions and example config work for an Arduino Mega with a RAMPS 1.4 shield
+ 3D printed motor mounts and gears
+ Optional: Two Buttons; Each requires one additional digital pin
+ Optional: A GPS module; Requires either a free TX/RX pin pair or two digital pins

## Serial Commands
+ :HLP# Print available Commands
+ :GR# Get Right Ascension
+ :GD# Get Declination
+ :Sr,HH:MM:SS# Set RA; Example: :Sr,12:34:56#
+ :Sd,[+/-]DD:MM:SS# Set Declination (DD is degrees) Example: :Sd,+12:34:56#
+ :MS# Start Move
+ :Q# Quit Move (Not Implemented)
+ :DBGM[0-5]# Move to debug position X
+ :DBGMIA# Increase Ascension by 1 degree
+ :DBGMDA# Decrease Ascension by 1 degree
+ :DBGMID# Increase Declination by 1 degree
+ :DBGMDD# Decrease Declination by 1 degree
+ :DBGDM[0-9]# Disable Motors for X seconds

## TODOs

The Most important TODOs are as follows (in no particular order)
+ Documentation
+ Board compatibility: Out-of-the-box support for Arduino Mega and Arduino Due with their respective RAMPS shields
+ GPS: Reliably read the time from GPS
+ Time keeping: Handle big swings which could happen due to GPS issues
+ EEPROM: Store time and location in EEPROM (Mega) or Flash (Due); Provide a simple API for doing so
+ Motor control: Turn On/Off permanently. Off for X seconds is already implemented as :DBGDM[0-9]#
