# dobson-star-tracker
This project aims to enable makers to motorize their dobson-style mounted telescopes using easily sourced hardware.

## Hardware

+ 2 Stepper Motors + drivers
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
+ :DBGM[0-5]# Move to debug position X
+ :DBGMIA# Increase Ascension by 1 degree
+ :DBGMDA# Decrease Ascension by 1 degree
+ :DBGMID# Increase Declination by 1 degree
+ :DBGMDD# Decrease Declination by 1 degree
+ :DBGDM[0-9]# Disable Motors for X seconds

##TODOs

The Most important TODOs are as follows (in no particular order)
+ Documentation
+ Console interface which allows for basic control of the telescope (list commands, get/set position, jog to, track current position)
+ GPS: Reliably read the time from GPS
+ EEPROM: Store time and location in EEPROM
+ Motor control: On, off, improve the loop