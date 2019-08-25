ifeq ($(OS),Windows_NT)
SHELL = cmd
RMDIR = rmdir /s /q
RM = del /q
mymkdir = if not exist "$1" mkdir "$1"
else
RMDIR = rm -fr
RM = rm -f
mymkdir = mkdir -p $1
endif

PROJECT_OBJS = \
	project/RampScope.cpp.o \
	project/conversion.cpp.o \
	project/eeprom.cpp.o \
	project/location.cpp.o \

PLATFORM_CORE_OBJS = \
	core/abi.cpp.o \
	core/CDC.cpp.o \
	core/HardwareSerial.cpp.o \
	core/HardwareSerial0.cpp.o \
	core/HardwareSerial1.cpp.o \
	core/HardwareSerial2.cpp.o \
	core/HardwareSerial3.cpp.o \
	core/hooks.c.o \
	core/IPAddress.cpp.o \
	core/main.cpp.o \
	core/new.cpp.o \
	core/PluggableUSB.cpp.o \
	core/Print.cpp.o \
	core/Stream.cpp.o \
	core/Tone.cpp.o \
	core/USBCore.cpp.o \
	core/WInterrupts.c.o \
	core/wiring.c.o \
	core/wiring_analog.c.o \
	core/wiring_digital.c.o \
	core/wiring_pulse.c.o \
	core/wiring_pulse.S.o \
	core/wiring_shift.c.o \
	core/WMath.cpp.o \
	core/WString.cpp.o \

PLATFORM_VARIANT_OBJS = \

LIBRARIES_OBJS = \
	libraries/FuGPS_Library/1.1.0/src/FuGPS.cpp.o \
	libraries/AccelStepper/1.58.0/src/AccelStepper.cpp.o \
	libraries/AccelStepper/1.58.0/src/MultiStepper.cpp.o \
	libraries/TimerOne/1.1.0/TimerOne.cpp.o \

TARGETS = \
	RampScope.hex \
	RampScope.eep \

all: $(TARGETS)

RampScope.hex: RampScope.elf
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-objcopy" -O ihex -R .eeprom  "./RampScope.elf" "./RampScope.hex"

RampScope.eep: RampScope.elf
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-objcopy" -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0  "./RampScope.elf" "./RampScope.eep"

RampScope.elf: $(PROJECT_OBJS) $(LIBRARIES_OBJS) core.a
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc" -w -Os -g -flto -fuse-linker-plugin -Wl,--gc-sections -mmcu=atmega2560  -o "./RampScope.elf" $(PROJECT_OBJS) $(LIBRARIES_OBJS) "./core.a" "-L." -lm

core.a:	$(PLATFORM_CORE_OBJS) $(PLATFORM_VARIANT_OBJS)

clean:
	-$(RMDIR) project core variant libraries
	-$(RM) *.hex *.eep *.bin *.elf *.a *.ar *.d

size:
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-size" -A "./RampScope.elf"

project/RampScope.cpp.o: ../../RampScope.cpp project/RampScope.cpp.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"

project/RampScope.cpp.d: ;

-include project/RampScope.cpp.d 

project/conversion.cpp.o: ../../conversion.cpp project/conversion.cpp.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"

project/conversion.cpp.d: ;

-include project/conversion.cpp.d 

project/eeprom.cpp.o: ../../eeprom.cpp project/eeprom.cpp.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"

project/eeprom.cpp.d: ;

-include project/eeprom.cpp.d 

project/location.cpp.o: ../../location.cpp project/location.cpp.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"

project/location.cpp.d: ;

-include project/location.cpp.d 


core/abi.cpp.o: C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/abi.cpp core/abi.cpp.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/abi.cpp.d: ;

-include core/abi.cpp.d

core/CDC.cpp.o: C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/CDC.cpp core/CDC.cpp.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/CDC.cpp.d: ;

-include core/CDC.cpp.d

core/HardwareSerial.cpp.o: C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/HardwareSerial.cpp core/HardwareSerial.cpp.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/HardwareSerial.cpp.d: ;

-include core/HardwareSerial.cpp.d

core/HardwareSerial0.cpp.o: C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/HardwareSerial0.cpp core/HardwareSerial0.cpp.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/HardwareSerial0.cpp.d: ;

-include core/HardwareSerial0.cpp.d

core/HardwareSerial1.cpp.o: C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/HardwareSerial1.cpp core/HardwareSerial1.cpp.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/HardwareSerial1.cpp.d: ;

-include core/HardwareSerial1.cpp.d

core/HardwareSerial2.cpp.o: C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/HardwareSerial2.cpp core/HardwareSerial2.cpp.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/HardwareSerial2.cpp.d: ;

-include core/HardwareSerial2.cpp.d

core/HardwareSerial3.cpp.o: C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/HardwareSerial3.cpp core/HardwareSerial3.cpp.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/HardwareSerial3.cpp.d: ;

-include core/HardwareSerial3.cpp.d

core/hooks.c.o: C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/hooks.c core/hooks.c.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc" -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"
	
core/hooks.c.d: ;

-include core/hooks.c.d

core/IPAddress.cpp.o: C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/IPAddress.cpp core/IPAddress.cpp.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/IPAddress.cpp.d: ;

-include core/IPAddress.cpp.d

core/main.cpp.o: C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/main.cpp core/main.cpp.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/main.cpp.d: ;

-include core/main.cpp.d

core/new.cpp.o: C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/new.cpp core/new.cpp.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/new.cpp.d: ;

-include core/new.cpp.d

core/PluggableUSB.cpp.o: C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/PluggableUSB.cpp core/PluggableUSB.cpp.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/PluggableUSB.cpp.d: ;

-include core/PluggableUSB.cpp.d

core/Print.cpp.o: C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/Print.cpp core/Print.cpp.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/Print.cpp.d: ;

-include core/Print.cpp.d

core/Stream.cpp.o: C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/Stream.cpp core/Stream.cpp.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/Stream.cpp.d: ;

-include core/Stream.cpp.d

core/Tone.cpp.o: C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/Tone.cpp core/Tone.cpp.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/Tone.cpp.d: ;

-include core/Tone.cpp.d

core/USBCore.cpp.o: C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/USBCore.cpp core/USBCore.cpp.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/USBCore.cpp.d: ;

-include core/USBCore.cpp.d

core/WInterrupts.c.o: C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/WInterrupts.c core/WInterrupts.c.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc" -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"
	
core/WInterrupts.c.d: ;

-include core/WInterrupts.c.d

core/wiring.c.o: C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/wiring.c core/wiring.c.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc" -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"
	
core/wiring.c.d: ;

-include core/wiring.c.d

core/wiring_analog.c.o: C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/wiring_analog.c core/wiring_analog.c.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc" -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"
	
core/wiring_analog.c.d: ;

-include core/wiring_analog.c.d

core/wiring_digital.c.o: C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/wiring_digital.c core/wiring_digital.c.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc" -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"
	
core/wiring_digital.c.d: ;

-include core/wiring_digital.c.d

core/wiring_pulse.c.o: C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/wiring_pulse.c core/wiring_pulse.c.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc" -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"
	
core/wiring_pulse.c.d: ;

-include core/wiring_pulse.c.d

core/wiring_pulse.S.o: C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/wiring_pulse.S
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc" -c -g -x assembler-with-cpp -flto -MMD -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/wiring_shift.c.o: C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/wiring_shift.c core/wiring_shift.c.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc" -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"
	
core/wiring_shift.c.d: ;

-include core/wiring_shift.c.d

core/WMath.cpp.o: C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/WMath.cpp core/WMath.cpp.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/WMath.cpp.d: ;

-include core/WMath.cpp.d

core/WString.cpp.o: C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/WString.cpp core/WString.cpp.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/WString.cpp.d: ;

-include core/WString.cpp.d



libraries/FuGPS_Library/1.1.0/src/FuGPS.cpp.o: C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src/FuGPS.cpp libraries/FuGPS_Library/1.1.0/src/FuGPS.cpp.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"

libraries/FuGPS_Library/1.1.0/src/FuGPS.cpp.d: ;

-include libraries/FuGPS_Library/1.1.0/src/FuGPS.cpp.d

libraries/AccelStepper/1.58.0/src/AccelStepper.cpp.o: C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src/AccelStepper.cpp libraries/AccelStepper/1.58.0/src/AccelStepper.cpp.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"

libraries/AccelStepper/1.58.0/src/AccelStepper.cpp.d: ;

-include libraries/AccelStepper/1.58.0/src/AccelStepper.cpp.d

libraries/AccelStepper/1.58.0/src/MultiStepper.cpp.o: C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src/MultiStepper.cpp libraries/AccelStepper/1.58.0/src/MultiStepper.cpp.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"

libraries/AccelStepper/1.58.0/src/MultiStepper.cpp.d: ;

-include libraries/AccelStepper/1.58.0/src/MultiStepper.cpp.d

libraries/TimerOne/1.1.0/TimerOne.cpp.o: C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0/TimerOne.cpp libraries/TimerOne/1.1.0/TimerOne.cpp.d
	@$(call mymkdir,$(dir $@))
	"C:/Users/lukas/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/mega" -I"C:/Users/lukas/.arduinocdt/libraries/FuGPS_Library/1.1.0/src" -I"C:/Users/lukas/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"C:/Users/lukas/.arduinocdt/libraries/AccelStepper/1.58.0/src" -I"C:/Users/lukas/.arduinocdt/libraries/TimerOne/1.1.0" "$<" -o "$@"

libraries/TimerOne/1.1.0/TimerOne.cpp.d: ;

-include libraries/TimerOne/1.1.0/TimerOne.cpp.d

