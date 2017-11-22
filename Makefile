PKG_NAME=SmartSwitch
SRC_ROOT=/home/pi/$(PKG_NAME)
LIB_ROOT=$(SRC_ROOT)/libraries
SKETCH_OBJ=$(SRC_ROOT)/sketch.o
SKETCH_ELF=$(SRC_ROOT)/$(PKG_NAME).elf
SKETCH_HEX=$(SRC_ROOT)/$(PKG_NAME).hex

ARD_CORE_LIB=$(SRC_ROOT)/core_attiny_avr_ATtinyX4_cpu_attiny84,clock_internal8_09ff688e17cfe342a76c80acd8dcdce5.a
#ARD_CORE_LIB=$(SRC_ROOT)/core_attiny_avr_ATtinyX5_cpu_attiny84,clock_internal8_09ff688e17cfe342a76c80acd8dcdce5.a

GPP=/opt/arduino-1.8.5/hardware/tools/avr/bin/avr-g++
GCC=/opt/arduino-1.8.5/hardware/tools/avr/bin/avr-gcc
OBJCOPY=/opt/arduino-1.8.5/hardware/tools/avr/bin/avr-objcopy
AVR_DUDE=/opt/arduino-1.8.5/hardware/tools/avr/bin/avrdude

GPP_FLAGS=-c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -flto -mmcu=attiny84 -DF_CPU=8000000L -DARDUINO=10805 -DARDUINO_attiny -DARDUINO_ARCH_AVR
GCC_FLAGS=-c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=attiny84 -DF_CPU=8000000L -DARDUINO=10805 -DARDUINO_attiny -DARDUINO_ARCH_AVR

INCS= \
	-I/opt/arduino-1.8.5/hardware/arduino/avr/cores/arduino \
	-I/home/pi/.arduino15/packages/attiny/hardware/avr/1.0.2/variants/tiny14 \
	-I$(SRC_ROOT) \
	-I$(LIB_ROOT)/TinyWireS

TinyWireS-objs= \
	$(LIB_ROOT)/TinyWireS/usiTwiSlave.o \
	$(LIB_ROOT)/TinyWireS/TinyWireS.o

SmartSwitch-objs= \
	$(SRC_ROOT)/SmartSwitch.o 

all: build flash
	echo "All Done!"

build: $(SKETCH_HEX)
	echo "Build Done!"

flash: $(SKETCH_HEX)
	$(AVR_DUDE) -C$(SRC_ROOT)/avrdude.conf -v -pattiny84 -cusbtiny -Uflash:w:$(SKETCH_HEX):i 
	echo "Upload Done!"

SmartSwitch-lib: $(SmartSwitch-objs)
	echo "Done compiling SmartSwitch-lib"

%.o: %.cpp
	$(GPP) $(GPP_FLAGS) $(INCS) $< -o $@

%.o: %.c
	$(GCC) $(GCC_FLAGS) $(INCS) $< -o $@

$(SKETCH_ELF): $(SKETCH_OBJ) $(SmartSwitch-objs) $(TinyWireS-objs)
	$(GCC) -w -Os -g -flto -fuse-linker-plugin -Wl,--gc-sections -mmcu=attiny84  -o $@ $^ $(ARD_CORE_LIB) "-L/tmp/arduino_build_193316" -lm

$(SKETCH_HEX): $(SKETCH_ELF)
	$(OBJCOPY) -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 $< $(SRC_ROOT)/SmartSwitch.eep
	$(OBJCOPY) -O ihex -R .eeprom $< $@

reset:
	gpio write 0 1
	gpio mode 0 out
	gpio write 0 0
	gpio write 0 1
	gpio mode 0 in

clean:
	find -L . -name "*.o" | xargs rm


