#include <Arduino.h>
#line 1 "/home/pi/Arduino/sketches/cc1310_i2c_smart_switch_slave/cc1310_i2c_smart_switch_slave.ino"

/* True2Air SmartSwitch: I2C Slave that reports switch state on demand
 * I2C Slave code taken from https://playground.arduino.cc/Code/USIi2c
 * further modified by: https://github.com/rambo/TinyWire
 *
 * See SmartSwitch.{h,cpp} for more info
 */

#define CFG_ATTINY84 1

#include "SmartSwitch.h"

SmartSwitch sw;

void setup();
void loop();

void setup() {
  sw.setup();
}

void loop() {
  sw.loop();
}



