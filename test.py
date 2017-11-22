#!/usr/bin/python
#
# Basic example of using Python-SMBus and a ADS1115

#from smbus2 import SMBus
import smbus as smbus
import RPi.GPIO as GPIO
import time
import datetime
import sys
import traceback

#bus = SMBus(1)
bus = smbus.SMBus(1)

# default address of the SmartSwitch
address = 0x12

# Pin Definitons:

#I2C Bus enable management 
BusEnablePin = 27 # Broadcom pin 27 (P1 pin 13)

SDAPin = 2 # Broadcom pin 2 (P1 pin 3)
SDLPin = 3 # Broadcom pin 3 (P1 pin 5)

#When I want the I2C bus to be active, enable BusEnablePin as output and drive HIGH
#(That pin is latching the SDA/SDL line to the ATTiny (since those pins are shared
# by ISP).
#Similarly, when disabled, set SDAPin and SDLPin as HiZ for the same reason.
#
#Note pullups aren't needed on RPI as they are builtin on the board


def readSample(mux):
    v = bus.read_word_data(address, conversion_reg)
    #note, result is interpreted by Python as little-endian, but it's actually
    #big-endian -- so swap the bytes
    bh = (v>>8)&0xff
    bl = v&0xff
    v = (bl<<8)|bh
    if (v > 0x8000):
        v = 0
    return v


def nowstr():
    fmt = 'INFO: %Y-%b-%d %H:%M:%S :'
    return datetime.datetime.today().strftime('INFO: %Y-%b-%d %H:%M:%S :')


def shutdown():
    GPIO.setup(BusEnablePin, GPIO.IN)
    bus.close()
    GPIO.setup(SDAPin, GPIO.IN)
    GPIO.setup(SDLPin, GPIO.IN)
    GPIO.cleanup()
    
def sysexception(t,e,tb):
    print nowstr(), "Exception trapped; disabling i2c bus and halting"
    print nowstr(), "Type: ",t
    print nowstr(), "Exception: ",e
    print nowstr(), "Trace: ", traceback.print_tb(tb)
    shutdown()
    
    
# Pin Setup:
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme
GPIO.setup(BusEnablePin, GPIO.OUT)
GPIO.output(BusEnablePin, GPIO.HIGH)

print nowstr(), "True2Air I2C Tester started"

sys.excepthook = sysexception

# get device info
cmd_info = 0x01
while True:
    time.sleep(0.1)
    print nowstr(), "Byte 1: ",bus.read_byte_data(address, 1)
    time.sleep(0.1)
    print nowstr(), "Byte 1: ",bus.read_byte_data(address, 1)
    time.sleep(0.1)
    print nowstr(), "Byte 1: ",bus.read_byte_data(address, 1)
    time.sleep(0.1)
    print nowstr(), "Byte 1: ",bus.read_byte_data(address, 1)

#bus.write_i2c_block_data( address, cmd_info )


shutdown()
