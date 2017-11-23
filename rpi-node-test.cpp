#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

typedef unsigned char uint8_t;

#define ADDRESS 0x12

#define BusEnablePin 2  //WiringPi pin 2, Broadcom pin 27, P1 physical pin 13
//When I want the I2C bus to be active, enable BusEnablePin as output and drive HIGH
//(That pin is latching the SDA/SDL line to the ATTiny (since those pins are shared
// by the ATTiny's ISP).

void shutdownI2C() {
  digitalWrite(BusEnablePin, 0);
}

void my_handler(int s) {
  printf("Exception trapped; detaching i2c bus and halting\n");

  shutdownI2C();
  exit(0);
}

inline void assertExpected(int val,int expected) {
//  if (val < 0) {
//    printf("Error received; errmsg: %s\n", strerror(errno));
//  }
  if (val != expected) {
//    printf("Unexpected value: %02x; expected %02x\n", val, expected);
    //shutdownI2C();
    //exit(-1);
  }
}

int main(void)
{
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;

  sigaction(SIGINT, &sigIntHandler, NULL);

  wiringPiSetup();
  
  pinMode(BusEnablePin, OUTPUT);
  digitalWrite(BusEnablePin, 1);

  printf("Raspberry Pi emulating node i2c master interface\n");

  int fd = wiringPiI2CSetup(ADDRESS);
  if (fd < 0) {
    printf("ERROR: fd: %d\n", fd);
    exit(-1);
  }
  printf("INFO: fd: %d\n", fd);

  printf("INFO: sleeping for 5s\n");
  usleep(5l*1000000l);
  
  int b;
  for (;;) {
    int sleep_us = 100; 
    
    b = wiringPiI2CReadReg8(fd,1);
    printf("Byte 0: %02x\n",b);
    assertExpected(b, 0xde);
    usleep(sleep_us);

    b = wiringPiI2CReadReg8(fd,1);
    printf("Byte 1: %02x\n",b);
    assertExpected(b, 0xad);
    usleep(sleep_us);

    b = wiringPiI2CReadReg8(fd,1);
    printf("Byte 2: %02x\n",b);
    assertExpected(b, 0xbe);
    usleep(sleep_us);

    b = wiringPiI2CReadReg8(fd,1);
    printf("Byte 3: %02x\n",b);
    assertExpected(b, 0xef);
    usleep(sleep_us);

    sleep_us = 1*1000; // 1ms
    usleep(sleep_us);
  }

}

