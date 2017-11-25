
/* True2Air SmartSwitch: I2C Slave that reports switch state on demand
 * I2C Slave code taken from https://playground.arduino.cc/Code/USIi2c
 * further modified by: https://github.com/rambo/TinyWire
 *
 * SETUP ATTiny85/45:
 * ATtiny Pin 1 = (RESET) N/U                      ATtiny Pin 2 = (D3) N/U
 * ATtiny Pin 3 = (PB4) to DEBUG_LED1              ATtiny Pin 4 = GND
 * ATtiny Pin 5 = I2C SDA on DS1621  & GPIO        ATtiny Pin 6 = DEBUG_LED2
 * ATtiny Pin 7 = I2C SCL on DS1621  & GPIO        ATtiny Pin 8 = VCC (2.7-5.5V)
 * 
 * SETUP ATTiny84/44:
 * ATtiny Pin 1  = VCC (2.7-5.5V)                   ATtiny Pin 2 = (PB0) unused
 * ATtiny Pin 3  = (PB1) unused                     ATtiny Pin 4 = pgrm XRST
 * ATtiny Pin 5  = (PB2) unused                     ATtiny Pin 6  = (PA7) unused
 * ATtiny Pin 7  = I2C SDA & pgrm MOSI              ATtiny Pin 8  = pgrm MISO
 * ATtiny Pin 9  = I2C SCL & pgrm SCK               ATtiny Pin 10 = PA3 Switch input (pullup)
 * ATtiny Pin 11 = (PA2) unused                     ATtiny Pin 12 = DEBUG_LED1
 * ATtiny Pin 13 = DEBUG_LED2                       ATtiny Pin 14 = GND
 * 
 * NOTE! - It's very important to use pullups on the SDA & SCL lines!
 * Current Rx & Tx buffers set at 32 bytes - see usiTwiSlave.h
 * 
 * Credit and thanks to Don Blake for his usiTwiSlave code. 
 * More on TinyWireS usage - see TinyWireS.h
 */

#include <Arduino.h>

#include "SmartSwitch.h"

//JFC: sample code from contiki t2a example appears to start in master mode -- I think
// the idea is that it sends out a request for id, and the master will respond by allocating
// an idea for the rest of lifetime.
//
// Unfortunately, the ATTiny85 doesn't have full I2C support, so for now, I'm going to get it
// working strictly as a slave with the TinyWireS library.  Then I think I can come back and
// introduce the TinyWireM package for the proper t2a initialization

//STATES
//communication states
enum I2C_COMM_PROT_ACTION {GET_SENSACT_NUM, GET_SENSOR_NAME, GET_SENSOR_TYPE, SENSOR_READ, SENS_ACT_WRITE, NOOP};

// Sensor return types
enum TRU2AIR_SENSOR_DATA_TYPE {SENS_DOUBLE, SENS_UINT32 };

#include "TinyWireS.h"                  // wrapper class for I2C slave routines
#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE ( 64 )
#endif

volatile uint8_t i2c_regs[] =
{
    0xDE, 
    0xAD,
    0xBE, 
    0xEF, 
};
// Tracks the current register pointer position
volatile byte reg_position;
const byte reg_size = sizeof(i2c_regs);

#define BUS_INACTIVITY_MS 2
volatile uint32_t busAct_ms = 0;
volatile byte busIsActive = 0;


static void receiveEvent(uint8_t howMany);
static void requestEvent();
static void blink(byte led, byte times);
static void blink(volatile byte &port, byte mask, byte times);
static void halt(byte led, byte times);

#if defined(__AVR_ATtiny84__)
#define DEBUG_LED1_APIN  1               // ATtiny84 PA1 Pin 12
#define DEBUG_LED1_PORT  PORTA
#define DEBUG_LED1_PIN   PINA
#define DEBUG_LED1_MASK  0b00000010
#define DEBUG_LED2_APIN  0               // ATtiny84 PA0 Pin 13
#define DEBUG_LED2_PORT  PORTA
#define DEBUG_LED2_PIN   PINA
#define DEBUG_LED2_MASK  0b00000001
#define SDA_PIN          6               // ATtiny84 Pin 7
#define SCL_APIN         4               // ATtiny84 PA4 Pin 9
#define SCL_PIN          PINA
#define SCL_MASK         0b00010000
#define SWITCH_APIN      3               // ATtiny84 PA3 Pin 10
#define SWITCH_PIN       PINA
#define SWITCH_PORT      PORTA
#define SWITCH_MASK      0b00001000
#endif

#if defined(__AVR_ATtiny85__)
#define DEBUG_LED1_APIN  4               // ATtiny85 PB4 Pin 3
#define DEBUG_LED1_PORT  PORTB
#define DEBUG_LED1_PIN   PINB
#define DEBUG_LED1_MASK  0b00010000
#define DEBUG_LED2_APIN  1               // ATtiny85 PB1 pin 6
#define DEBUG_LED2_PORT  PORTB
#define DEBUG_LED2_PIN  PINB
#define DEBUG_LED2_MASK  0b00000010
#define SDA_PIN          0               // ATtiny85 pin 5
#define SCL_APIN         2               // ATtiny85 PB2 pin 7
#define SCL_PIN          PINB
#define SCL_MASK         0b00000100
#endif

#define HALT() halt(DEBUG_LED2_APIN,0)
#define HALT(x) halt(DEBUG_LED2_APIN,x)


byte STATE = GET_SENSACT_NUM;

typedef struct tru2air_header_t {
  unsigned char action;
  unsigned char specifier;
} tru2air_header_t;

tru2air_header_t HEADER = {0xff,0xff};

typedef struct sensact_descriptor_t {
  unsigned char name [24];
  unsigned char type;
} sensact_descriptor_t;


//TEMPORARY DUMMY VARIABLES
byte SENS_NUM = 0x03;
sensact_descriptor_t sensors[] = {
   {"1_st_sensor", SENS_DOUBLE},
   {"2_nd_sensor", SENS_DOUBLE},
   {"3_rd_sensor", SENS_UINT32}
};
unsigned char device_addr[] = {0xde, 0xad, 0xbe, 0xef};


inline void BUS_ACTIVE(unsigned int ms) {
  busAct_ms = ms;
  busIsActive = 1;
  DEBUG_LED1_PORT &= ~DEBUG_LED1_MASK;
}

inline void BUS_INACTIVE() {
  busIsActive = 0;
  DEBUG_LED1_PORT |= DEBUG_LED1_MASK;
}

void receiveEventNoop(uint8_t howMany) {}
void requestEventNoop() {}

void SmartSwitch::setup() {
  pinMode(DEBUG_LED1_APIN,OUTPUT);        // for general DEBUG use
  pinMode(DEBUG_LED2_APIN,OUTPUT);        // for general HALT use
  pinMode(SCL_APIN, INPUT);
  pinMode(SWITCH_APIN, INPUT);
  digitalWrite(SWITCH_APIN, HIGH);        // enable pullup

  digitalWrite(DEBUG_LED1_APIN,HIGH);     // make sure LEDs start out off
  digitalWrite(DEBUG_LED2_APIN,HIGH);

  while ((digitalRead(SCL_APIN) == 0) || (digitalRead(SDA_PIN) == 0)) {
    // waiting for both i2c lines to be high -- indicating live bus (though not necessarily active/available)
    delay(125);
    DEBUG_LED1_PORT &= ~DEBUG_LED1_MASK;
    delay(125);
    DEBUG_LED1_PORT |= DEBUG_LED1_MASK;
  }
  BUS_ACTIVE(millis());
  
  blink(DEBUG_LED1_PORT, DEBUG_LED1_MASK, 6); // show we're alive
  delay(1500);                           // pause for 1.5 seconds (NOTE: can't use delay beyond here!)

  TinyWireS.begin(0x12);      // init I2C Slave mode
  TinyWireS.onReceive(receiveEvent);
  TinyWireS.onRequest(requestEvent);
}

bool triggered = true;
void SmartSwitch::loop() {
#ifdef FOO
  byte scl = SCL_PIN&SCL_MASK;
  if (scl == 0) {
    BUS_ACTIVE(millis());
  } else if (busIsActive) {
    if (millis() - busAct_ms > BUS_INACTIVITY_MS) {
      BUS_INACTIVE();
    }
  }
#endif
  
  TinyWireS_stop_check();
  if (triggered) return;
    /**
     * This is the only way we can detect stop condition (http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&p=984716&sid=82e9dc7299a8243b86cf7969dd41b5b5#984716)
     * it needs to be called in a very tight loop in order not to miss any (REMINDER: Do *not* use delay() anywhere, use tws_delay() instead).
     * It will call the function registered via TinyWireS.onReceive(); if there is data in the buffer on stop.
     */

     if (reg_position != 0) {
       triggered = true;
//       halt(DEBUG_LED1_APIN, reg_position);
//        HALT(10);
        unsigned char cmd = i2c_regs[0];
        switch(cmd) {
        case GET_SENSACT_NUM:
            STATE = GET_SENSACT_NUM;
            break;
        case GET_SENSOR_NAME:
            STATE = GET_SENSOR_NAME;
            break;
        case GET_SENSOR_TYPE:
            STATE = GET_SENSOR_TYPE;
            break;
        default:
            STATE = NOOP;
            break;
        }
     } else STATE = NOOP;

     TinyWireS_stop_check();
}



/**
 * This is called for each read request we receive, never put more than one byte of data (with TinyWireS.send) to the 
 * send-buffer when using this callback
 */
static void requestEvent()
{  
//halt(DEBUG_LED2_APIN, 8);
    TinyWireS.send(i2c_regs[reg_position]);
    // Increment the reg position on each read, and loop back to zero
    reg_position++;
    if (reg_position >= reg_size)
    {
        reg_position = 0;
    }
}


/**
 * The I2C data received -handler
 *
 * This needs to complete before the next incoming transaction (start, data, restart/stop) on the bus does
 * so be quick, set flags for long running tasks to be called from the mainloop instead of running them directly,
 */
static void receiveEvent(uint8_t howMany)
{
    if (howMany < 1)
    {
        // Sanity-check
        return;
    }
    if (howMany > TWI_RX_BUFFER_SIZE)
    {
        // Also insane number
        return;
    }

    reg_position = TinyWireS.receive();
    howMany--;
    if (!howMany)
    {
        // This write was only to set the buffer for next read
        if (reg_position == 1)
	  i2c_regs[1] = SWITCH_PIN&SWITCH_MASK ? 1 : 0;
        return;
    }
//halt(DEBUG_LED2_APIN,5);
    while(howMany--)
    {
        i2c_regs[reg_position] = TinyWireS.receive();
        reg_position++;
        if (reg_position >= reg_size)
        {
            reg_position = 0;
        }
    }
}


void receiveCb(int numBytes) {
  // reseting the specifier if there is any
  HEADER.specifier = 0xff;

  HEADER.action = TinyWireS.receive();
  
  if(numBytes == 2) {
    HEADER.specifier = TinyWireS.receive();
  }
  
  switch(HEADER.action) {
    case GET_SENSACT_NUM:
      STATE = GET_SENSACT_NUM;
      break;
    case GET_SENSOR_NAME:
      STATE = GET_SENSOR_NAME;
      break;
    case GET_SENSOR_TYPE:
      STATE = GET_SENSOR_TYPE;
      break;
    default:
      break;
  }  

}

void requestCb() {
  bool endOfMsg = false;
  unsigned char msgChar = 0;

  switch(STATE) {
    case GET_SENSACT_NUM:
      TinyWireS.send(device_addr[0]);
      TinyWireS.send(device_addr[1]);
      TinyWireS.send(device_addr[2]);
      TinyWireS.send(device_addr[3]);
      TinyWireS.send(sizeof(sensors)/sizeof(sensact_descriptor_t));
      break;
    case GET_SENSOR_NAME:
      //TODO: handle bad HEADER

      for (int i = 0; i < strlen(sensors[HEADER.specifier].name)+1; i++)
        TinyWireS.send((unsigned char)sensors[HEADER.specifier].name[i]);
      
      break;
      
    case GET_SENSOR_TYPE:
      TinyWireS.send(sensors[HEADER.specifier].type);  
      break;
      
    default:
      break;
  }
}

static void blink(byte led, byte times){ // poor man's display
  for (byte i=0; i< times; i++){
    digitalWrite(led,LOW);
    tws_delay(275);
    digitalWrite(led,HIGH);
    tws_delay(275);
  }
}


static void blink(volatile byte &port, byte mask, byte times){ // poor man's display
  for (byte i=0; i< times; i++){
    port &= ~mask;
    tws_delay(275);
    port |= mask;
    tws_delay(275);
  }
}


static void halt(byte led, byte times) {
  for (byte i=0; i< times; i++){
    digitalWrite(led,LOW);
    delay(275);
    digitalWrite(led,HIGH);
    delay(275);
  }
  delay(1000);
  digitalWrite(led,LOW);
  while (1) {}
}
