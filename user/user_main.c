/******************************************************************************
 * Copyright 2013-2014 Espressif Systems (Wuxi)
 *
 * FileName: user_main.c
 *
 * Description: entry file of user application
 *
 * Modification history:
 *     2014/1/1, v1.0 create this file.
*******************************************************************************/
#include "ets_sys.h"
#include "osapi.h"
#include "gpio.h"

#include "arduinostyle.h"
#include "arduino/arduino_spi.h"

#include "user_interface.h"

#include "user_devicefind.h"
#include "user_webserver.h"

#if ESP_PLATFORM
#include "user_esp_platform.h"
#endif

#ifdef SERVER_SSL_ENABLE
#include "ssl/cert.h"
#include "ssl/private_key.h"
#else
#ifdef CLIENT_SSL_ENABLE
unsigned char *default_certificate;
unsigned int default_certificate_len = 0;
unsigned char *default_private_key;
unsigned int default_private_key_len = 0;
#endif
#endif

#include "ets_sys.h"
#include "os_type.h"
#include "osapi.h"
#include "mem.h"
#include "user_interface.h"

#include "espconn.h"

static volatile os_timer_t dbgTimer;

#define delay(t) ets_delay_us(t * 1000)
#define _CS_pin 12
#define _SCK_pin 14
#define _SO_pin 13
//#define LOW(a) gpio_output_set(0, a, a, 0) //set GPIO low
#define LOW(a) GPIO_OUTPUT_SET(a, 0);
//#define HIGH(a) gpio_output_set (a, 0, a, 0) // set GPIO high
#define HIGH(a) GPIO_OUTPUT_SET(a, 1);
#define digitalRead(a) GPIO_INPUT_GET(a)
#define OP_NOOP   0
#define OP_DIGIT0 1
#define OP_DIGIT1 2
#define OP_DIGIT2 3
#define OP_DIGIT3 4
#define OP_DIGIT4 5
#define OP_DIGIT5 6
#define OP_DIGIT6 7
#define OP_DIGIT7 8
#define OP_DECODEMODE  9
#define OP_INTENSITY   10
#define OP_SCANLIMIT   11
#define OP_SHUTDOWN    12
#define OP_DISPLAYTEST 15

void dbgMessageTimeout(void)
{
    uint16_t value = 0;
    uint8_t error_tc = 0;

    /*
      Initiate a temperature conversion. According to MAX's tech notes FAQ's
      for the chip, Line going high initiates a conversion, which means, we
      need to clock the chip low to high to initiate the conversion, then wait
      for the conversion to be complete before trying to read the data from
      the chip.
    */
    LOW(_CS_pin);
    delay(2);
    HIGH(_CS_pin);
    delay(220);

    /* Read the chip and return the raw temperature value */

    /*
      Bring CS pin low to allow us to read the data from
      the conversion process
    */
    LOW(_CS_pin);

    /* Cycle the clock for dummy bit 15 */
    HIGH(_SCK_pin);
    delay(1);
    LOW(_SCK_pin);

     /*
      Read bits 14-3 from MAX6675 for the Temp. Loop for each bit reading
      the value and storing the final value in 'temp'
    */
    int i = 0;

    for (i=11; i>=0; i--) {
        HIGH(_SCK_pin);
        value += digitalRead(_SO_pin) << i;
        LOW(_SCK_pin);
    }

    /* Read the TC Input inp to check for TC Errors */
    HIGH(_SCK_pin);
    error_tc = digitalRead(_SO_pin);
    LOW(_SCK_pin);

    /*
      Read the last two bits from the chip, faliure to do so will result
      in erratic readings from the chip.
    */
    for (i=1; i>=0; i--) {
        HIGH(_SCK_pin);
        delay(1);
        LOW(_SCK_pin);
    }

    // Disable Device
    HIGH(_CS_pin);

    /*
      Keep in mind that the temp that was just read is on the digital scale
      from 0˚C to 1023.75˚C at a resolution of 2^12.  We now need to convert
      to an actual readable temperature (this drove me nuts until I figured
      this out!).  Now multiply by 0.25.  I tried to avoid float math but
      it is tough to do a good conversion to ˚F.  THe final value is converted
      to an int and returned at x10 power.
      2 = temp in deg F
      1 = temp in deg C
      0 = raw chip value 0-4095
    */

  os_printf("%d", value);
}

int on = 0;

char spidata[16];
char status[64];
int maxDevices;

#define CS 12
#define MOSI 13
#define CLK	 14

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val)
{
	uint8_t i;

	for (i = 0; i < 8; i++)  {
		if (bitOrder == LSBFIRST)
			digitalWrite(dataPin, !!(val & (1 << i)));
		else	
			digitalWrite(dataPin, !!(val & (1 << (7 - i))));
			
		digitalWrite(clockPin, HIGH);
		digitalWrite(clockPin, LOW);		
	}
}

void spiTransfer(int addr, volatile char opcode, volatile char data) {
    //Create an array with the data to shift out
    int offset=addr*2;
    int maxbytes=maxDevices*2;
    int i;

    for(i=0;i<maxbytes;i++)
        spidata[i]=0;
    //put our device data into the array
    spidata[offset+1]=opcode;
    spidata[offset]=data;
    //enable the line 
    digitalWrite(CS,LOW);
    //Now shift out the data 
    for(i=maxbytes;i>0;i--)
        shiftOut(MOSI, CLK, MSBFIRST, spidata[i-1]);
    //latch the data onto the display
    digitalWrite(CS,HIGH);
}    

void clearDisplay(int addr) {
    int offset;
    int i;

    if(addr<0 || addr>=maxDevices)
        return;
    offset=addr*8;

    for(i=0;i<8;i++) {
        status[offset+i]=0;
        spiTransfer(addr, i+1,status[offset+i]);
    }
}

void setLed(int addr, int row, int column, bool state) {
    int offset;
    char val=0x00;

    if(addr<0 || addr>=maxDevices)
        return;
    if(row<0 || row>7 || column<0 || column>7)
        return;
    offset=addr*8;
    val=0b10000000 >> column;
    if(state)
        status[offset+row]=status[offset+row]|val;
    else {
        val=~val;
        status[offset+row]=status[offset+row]&val;
    }
    spiTransfer(addr, row+1,status[offset+row]);
}

void setScanLimit(int addr, int limit) {
    if(addr<0 || addr>=maxDevices)
        return;
    if(limit>=0 && limit<8)
        spiTransfer(addr, OP_SCANLIMIT,limit);
}

void shutdown(int addr, bool b) {
    if(addr<0 || addr>=maxDevices)
        return;
    if(b)
        spiTransfer(addr, OP_SHUTDOWN,0);
    else
        spiTransfer(addr, OP_SHUTDOWN,1);
}

void setIntensity(int addr, int intensity) {
    if(addr<0 || addr>=maxDevices)
        return;
    if(intensity>=0 && intensity<16)	
        spiTransfer(addr, OP_INTENSITY,intensity);
}

void lightemup()
{
  int d;
  int x;
  int y;

  for(d = 0; d < maxDevices; d++) {
    for(x = 0; x < 8; x++) {
      for(y = 0; y < 8; y++) {
        setLed(d, x, y, true);
        os_delay_us(9000);
      }
    }
  }
}

void dbgMessageTimeout2(void)
{
  char msg[] = "tick....\n\r";
  uart0_tx_buffer(msg, sizeof(msg));
}

void user_init(void)
{
    //ets_wdt_disable();


  os_printf("Starting up");

  uart_init(115200, 115200);

  os_printf("setting timers");

  os_timer_disarm(&dbgTimer);
  os_timer_setfn(&dbgTimer, (os_timer_func_t *)dbgMessageTimeout2, NULL);
  os_timer_arm(&dbgTimer, 1000, 1);

  int i;
  int numDevices = 4;

  if(numDevices<=0 || numDevices>8 )
    numDevices = 8;

  maxDevices = numDevices;

  os_printf("declaring numbers of devices. numDevices: %d, maxDevices: %d", numDevices, maxDevices);

  pinMode(MOSI, OUTPUT);
  pinMode(CLK, OUTPUT);
  pinMode(CS, OUTPUT);

  digitalWrite(CS, HIGH);

  for(i=0;i<64;i++) 
    status[i]=0x00;

  for(i=0;i<maxDevices;i++) {
    spiTransfer(i, OP_DISPLAYTEST, 0);
    //scanlimit is set to max on startup
    setScanLimit(i, 7);
    //decode is done in source
    spiTransfer(i, OP_DECODEMODE, 0);
    os_printf("clearing %d\n", i);
    clearDisplay(i);
    setIntensity(i, 7);
    //we go into shutdown-mode on startup
    shutdown(i, true);
    shutdown(i, false);
  }

  lightemup();
}
