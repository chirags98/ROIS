#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2018-07-29 18:44:32

#include "Arduino.h"
#include <avr/wdt.h>
extern uint8_t mcusr_copy;
#include "Arduino.h"
#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#include <SPI.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiSpi.h"
#include <OneWire.h>
#include <DallasTemperature.h>

void sensor_ISR() ;
int transmit_to_master(char str[], char msg_type, int size_of_str) ;
void setup() ;
void loop() ;
ISR(WDT_vect);
void digitalInterrupt();
void sleep() ;
void wake_up() ;
float get_temperature() ;
void send_temperature() ;
void wakeup_sendTemp() ;
void my_wdt_reset() ;
void wdt_first(void) ;
void disable_wdt(void) ;

#include "Node.ino"


#endif
