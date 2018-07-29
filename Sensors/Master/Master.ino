#include "Arduino.h"

#include "RF24Network.h"
#include "RF24.h"
#include "RF24Mesh.h"
#include <SPI.h>
//Include eeprom.h for AVR (Uno, Nano) etc. except ATTiny
#include <EEPROM.h>

#include "SSD1306Ascii.h"
#include "SSD1306AsciiSpi.h"

#include <avr/wdt.h>

// pin definitions
#define OLED_DC 6
#define OLED_CS 7
#define OLED_RST 8

#define DEBUG

#ifdef DEBUG
    #define DEBUG_BEGIN(x)    Serial.begin(x)
    #define DEBUG_PRINT(x)    Serial.print(x)
    #define DEBUG_PRINTLN(x)  Serial.println(x)
    #define DEBUG_PRINTLN_OCT(x, y)  Serial.println(x, y)
    #define OLED_CLR()        oled.clear()
    #define OLED_PRINT(x)     oled.print(x)
    #define OLED_PRINTLN(x)   oled.println(x)
#else
    #define DEBUG_BEGIN(x)
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
    #define OLED_CLR()
    #define OLED_PRINT(x)
    #define OLED_PRINTLN(x)
#endif


SSD1306AsciiSpi oled;

/***** Configure the chosen CE,CS pins *****/
RF24 radio(10, 5);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

char received_packet[] = "000-0\n";
char temperature_packet[] = "000-00.00\n";

long long print_timer = 0;
volatile int wdt_counter = 0;

//Watchdog interrupt ISR
ISR(WDT_vect){
  //DON'T FORGET THIS!  Needed for the watch dog timer.  This is called after a watch dog timer timeout - this is the interrupt function called after waking up

	wdt_counter++;
	if (wdt_counter < 225) { // 1800 seconds or 30mins limit
	//if (wdt_counter < 75) { // 600 seconds or 10mins limit
		// start timer again (we are still in interrupt-only mode)
		wdt_reset();
	} else {
		// go for immediate reset
		//WDTCSR = (1<<WDCE) | (1<<WDE);	// Enable the WD Change Bit - configure mode

		//WDTCSR = (24);//change enable and WDE - also resets
		//WDTCSR = (1<<WDE) | (1<<WDP0);	// set reset flag (WDE) and 32ms (WDP0)

		asm volatile ("  jmp 0");
	}
}

void setup() {
	//SETUP WATCHDOG TIMER
	WDTCSR = (24);//change enable and WDE - also resets	0001 1000
	WDTCSR = (33);//prescalers only (also get rid of the WDE and WDCE bit)	//0010 0001
	WDTCSR |= (1<<6);//enable interrupt mode	//0010 0000

    radio.begin();

	radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_250KBPS);
	//radio.setPALevel(RF24_PA_HIGH);
    //radio.setDataRate(RF24_1MBPS);

    oled.begin(&Adafruit128x64, OLED_CS, OLED_DC, OLED_RST);
    oled.setFont(System5x7);
    OLED_CLR();
    oled.set2X();

    OLED_PRINTLN("Syncing");
    OLED_PRINTLN("with PC");

    Serial.begin(115200);

    // Set the nodeID to 0 for the master node
    mesh.setNodeID(0);

    //DEBUG_PRINTLN(mesh.getNodeID());
    // Connect to the mesh
    mesh.begin();

    /*
    while(1) {
        Serial.print("Sync\n");
        delay(100);
        char received_char;
        if (Serial.available()) {
            received_char = Serial.read();

            if (received_char == 'A') {
                break;
            }
        }
    }

    OLED_CLR();
    OLED_PRINTLN("Synced");
    OLED_PRINTLN("with PC!");
    delay(500);
	*/

    OLED_CLR();
    OLED_PRINTLN("**Master**");


}

void loop() {
    // Call mesh.update to keep the network updated
    mesh.update();
    // In addition, keep the 'DHCP service' running on the master node so addresses will
    // be assigned to the sensor nodes
    mesh.DHCP();

    // Check for incoming data from the sensors
    if (network.available()) {
        //DEBUG_PRINTLN("available!");
        RF24NetworkHeader header;
        network.peek(header);
        switch (header.type) {
            // Display the incoming millis() values from the sensor nodes
            case 'M':
                network.read(header, &received_packet, sizeof(received_packet));
                //DEBUG_PRINT("Received: ");
                //DEBUG_PRINT(received_packet);

                Serial.print(received_packet);

                OLED_CLR();
                OLED_PRINTLN("Received: ");
                OLED_PRINTLN(received_packet);

                break;

            case 'N':
            	network.read(header, &temperature_packet, sizeof(temperature_packet));
				//DEBUG_PRINT("Received: ");
				//DEBUG_PRINT(received_packet);

				Serial.print(temperature_packet);

				OLED_CLR();
				OLED_PRINTLN("Received: ");
				OLED_PRINTLN(temperature_packet);

            default:
                network.read(header, 0, 0);
                //DEBUG_PRINTLN(header.type);
                break;
        }
    }

    if (millis() - print_timer > 5000) {
        print_timer = millis();

        //DEBUG_PRINTLN(" ");
        //DEBUG_PRINTLN(F("********Assigned Addresses********"));
        int count = 0;
        for (int i = 0; i < mesh.addrListTop; i++) {
            //DEBUG_PRINT("NodeID: ");
            //DEBUG_PRINT(mesh.addrList[i].nodeID);
            //DEBUG_PRINT(" RF24Network Address: 0");
            //DEBUG_PRINTLN_OCT(mesh.addrList[i].address, OCT);
            count++;
        }

        OLED_CLR();
        OLED_PRINTLN("**Master**");
        OLED_PRINT("Nodes: ");
        OLED_PRINTLN(count);

        //DEBUG_PRINTLN(F("**********************************"));
    }
}

