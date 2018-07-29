#include "Arduino.h"
//The setup function is called once at startup of the sketch

//******************************************************************************************//
/*This code measures the temperature using the DS18b20 sensor and sends the
output to a LCD and over serial interface to the computer*/
//******************************************************************************************//
#include <OneWire.h>
#include <DallasTemperature.h>

float Temp = 0; //Var to store the value of the temp received from the sensor
#define OneWire_bus A3 //Data bus is connected to digital pin 6 of the arduino
OneWire oneWire(OneWire_bus); //Setup a OneWire instance to comm with any OneWire device
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature.

void setup()
{
    pinMode(A4, OUTPUT);
    digitalWrite(A4, HIGH);

	sensors.begin(); //Initialize the sensor
	Serial.begin(9600); //Initialize the serial communication at 9600 baud
}

void loop()
{
sensors.requestTemperatures(); //Request the temperature & store it in memory
delay(1000);
Temp = sensors.getTempCByIndex(0); //Get the temp from the memory

//Output the temperature over the serial interface
Serial.print(Temp);
Serial.println(" C ");
}
