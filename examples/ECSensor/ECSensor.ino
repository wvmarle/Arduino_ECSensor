/*
 * Basic test sketch for the ECSensor libraries.
 * 
 * Which library to use, and how to connect the sensor, is microcontroller dependent!
 * Currently supported processors:
 *  - ATmega328p (a.o. Arduino Uno, Micro, Nano, Pro Mini)
 *  - ESP8266 (a.o. NodeMCU, WeMOS D1)
 */




#include "EC_ATmega.h"
// Pin connections for the ATmega:
// CP / CapPos: 2
// CN / CapNeg: 4
// EC:          7
// These pins are hard coded for efficiency and speed.
// CapPos is chosen for its external interrupt; the other two pins for not having any special purpose.

ECSensor sensor;

void setup() {
  sensor.begin();
  Serial.begin(115200);
}

uint16_t i = 0;
void loop() {
  Serial.print ("test: ");
  Serial.println (i++);
  Serial.print("Discharge cycles: ");
  Serial.print(sensor.readSensor());
  Serial.println();
  delay(2000);
}

