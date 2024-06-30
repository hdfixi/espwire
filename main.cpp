#include <espwire.h>
#include "Arduino.h"
int selPins[] = {34, 35, 32, 33}; // Example mux selection pins
int enbPins[] = {36, 37, 38, 39}; // Example mux enable pins


// Instantiate espwire object with pin configuration
// clearPin: 18  - Pin for clearing the shift register
// latchPin: 19  - Pin for latching the data into the shift register
// clockPin: 23  - Pin for clock signal to the shift register
// GPin: 21      - Pin for PWM control
// dataPin: 22   - Pin for data input to the shift register
// inputPin: 14  - Pin for reading input signal
espwire myEspWire(18, 19, 23, 21, 22, 14);

void setup() {
  myEspWire.setSelectionPins(selPins[0], selPins[1], selPins[2], selPins[3], enbPins[0], enbPins[1], enbPins[2], enbPins[3]);
  myEspWire.begin();
}

void loop() {
  int wireNumber = 1; // Example wire number to check
  if (myEspWire.isWirePass(wireNumber)) {
    Serial.println("Wire is pass");
  } else if (myEspWire.isWireDefective(wireNumber)) {
    Serial.println("Wire is defective");
  } else if (myEspWire.isWireShortCircuited(wireNumber)) {
    Serial.println("Wire is short-circuited");
  }
  delay(1000); // Delay for testing purposes
}
