#ifndef ESPWIRE_H
#define ESPWIRE_H

#include "Arduino.h"

class espwire {
public:
  espwire(int clearPin, int latchPin, int clockPin, int GPin, int dataPin, int inputPin);
  void begin();
  void setSelectionPins(int pin1, int pin2, int pin3, int pin4, int enb1, int enb2, int enb3, int enb4);
  void setWireSetActive(uint64_t activeWires);
  void selectWire(uint16_t wireNumber); // Method to select a wire using mux
  void exciteTPIC(uint64_t data); // Method to excite TPIC6C595 with data

  bool isWirePass(int wireNumber); // Method to check if a wire passes the test
  bool isWireDefective(int wireNumber); // Method to check if a wire is defective
  bool isWireShortCircuited(int wireNumber); // Method to check if a wire is short-circuited
  
private:
  static void IRAM_ATTR handleInterrupt();
  void shiftOut64(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint64_t val);

  int clearPin;
  int latchPin;
  int clockPin;
  int dataPin;
  int GPin;
  int inputPin; // Renamed from pin1 to inputPin
  int selPins[4];
  int enb[4];
  uint64_t activeWires; // each bit in this variable represents a wire with 1 meaning current is passing through and 0 means it is connected to GND
  
  static volatile int w;
  static volatile unsigned long lastInterruptTime;
};

#endif // ESPWIRE_H
