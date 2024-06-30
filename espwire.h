#ifndef ESPWIRE_H
#define ESPWIRE_H

#include "Arduino.h"

class espwire {
public:
  espwire(int clearPin, int latchPin, int clockPin, int GPin, int dataPin, int inputPin);
  void begin();
  void update();
  void setSelectionPins(int pin1,int pin2,int pin3,int pin4,int enb1,int enb2,int enb3, int enb4);
  void selectWire(uint16_t wireNumber); // Method to select a wire using mux
  void exciteTPIC(uint64_t data); // Method to excite TPIC6C595 with data
  
private:
  static void IRAM_ATTR handleInterrupt();
  void shiftOut64(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint64_t val);

  int clearPin;
  int latchPin;
  int clockPin;
  int dataPin;
  int GPin;
  int inputPin; // Renamed from pin1 to inputPin
  int numSelPins;
  uint64_t testDigits; // Updated to uint64_t for 64 bits of data
  int selecPins[4];
  int enb[4];
};

#endif // ESPWIRE_H
//courtcircuit fail(mch mwjoud) pass 