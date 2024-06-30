#include "espwire.h"

volatile int espwire::w = 0;
volatile unsigned long espwire::lastInterruptTime = 0;

espwire::espwire(int clearPin, int latchPin, int clockPin, int GPin, int dataPin, int inputPin)
: clearPin(clearPin), latchPin(latchPin), clockPin(clockPin), dataPin(dataPin), GPin(GPin), inputPin(inputPin), activeWires(0x00) {}

void espwire::setSelectionPins(int pin1, int pin2, int pin3, int pin4, int enb1, int enb2, int enb3, int enb4) {
  selPins[0] = pin1;
  selPins[1] = pin2;
  selPins[2] = pin3;
  selPins[3] = pin4;

  enb[0] = enb1;
  enb[1] = enb2;
  enb[2] = enb3;
  enb[3] = enb4;
}

void espwire::setWireSetActive(uint64_t activeWires) {
  this->activeWires = activeWires;
}

void espwire::begin() {
  pinMode(clearPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

  for (int i = 0; i < 4; i++) {
    pinMode(selPins[i], OUTPUT);
    pinMode(enb[i], OUTPUT);
  }

  pinMode(inputPin, INPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(inputPin), handleInterrupt, RISING);

  const int freq = 5000;
  const int ledChannel = 0;
  const int resolution = 8;

  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(GPin, ledChannel);

  digitalWrite(clearPin, HIGH);
  ledcWrite(0, 240);

  exciteTPIC(activeWires);
  
  for (int i = 0; i < 4; i++) {
    digitalWrite(selPins[i], LOW);
    digitalWrite(enb[i], HIGH); //initial no mux is selected 
  }
}

void espwire::selectWire(uint16_t wireNumber) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(selPins[i], ((wireNumber % 16) >> i) & 0x01);
    digitalWrite(enb[i], (((wireNumber / 16)==i)?LOW:HIGH));
  }
}

void espwire::exciteTPIC(uint64_t data) {
  digitalWrite(latchPin, LOW);
  shiftOut64(dataPin, clockPin, LSBFIRST, data);
  digitalWrite(latchPin, HIGH);
}

bool espwire::isWirePass(int wireNumber) {
  selectWire(wireNumber);
  exciteTPIC(activeWires);
  return digitalRead(inputPin) == HIGH;
}

bool espwire::isWireDefective(int wireNumber) {
  selectWire(wireNumber);
  exciteTPIC(activeWires);
  return digitalRead(inputPin) == LOW;
}

bool espwire::isWireShortCircuited(int wireNumber) {
  selectWire(wireNumber);
  exciteTPIC(0xFFFFFFFFFFFFFFFF^(1<<(wireNumber-1))); // Set all wires to high exepect the one to be tested 
  return digitalRead(inputPin) == HIGH;
}

void IRAM_ATTR espwire::handleInterrupt() {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime > 50) { // Debouncing time
    w++;
    lastInterruptTime = currentTime;
  }
}

void espwire::shiftOut64(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint64_t val) {
  for (int i = 0; i < 64; i++) {
    if (bitOrder == LSBFIRST) {
      digitalWrite(dataPin, !!(val & (1ULL << i)));
    } else {
      digitalWrite(dataPin, !!(val & (1ULL << (63 - i))));
    }
    digitalWrite(clockPin, HIGH);
    delayMicroseconds(1);
    digitalWrite(clockPin, LOW);
    delayMicroseconds(1);
  }
}
