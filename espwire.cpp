#include "espwire.h"

espwire::espwire(int clearPin, int latchPin, int clockPin, int GPin, int dataPin, int inputPin)
: clearPin(clearPin), latchPin(latchPin), clockPin(clockPin), dataPin(dataPin), GPin(GPin), inputPin(inputPin),testDigits(0) {}

void espwire::setSelectionPins(int pin1, int pin2, int pin3, int pin4, int enb1, int enb2, int enb3, int enb4)
{
    selPins[0] = pin1;
    selPins[1] = pin2;
    selPins[2] = pin3;
    selPins[3] = pin4;

    enb[0] = enb1;
    enb[1] = enb2;
    enb[2] = enb3;
    enb[3] = enb4;
}

void espwire::begin() {
  unsigned long t = 0;
  volatile int w = 0;
  unsigned long prev = 0;
  bool test = false;

  pinMode(clearPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

  for (int i = 0; i < 4; i++) {
    pinMode(selecPins[i], OUTPUT);
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

  digitalWrite(latchPin, LOW);
  shiftOut64(dataPin, clockPin, LSBFIRST, testDigits);
  digitalWrite(latchPin, HIGH);

  for (int i = 0; i < numSelPins; i++) {
    digitalWrite(selPins[i], LOW);
  }
}

void espwire::update() {
  unsigned long t = 0;
  volatile int w = 0;
  unsigned long prev = 0;
  bool test = false;

  digitalWrite(latchPin, HIGH);
  t = millis();

  if (t - prev > 2000) {
    if (w < 2) {
      Serial.println("wire have a prob");
      test = true;
    } else {
      Serial.println("wire is okey");
      test = false;
    }
    prev = t;
    w = 0;
  }

  Serial.println(test ? "wire have a prob" : "wire is okey");
  Serial.println(digitalRead(inputPin));
}

void espwire::selectWire(uint16_t wireNumber) {
    for (int i = 0; i < 4; i++)
    {
        digitalWrite(selecPins[i], ((wireNumber % 16) >> i) & 0x01);
        digitalWrite(enb[i], ((wireNumber / 16) >> i) & 0x01);
    }
}

void espwire::exciteTPIC(uint64_t data) {
  digitalWrite(latchPin, LOW);
  shiftOut64(dataPin, clockPin, LSBFIRST, data);
  digitalWrite(latchPin, HIGH);
}

void IRAM_ATTR espwire::handleInterrupt() {
  if (digitalRead(inputPin) == HIGH) {
    w++;
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
