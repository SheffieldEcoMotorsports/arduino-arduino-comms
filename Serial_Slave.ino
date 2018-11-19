#include <SoftwareSerial.h>
const byte ledPin = 13;
const byte interruptPin = 2;
volatile byte state = LOW;
unsigned long time;
SoftwareSerial mySerial(10, 11);
void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, RISING);
}

void loop() {
  
}

void blink() {
  mySerial.begin(2400);
  //state = !state;
  time = millis();
  String input_;//if (mySerial.available()) 
  char ch;
  mySerial.println(time);
  //delay(2);  //slow looping to allow buffer to fill with next character
  mySerial.end();
}
