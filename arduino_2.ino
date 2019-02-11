#include <SPI.h>
const byte interruptPin = 2;
unsigned long time;
String output;
int current = 2; 
int thermistor = 3;
int val = 0;
void setup() {
  Serial.begin(9600);
  Serial.println("test");
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), acquisition, RISING);
}

void loop() {
  String out,curent,thermistor,thermocouple;
  //current=current();
  //thermistor=thermistor();
  thermocouple=thermocouple();
  //out="["+current+","+thermistor+","+thermocouple+"]";
  Serial.println(thermocouple);
}

String current() {
  float volt;
  int val = 0; 
  //state = !state;
  time = millis();
  val = analogRead(analogPin);
  volt=(5*val)/1023;
  return(String(volt));
}
String thermistor() {
  float volt;
  int val = 0,temp; 
  //state = !state;
  time = millis();
  val = analogRead(analogPin);
  volt=(5*val)/1023;
  temp=function(volt);
  return(String(temp));
}
String thermocouple() {
  time = millis();
  int32_t result;
  int16_t T,temp;
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));
  digitalWrite(chipSelectPin, LOW);
  result = SPI.transfer(0x00);
  digitalWrite(chipSelectPin, HIGH);
  temp=result>>18;
  T=(temp<<2)/4;
  return(String(T));
}


