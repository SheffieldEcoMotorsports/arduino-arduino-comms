#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <SoftwareSerial.h>
// Pins for thermocouple
#define CS 10
#define MISO 12
#define SCLK 13

const byte interruptPin = 2;
unsigned long time;
String output,out;
//int current = 2;
int val = 0;

SoftwareSerial BTserial(5,6);
Adafruit_MAX31855 Temp(SCLK, CS, MISO); // Creation of the thermocouple object

void setup() {
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), acquisition, RISING);

  // Bluetooth Set Up
  BTserial.begin(9600); //Start with BTbaudRate
  BTserial.print("AT");   //Check connection (Should return msg "OK")
  delay(1000);      //Backoff time of 1 sec as per datasheet
  if (BTserial.available() > 0)
  {
    while (BTserial.available() > 0) {
      char c = BTserial.read();
    }
  }
  BTserial.print("AT+NAMECereal_bus");   //Set name (Should return msg "OKsetname")
  delay(1000);    //Backoff time to receive msg
  if (BTserial.available() > 0)
  {
    while (BTserial.available() > 0) {
      char c = BTserial.read();
    }
  }
  BTserial.print("AT+PIN5678");       //Set pin (Should return msg "OKsetPIN");
  delay(1000);//Backoff time of 1 sec as per datasheet
  if (BTserial.available() > 0)
  {
    while (BTserial.available() > 0) {
      char c = BTserial.read();
    }
  }
}

void loop() {
  String  current, thermocouple, bluetooth;
  current = Currentfcn();
  //thermistor=thermistor();
  thermocouple = Thermocouplefcn();
  bluetooth = Bluetoothfcn();
  out = "[" + current + "," + thermocouple + "," + bluetooth + "]";
}

String Bluetoothfcn() {
  if (BTserial.available() > 0) { // triggered if something new is printed in bluetooth COM port
   String Str = "";
    while (BTserial.available() > 0) {
      // read the newest byte in the serial buffer:
     char incomingControl = BTserial.read();
      Str = Str + incomingControl;
    }
    //If serial reads 1, car moves backwards
    // *** PLEASE NOTE all of the motion commands are reversed due to the configuration of the remote and the set up of motors***
    //if (incomingControl == '1')
    
  }
}


String Currentfcn() {
  /*
    float volt;
    int val = 0;
    //state = !state;
    time = millis();
    val = analogRead(analogPin);
    volt=(5*val)/1023;
    return(String(volt));
  */
  int dummy = random(0, 100);
  return (String(dummy));
}

String Thermocouplefcn() {
  float  Temperature=Temp.readCelsius(); // Acquisition of temperature in degrees  Celsius
  delay(1000);
  return (String(Temperature));
}

void acquisition() {
  Serial.begin(9600);
  //state = !state;
  Serial.print(out);
  //delay(2);  //slow looping to allow buffer to fill with next character
  Serial.end();
}
