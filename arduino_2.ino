#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <SoftwareSerial.h>
// Pins for thermocouple
#define CS 10
#define MISO 12
#define SCLK 13

const byte interruptPin = 2;
unsigned long time;
String output,out, LastBatteryValue = "100";
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
  if(bluetooth != "No value")
  {
    LastBatteryValue = bluetooth;
  }
  out = "[" + current + "," + thermocouple + "," + LastBatteryValue + "]";
}

String Bluetoothfcn() {
  String Str = "";
  if (BTserial.available() > 0) { // triggered if something new is printed in bluetooth COM port
   char incomingControl = "";
    while (BTserial.available() > 0) {
      // read the newest byte in the serial buffer:
     incomingControl = BTserial.read();
     if(incomingControl != '\n')
     {
      Str = Str + incomingControl;
     }
     else
     {
      return(Str);
     }       
    }
  }   
  else
  {
  return("No value");
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
  Serial.println(out);
  //delay(2);  //slow looping to allow buffer to fill with next character
  Serial.end();
}
