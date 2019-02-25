#include <SPI.h>
#include "Adafruit_MAX31855.h"

// Pins for thermocouple
#define CS 10
#define MISO 12
#define SCLK 13

const byte interruptPin = 2;
unsigned long time;
String output;
int current = 2;
int val = 0;

Adafruit_MAX31855 Temp(SCLK, CS, MISO); // Creation of the thermocouple object

void setup() {
  Serial.begin(9600);
  Serial.println("test");
  pinMode(ledPin, OUTPUT);
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
      Serial.write(c);
    }
  }
  BTserial.print("AT+NAMECereal_bus");   //Set name (Should return msg "OKsetname")
  delay(1000);    //Backoff time to receive msg
  if (BTserial.available() > 0)
  {
    while (BTserial.available() > 0) {
      char c = BTserial.read();
      Serial.write(c);
    }
  }
  BTserial.print("AT+PIN5678");       //Set pin (Should return msg "OKsetPIN");
  delay(1000);//Backoff time of 1 sec as per datasheet
  if (BTserial.available() > 0)
  {
    while (BTserial.available() > 0) {
      char c = BTserial.read();
      Serial.write(c);
    }
  }
}

void loop() {
  String out, curent, thermocouple, bluetooth;
  current = current();
  //thermistor=thermistor();
  thermocouple = thermocouple();
  bluetooth = bluetooth();
  out = "[" + current + "," + thermocouple + "," + bluetooth + "]";
  Serial.println(out);
}

String bluetooth() {
  if (BTserial.available() > 0) { // triggered if something new is printed in bluetooth COM port
    Str = "";
    while (BTserial.available() > 0) {
      // read the newest byte in the serial buffer:
      incomingControl = BTserial.read();
      Str = Str + incomingControl;
    }
    Serial.println(Str);
    //Serial.println(Index1);
    //Serial.println(incomingControl);
    //If serial reads 1, car moves backwards
    // *** PLEASE NOTE all of the motion commands are reversed due to the configuration of the remote and the set up of motors***
    //if (incomingControl == '1')
    speed_display.showNumberDecEx(Str.toInt(), B01000000, false);
  }
}


String current() {
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
  return (String(dummy))
}

String thermocouple() {
  Temperature=Temp.readCelsius(); // Acquisition of temperature in degrees  Celsius
  delay(1000);
  return (String(Temperature));
}

void interruptFcn() {
  mySerial.begin(9600);
  //state = !state;
  mySerial.print(serialSend);
  //delay(2);  //slow looping to allow buffer to fill with next character
  mySerial.end();
}
