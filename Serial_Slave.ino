#include <SoftwareSerial.h>

#define rxPin 10
#define txPin 11
SoftwareSerial mySerial =  SoftwareSerial(rxPin, txPin);

int IntPin = 2;

void setup()
{
  Serial.begin(2400);
  Serial.println("Starting..");
  mySerial.begin(2400);
  pinMode(IntPin, OUTPUT);
  digitalWrite(IntPin, LOW);
}

void loop()
{

  digitalWrite(IntPin, HIGH);        // sets the digital pin 13 off
  
  while (mySerial.available())
 {
   char c = mySerial.read();
   Serial.print(c);
 }

  digitalWrite(IntPin, LOW);        // sets the digital pin 13 off
  delay(5000);

}
