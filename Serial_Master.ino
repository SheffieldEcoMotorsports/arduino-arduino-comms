#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h> //include the SD library
#include <Arduino.h>
#include <TM1637Display.h>

// Module connection pins (Digital Pins)
#define DISPLAY_CLK 8 
#define SPEED_DISPLAY_DIO 4 //speed display
#define BATT_DISPLAY_DIO 5 //battery display
#define LAP_DISPLAY_DIO 6 //lap number display
#define TIME_DISPLAY_DIO 7 //time display

#define rxPin2 14
#define txPin2 15
#define ssPin 10 //SD Card 
#define IntPin2 3//Interrupt 
SoftwareSerial mySerial =  SoftwareSerial(rxPin2, txPin2); //Arduino B comms

bool sdInitSuccess = false; //card init status
File myFile;
String Str1 = "";
String Str2 = "";

//TM1637Display speed_display(DISPLAY_CLK, SPEED_DISPLAY_DIO);  //set up the speed display
TM1637Display batt_display(DISPLAY_CLK, BATT_DISPLAY_DIO);  //set up the battery display
//TM1637Display lap_display(DISPLAY_CLK, LAP_DISPLAY_DIO);  //set up the lap display
//TM1637Display time_display(DISPLAY_CLK, TIME_DISPLAY_DIO);  //set up the stopwatch display

void setup()
{
  batt_display.setBrightness(0x0f);
  mySerial.begin(9600);
  Serial.begin(9600);
  pinMode(IntPin2, OUTPUT);
  digitalWrite(IntPin2, LOW);
  if(SD.begin(ssPin))
  {
    sdInitSuccess = true;
  }
}

void loop()
{
  Str2 = "";
  digitalWrite(IntPin2, HIGH);        // sets the digital pin 3 off
  myFile = SD.open("TEST.txt", FILE_WRITE);
  
  while (mySerial.available())
 {
   char c = mySerial.read(); 
   //Serial.print(c);
   Str2.concat(c);
 }
  digitalWrite(IntPin2, LOW);        // sets the digital pin 13 off
  delay(500);
  myFile.close();

  int CommaIndex1 = Str2.indexOf(",");//Lat
  int CommaIndex2 = Str2.indexOf(",",CommaIndex1+1);//Long
  int CommaIndex3 = Str2.indexOf(",",CommaIndex2+1);//BatLvl
  int CommaIndex4 = Str2.indexOf(",",CommaIndex3+1);//Acc1
  int CommaIndex5 = Str2.indexOf(",",CommaIndex4+1);//Acc2
  int CommaIndex6 = Str2.indexOf(",",CommaIndex5+1);//Acc3
  //Acc4
  
  String BatLvl = Str2.substring(CommaIndex2+1,CommaIndex3);
  int BattLvl = BatLvl.toInt();
  Serial.print(BattLvl);
  batt_display.showNumberDec(BattLvl, false);

//    speed_display.setBrightness(0x0f);
//  
//  lap_display.setBrightness(0x0f);
//  time_display.setBrightness(0x0f);
}
