#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h> //include the SD library
#include <Arduino.h>
#include <TM1637Display.h>

// Module connection pins (Dig#include <SoftwareSerial.h>
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


//Arduino 2 uses normal serial & Arduino 3 uses software serial
#define rxPin2 14 //Arduino 3 rx
#define txPin2 15 //Arduino 3 tx
#define ssPin 10 //SD Card 
#define IntPin3 3//Interrupt for Arduino 3
#define IntPin2 2//Interrupt for Arduino 2
SoftwareSerial mySerial =  SoftwareSerial(rxPin2, txPin2); //Arduino 3 comms

bool sdInitSuccess = false; //card init status
File myFile;
String Str2 = "";
String Str3 = "";

TM1637Display speed_display(DISPLAY_CLK, SPEED_DISPLAY_DIO);  //set up the speed display
TM1637Display batt_display(DISPLAY_CLK, BATT_DISPLAY_DIO);  //set up the battery display
TM1637Display lap_display(DISPLAY_CLK, LAP_DISPLAY_DIO);  //set up the lap display
TM1637Display time_display(DISPLAY_CLK, TIME_DISPLAY_DIO);  //set up the stopwatch display

void setup()
{
  batt_display.setBrightness(0x0f);

  // To arduino 2
  Serial.begin(9600); 
  pinMode(IntPin2, OUTPUT);
  digitalWrite(IntPin2, LOW);

  // To arduino 3
  mySerial.begin(9600); 
  pinMode(IntPin3, OUTPUT);
  digitalWrite(IntPin3, LOW);
  
  if(SD.begin(ssPin))
  {
    sdInitSuccess = true;
  }
  
  if (SD.exists("TEST.txt"))
  {
    SD.remove("TEST.txt");
  }
  myFile = SD.open("TEST.txt", FILE_WRITE);
  myFile.println("Time,BatLvl,Speed,Lat,Log,AccX,AccY,AccZ,Pitch,Yaw,Roll,Current,Temp");
  myFile.close();
  
}

void loop()
{

  /// Arduino 3 //// 
  Str3 = "";
  digitalWrite(IntPin3, HIGH);        // sets the digital pin 3 off
  myFile = SD.open("TEST.txt", FILE_WRITE);
  
  while (mySerial.available())
 {
   char c = mySerial.read(); 
   Str3.concat(c);
 }
  digitalWrite(IntPin3, LOW);        // sets the digital pin 13 off
  delay(500);

  int Str3CommaIndex1 = Str3.indexOf(",");//Lat
  int Str3CommaIndex2 = Str3.indexOf(",",Str3CommaIndex1+1);//Long
  int Str3CommaIndex3 = Str3.indexOf(",",Str3CommaIndex2+1);//Speed
  int Str3CommaIndex4 = Str3.indexOf(",",Str3CommaIndex3+1);//Acc1
  int Str3CommaIndex5 = Str3.indexOf(",",Str3CommaIndex4+1);//Acc2
  int Str3CommaIndex6 = Str3.indexOf(",",Str3CommaIndex5+1);//Acc3
  int Str3CommaIndex7 = Str3.indexOf(",",Str3CommaIndex6+1);//Pos1 - Pitch
  int Str3CommaIndex8 = Str3.indexOf(",",Str3CommaIndex7+1);//Pos2 - Yaw
  //Pos3 - Roll

  int Lat = (Str3.substring(1,Str3CommaIndex1)).toInt();
  int Long = (Str3.substring(Str3CommaIndex1+1,Str3CommaIndex2)).toInt();
  int Speed = (Str3.substring(Str3CommaIndex2+1,Str3CommaIndex3)).toInt();
  int AccX = (Str3.substring(Str3CommaIndex3+1,Str3CommaIndex4)).toInt();
  int AccY = (Str3.substring(Str3CommaIndex4+1,Str3CommaIndex5)).toInt();
  int AccZ = (Str3.substring(Str3CommaIndex5+1,Str3CommaIndex6)).toInt();
  int Pitch = (Str3.substring(Str3CommaIndex6+1,Str3CommaIndex7)).toInt();
  int Yaw = (Str3.substring(Str3CommaIndex7+1,Str3CommaIndex8)).toInt();
  int Roll = (Str3.substring(Str3CommaIndex8+1,Str3.length()-1)).toInt();
  
  speed_display.showNumberDec(Speed, false);
  

/// Arduino 2 //// 
  Str2 = "";
  digitalWrite(IntPin2, HIGH);        // sets the digital pin 2 high
  
  while (Serial.available())
 {
   char c = Serial.read(); 
   Str2.concat(c);
 }
  digitalWrite(IntPin2, LOW);        // sets the digital pin 2 low
  delay(500);

  int Str2CommaIndex1 = Str2.indexOf(",");//Current
  int Str2CommaIndex2 = Str2.indexOf(",",Str2CommaIndex1+1);//Temp
  //BatLvl

  int Current = (Str2.substring(1,Str2CommaIndex1)).toInt();
  int Temp = (Str2.substring(Str2CommaIndex1+1,Str2CommaIndex2)).toInt();
  int BatLvl = (Str2.substring(Str2CommaIndex2+1,Str2.length()-1)).toInt();
  batt_display.showNumberDec(BatLvl, false);


  int Time = millis();
  //SD Card Logging
  //myFile.println("Time,BatLvl,Speed,Lat,Log,AccX,AccY,AccZ,Pitch,Yaw,Roll,Current,Temp");
  myFile.println(String(Time)+","+String(BatLvl)+","+String(Speed)+","+String(Lat)+","+String(Long)+","+String(AccX)+","+String(AccY)+","+String(AccZ)+","+String(Pitch)+","+String(Yaw)+","+String(Roll)+","+String(Current)+","+String(Temp));

  myFile.close();
}ital Pins)
#define DISPLAY_CLK 8 
#define SPEED_DISPLAY_DIO 4 //speed display
#define BATT_DISPLAY_DIO 5 //battery display
#define LAP_DISPLAY_DIO 6 //lap number display
#define TIME_DISPLAY_DIO 7 //time display


//Arduino 2 uses normal serial & Arduino 3 uses software serial
#define rxPin2 14 //Arduino 3 rx
#define txPin2 15 //Arduino 3 tx
#define ssPin 10 //SD Card 
#define IntPin3 3//Interrupt for Arduino 3
#define IntPin2 2//Interrupt for Arduino 2
SoftwareSerial mySerial =  SoftwareSerial(rxPin2, txPin2); //Arduino 3 comms

bool sdInitSuccess = false; //card init status
File myFile;
String Str2 = "";
String Str3 = "";

TM1637Display speed_display(DISPLAY_CLK, SPEED_DISPLAY_DIO);  //set up the speed display
TM1637Display batt_display(DISPLAY_CLK, BATT_DISPLAY_DIO);  //set up the battery display
TM1637Display lap_display(DISPLAY_CLK, LAP_DISPLAY_DIO);  //set up the lap display
TM1637Display time_display(DISPLAY_CLK, TIME_DISPLAY_DIO);  //set up the stopwatch display

void setup()
{
  batt_display.setBrightness(0x0f);

  // To arduino 2
  Serial.begin(9600); 
  pinMode(IntPin2, OUTPUT);
  digitalWrite(IntPin2, LOW);

  // To arduino 3
  mySerial.begin(9600); 
  pinMode(IntPin3, OUTPUT);
  digitalWrite(IntPin3, LOW);
  
  if(SD.begin(ssPin))
  {
    sdInitSuccess = true;
  }
  
  if (SD.exists("TEST.txt"))
  {
    SD.remove("TEST.txt");
  }
  myFile = SD.open("TEST.txt", FILE_WRITE);
  myFile.println("Time,BatLvl,Speed,Lat,Log,AccX,AccY,AccZ,Pitch,Yaw,Roll,Current,Temp");
  myFile.close();
  
}

void loop()
{

  /// Arduino 3 //// 
  Str3 = "";
  digitalWrite(IntPin3, HIGH);        // sets the digital pin 3 off
  myFile = SD.open("TEST.txt", FILE_WRITE);
  
  while (mySerial.available())
 {
   char c = mySerial.read(); 
   Str3.concat(c);
 }
  digitalWrite(IntPin3, LOW);        // sets the digital pin 13 off
  delay(500);

  int CommaIndex1 = Str3.indexOf(",");//Lat
  int CommaIndex2 = Str3.indexOf(",",CommaIndex1+1);//Long
  int CommaIndex3 = Str3.indexOf(",",CommaIndex2+1);//Speed
  int CommaIndex4 = Str3.indexOf(",",CommaIndex3+1);//Acc1
  int CommaIndex5 = Str3.indexOf(",",CommaIndex4+1);//Acc2
  int CommaIndex6 = Str3.indexOf(",",CommaIndex5+1);//Acc3
  int CommaIndex7 = Str3.indexOf(",",CommaIndex6+1);//Pos1 - Pitch
  int CommaIndex8 = Str3.indexOf(",",CommaIndex7+1);//Pos2 - Yaw
  //Pos3 - Roll

  int Lat = (Str3.substring(1,CommaIndex1)).toInt();
  int Long = (Str3.substring(CommaIndex1+1,CommaIndex2)).toInt();
  int Speed = (Str3.substring(CommaIndex2+1,CommaIndex3)).toInt();
  int AccX = (Str3.substring(CommaIndex3+1,CommaIndex4)).toInt();
  int AccY = (Str3.substring(CommaIndex4+1,CommaIndex5)).toInt();
  int AccZ = (Str3.substring(CommaIndex5+1,CommaIndex6)).toInt();
  int Pitch = (Str3.substring(CommaIndex6+1,CommaIndex7)).toInt();
  int Yaw = (Str3.substring(CommaIndex7+1,CommaIndex8)).toInt();
  int Roll = (Str3.substring(CommaIndex8+1,Str2.length()-1)).toInt();
  
  speed_display.showNumberDec(Speed, false);
  

/// Arduino 2 //// 
  Str2 = "";
  digitalWrite(IntPin2, HIGH);        // sets the digital pin 2 high
  
  while (Serial.available())
 {
   char c = Serial.read(); 
   Str2.concat(c);
 }
  digitalWrite(IntPin2, LOW);        // sets the digital pin 2 low
  delay(500);

  int CommaIndex1 = Str2.indexOf(",");//Current
  int CommaIndex2 = Str2.indexOf(",",CommaIndex1+1);//Temp
  //BatLvl

  int Current = (Str2.substring(1,CommaIndex1)).toInt();
  int Temp = (Str2.substring(CommaIndex1+1,CommaIndex2)).toInt();
  int BatLvl = (Str2.substring(CommaIndex2+1,Str2.length()-1)).toInt();
  batt_display.showNumberDec(BattLvl, false);


  int Time = millis();
  //SD Card Logging
  //myFile.println("Time,BatLvl,Speed,Lat,Log,AccX,AccY,AccZ,Pitch,Yaw,Roll,Current,Temp");
  myFile.println(Time+","+BatLvl+","+Speed+","+Lat+","+Log+","+AccX+","+AccY+","+AccZ+","+Pitch+","+Yaw+","+Roll+","+Current+","+Temp);

  myFile.close();
}
