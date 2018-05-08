/*
//this code successfully logs gps data to a file, but is messy

#include <SoftwareSerial.h>
#include <SD.h>
#include <SPI.h>

//Information on protocols used:
//http://www.gpsinformation.org/dale/nmea.htm
const int gpsSentence = 128;
char gpsData[gpsSentence];      //Holds content of GPS data received
const int chipSelect = BUILTIN_SDCARD;
File flightData;

void setup() {
  Serial.begin(9600); //USB serial
  while (!Serial) {
    ; // endless loop to wait for the damn thing to connect. REMOVE FROM FINAL IMPLEMENTATION
  }

  Serial.println("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  //GPS initialization
  Serial3.begin(9600,SERIAL_8N1); //uart3 pins 7 and 8 to GPS
  Serial.println("Entering startup time...");
  delay(5000); //wait 5 seconds for warm up to finish
  memset(gpsData, '\0', sizeof(char)*128);
  Serial.println("GPS initialization complete."); //Show serial monitor that dinner's ready
}

//REAL SHIT
void loop() {
  
  char gps_output;
  
  //While data is available
  if(Serial3.available()){
      gps_output = Serial3.read();
       // open the file. note that only one file can be open at a time,
       // so you have to close this one before opening another.
      File dataFile = SD.open("log5.txt", FILE_WRITE);

      // if the file is available, write to it:
      if (dataFile) {
        dataFile.print(gps_output);
        dataFile.close();
      // print to the serial port too:
      Serial.print(gps_output);
      }
      // if the file isn't open, pop up an error:
      else {
        Serial.println("error opening file");
      }
   }
}
*/





//this code successfully logs gps data to a file, but is messy

#include <SoftwareSerial.h>
#include <SD.h>
#include <SPI.h>

//Information on protocols used:
//http://www.gpsinformation.org/dale/nmea.htm
const int gpsSentence = 128;
char gpsData[gpsSentence];      //Holds content of GPS data received
const int chipSelect = BUILTIN_SDCARD;
File flightData;

void setup() {
  Serial.begin(9600); //USB serial
  while (!Serial) {
    ; // endless loop to wait for the damn thing to connect. REMOVE FROM FINAL IMPLEMENTATION
  }

  Serial.println("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  //GPS initialization
  Serial3.begin(9600,SERIAL_8N1); //uart3 pins 7 and 8 to GPS
  Serial.println("Entering startup time...");
  delay(5000); //wait 5 seconds for warm up to finish
  memset(gpsData, '\0', sizeof(char)*128);
  Serial.println("GPS initialization complete."); //Show serial monitor that dinner's ready
}

//REAL SHIT
void loop() {
  
  char gps_output;
  
  //While data is available
  if(Serial3.available()){
      gps_output = Serial3.read();
       // open the file. note that only one file can be open at a time,
       // so you have to close this one before opening another.
      File dataFile = SD.open("log5.txt", FILE_WRITE);

      // if the file is available, write to it:
      if (dataFile) {
        dataFile.print(gps_output);
        if(gps_output == '$'){ //new
          Serial.print('\n');  //new
          dataFile.print('\n');//new
        }
        dataFile.close();
      // print to the serial port too:
      Serial.print(gps_output);
      }
      // if the file isn't open, pop up an error:
      else {
        Serial.println("error opening file");
      }
   }
}



