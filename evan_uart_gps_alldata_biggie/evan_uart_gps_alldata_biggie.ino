#include <SoftwareSerial.h>
#include <SD.h>
#include <SPI.h>

File flightData;

//Information on protocols used:
//http://www.gpsinformation.org/dale/nmea.htm
const int gpsSentence = 128;
char gpsData[gpsSentence];      //Holds content of GPS data received
char callSign[18] = "$callsign:KI7NXS\n";
char packetID[18] = "$packetID:BIGGIE\n";
//char packetID[18] = "$packetID:SMALLS\n";
const int chipSelect = BUILTIN_SDCARD;
int count = 0;
int callsignCount = 0;

void setup() {
  SD.begin(chipSelect);

  // create and open new file, apply read/write permissions, and close it.
  flightData = SD.open("data.txt", FILE_WRITE);
  flightData.close();
  
  //GPS initialization
  Serial3.begin(9600,SERIAL_8N1); //uart3 pins 7 and 8 to GPS
  delay(5000); //wait 5 seconds for warm up to finish
  memset(gpsData, '\0', sizeof(char)*128);
}

//REAL SHIT
void loop() {
  char gps_output;
  
  //While data is available
  if(Serial3.available()){
    gps_output = Serial3.read();
    if(gps_output == '\n' && count != 0){ //we legitimately reach the end of a line
      gpsData[count] = gps_output;
      write_to_SD(gpsData); //at end of line, write out that line to sd card
      write_to_SD(packetID); //writes packet identifier out after every packet sent (TRANSMITTER)
      if(callsignCount >= 100){ //writes Ross' callsign out every 100 packets
        write_to_SD(callSign);
        callsignCount = 0;
      }
      callsignCount++;
      count = 0;
      memset(gpsData, '\0', sizeof(char)*128);
    }
    else if(gps_output == '\n' && count == 0){ //first character of line is newline, trash
      memset(gpsData, '\0', sizeof(char)*128);
    }
    else if(gps_output != '\n' && count != 0){ //ordinary reading in middle of line
      if(gps_output == '$'){
        while(gps_output != '\n'){
          gps_output = Serial3.read();
        }
        count = 0;
        memset(gpsData, '\0', sizeof(char)*128);
      }
      else {
        if(count < 128){
          gpsData[count] = gps_output;
          count++;
        }
        else {
          count = 0;
          memset(gpsData, '\0', sizeof(char)*128);
        }
      }
    }
    else { //gps_output != '\n' && count == 0 //ordinary reading at start of line
      if(gps_output == '$'){
        gpsData[count] = gps_output;
        count++;
      }
      else {
        while(gps_output != '\n'){
          gps_output = Serial3.read();
        }
      }
    }
  }
}

//WRITE TO SD CARD
void write_to_SD(char data[]){
  flightData = SD.open("data.txt", FILE_WRITE);
  if(flightData){
    flightData.write(data);
  }
  flightData.close();
}
