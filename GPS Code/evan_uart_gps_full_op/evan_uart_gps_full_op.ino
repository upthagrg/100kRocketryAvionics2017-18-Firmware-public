#include <SoftwareSerial.h>
#include <SD.h>
#include <SPI.h>

File flightData;

//Information on protocols used:
//http://www.gpsinformation.org/dale/nmea.htm
const int gpsSentence = 128;
char gpsData[gpsSentence];      //Holds content of GPS data received
char callSign[18] = "$callsign:KI7NXS";
char packetID[18] = "$packetID:BIGGIE";
//char packetID[18] = "$packetID:SMALLS";
const int chipSelect = BUILTIN_SDCARD;
int count = 0;
int callsignCount = 0;

void setup() {
  Serial.begin(9600); //USB serial
  /*while (!Serial) {
    ; // endless loop to wait for the damn thing to connect. REMOVE FROM FINAL IMPLEMENTATION
  }*/

  Serial.println("Initializing SD card...");
  SD.begin(chipSelect);

  Serial.println("SD card initialization done.");
  // create and open new file, apply read/write permissions, and close it.
  Serial.println("Creating data.txt file...");
  flightData = SD.open("data.txt", FILE_WRITE);
  // Check to see if the file exists:
  if (SD.exists("data.txt")) {
    Serial.println("data.txt exists.");
  }
  else {
    Serial.println("data.txt doesn't exist.");
  }
  flightData.close();

  //TXRX Initialization
  Serial2.begin(19200);
  Serial.println("TXRX Initialized");
  
  //GPS initialization
  Serial3.begin(9600,SERIAL_8N1); //uart3 pins 7 and 8 to GPS
  Serial.println("Entering startup time...");
  delay(5000); //wait 5 seconds for warm up to finish
  memset(gpsData, '\0', sizeof(char)*128);
  Serial.println("GPS initialization complete."); //Show serial monitor that dinner's ready
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  digitalWrite(23, HIGH);
}

//REAL SHIT
void loop() {
  char gps_output;
  digitalWrite(22, HIGH);
  //While data is available
  if(Serial3.available()){
    gps_output = Serial3.read();
    if(gps_output == '\n' && count != 0){ //we legitimately reach the end of a line
      gpsData[count] = gps_output;
      Serial.println("Writing data to SD card...");
      Serial.println(count);
      write_to_SD(gpsData); //at end of line, write out that line to sd card
      send_to_txrx(gpsData);
      write_to_SD(packetID); //writes packet identifier out after every packet sent (TRANSMITTER)
      send_to_txrx(packetID);
      if(callsignCount >= 100){ //writes Ross' callsign out every 100 packets
        write_to_SD(callSign);
        send_to_txrx(callSign);
        callsignCount = 0;
      }
      callsignCount++;
      Serial.println("Done writing to SD card!");
      count = 0;
      memset(gpsData, '\0', sizeof(char)*128);
    }
    else if(gps_output == '\n' && count == 0){ //first character of line is newline, trash
      memset(gpsData, '\0', sizeof(char)*128);
      Serial.println(count);
      Serial.println("else if trash");
    }
    else if(gps_output != '\n' && count != 0){ //ordinary reading in middle of line
      if(gps_output == '$'){
        while(gps_output != '\n'){
          gps_output = Serial3.read();
        }
        count = 0;
        memset(gpsData, '\0', sizeof(char)*128);
        Serial.println(count);
        Serial.println("else if, if ordinary");
      }
      else {
        if(count < 128){
          gpsData[count] = gps_output;
          count++;
          Serial.println(count);
          Serial.println("else if, else, if ordinary");
        }
        else {
          count = 0;
          memset(gpsData, '\0', sizeof(char)*128);
          Serial.println(count);
          Serial.println("else if, else, else ordinary");
        }
      }
    }
    else { //gps_output != '\n' && count == 0 //ordinary reading at start of line
      if(gps_output == '$'){
        gpsData[count] = gps_output;
        count++;
        Serial.println(count);
        Serial.println("else, if");
      }
      else {
        while(gps_output != '\n'){
          gps_output = Serial3.read();
        }
        Serial.println(count);
        Serial.println("else, else while");
      }
    }
  }
  digitalWrite(22, LOW);
}

//WRITE TO SD CARD
void write_to_SD(char data[]){
  flightData = SD.open("data.txt", FILE_WRITE);
  if(flightData){
    //Serial.println("Writing data to SD card...");
    flightData.println(data);
    //Serial.println("Done writing to SD card!");
  }
  flightData.close();
}

//SEND TO TXRX LINK
void send_to_txrx(char dataLine[]){
  Serial2.write(dataLine);
  delay(2);
}
