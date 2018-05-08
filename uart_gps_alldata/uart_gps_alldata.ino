#include <SoftwareSerial.h>
#include <SD.h>
#include <SPI.h>

File flightData;

//Information on protocols used:
//http://www.gpsinformation.org/dale/nmea.htm
const int gpsSentence = 128;
char gpsData[gpsSentence];      //Holds content of GPS data received
const int chipSelect = BUILTIN_SDCARD;

void setup() {
  Serial.begin(9600); //USB serial
  while (!Serial) {
    ; // endless loop to wait for the damn thing to connect. REMOVE FROM FINAL IMPLEMENTATION
  }

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
  
  //GPS initialization
  Serial3.begin(9600,SERIAL_8N1); //uart3 pins 7 and 8 to GPS
  Serial.println("Entering startup time...");
  delay(5000); //wait 5 seconds for warm up to finish
  memset(gpsData, '\0', sizeof(char)*100);
  Serial.println("GPS initialization complete."); //Show serial monitor that dinner's ready
}

//REAL SHIT
void loop() {
  
  int count = 0;
  char gps_output;
  
  //While data is available
  if(Serial3.available()){
    if(gps_output == '\n'){
      write_to_SD(gpsData); //at end of line, write out that line to sd card
      Serial.println(gpsData); //debug
      count = 0;
      gps_output = Serial3.read();
    }
  
    else {
      gps_output = Serial3.read();
      gpsData[count] = gps_output;
      Serial.print(gps_output); //debug
    }
    count++; //might break here
  }
}

//WRITE TO SD CARD
void write_to_SD(char gpsData[]){
  flightData = SD.open("data.txt", FILE_WRITE);
  if(flightData && gpsData[0] != '\0'){
    Serial.println("Writing data to SD card...");
    flightData.println(gpsData);
    Serial.println("Done writing to SD card!");
  }
  flightData.close();
}
