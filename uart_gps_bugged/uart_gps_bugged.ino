#include <SoftwareSerial.h>
#include <SD.h>
#include <SPI.h>

File flightData;

//Information on protocols used:
//http://www.gpsinformation.org/dale/nmea.htm
const int gpsSentence = 128;
char gpsType[7];                //Holds type of GPS data received
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
  //delay(40000); //wait 40 seconds for warm up to finish
  //Serial3.write("$PMTK225,0");
  //Serial3.write("$PMTK353,1,0,0,0,0"); //GPS only mode - CHECK IF THIS WORKS
  Serial.println("GPS initialization complete."); //Show serial monitor that dinner's ready
}

//REAL SHIT
void loop() {
  
  int count = 0;
  char gps_output;

  //reset everything
  gps_output = '\0';
  memset(gpsType, '\0', sizeof(char)*7);
  memset(gpsData, '\0', sizeof(char)*100);
  
  //If UART3 got things to say we listen politely
  if(Serial3.available()){
    Serial.println("Made it to top");
    //Guarantee first read character is a '$' delimiter:
    while(gps_output != '$'){
      gps_output = Serial3.read();
      gpsType[0] = gps_output;
    }

    //Read in and store the type of transmission protocol
    if(gps_output == '$'){
      Serial.println("Made it to protocol");
      for(count = 1; count <= 6; count++){
        gps_output = Serial3.read();
        gpsType[count] = gps_output;
      }
      count = 0;
    }
    
    //Get GPS data based on data type
    
    //GNRMC
    if(strcmp(gpsType, "$GNRMC,") == 0){
      Serial.println("Made it to GNRMC");
      while(gps_output != '\n' && count <= 128){
        Serial.println("Made it to GNRMC while");
          gps_output = Serial3.read();
          gpsData[count] = gps_output;
          count++;
      }
      count = 0;
    }
    //GNGGA
    else if(strcmp(gpsType, "$GNGGA,") == 0){
      Serial.println("Made it to GNGGA");
      while(gps_output != '\n' && count < 128){
        Serial.println("Made it to GNGGA while");
          gps_output = Serial3.read();
          gpsData[count] = gps_output;
          count++;
      }
      count = 0;
    }
    
    //Shit data, iterate through the shit until we get to not-shit data
    else {
      Serial.println("Made it to ELSE");
      while(gps_output != '\n'){
        Serial.println("Made it to ELSE while");
          gps_output = Serial3.read();
          gpsData[count] = gps_output;
          count++;
      }
      memset(gpsType, '\0', sizeof(char)*7);
      memset(gpsData, '\0', sizeof(char)*128);
      count = 0;
    }
    
    write_to_SD(gpsType, gpsData);
   }
   Serial.println("Made it to bottom");
}

//WRITE TO SD CARD
void write_to_SD(char gpsType[], char gpsData[]){
  flightData = SD.open("data.txt", FILE_WRITE);
  if(flightData && gpsType[0] != '\0'){
    Serial.println("Writing data to SD card...");
    flightData.print(gpsType);
    flightData.println(gpsData);
    Serial.println("Done writing to SD card!");
  }
  flightData.close();
}
