#include <SoftwareSerial.h>
#include <SD.h>
#include <SPI.h>

File flightData;

//Information on protocols used:
//http://www.gpsinformation.org/dale/nmea.htm
const int gpsSentence = 80;
char gpsType[6];                //Holds type of GPS data received
char gpsData[gpsSentence];      //Holds content of GPS data received

void setup() {
  Serial.begin(9600); //USB serial
  while (!Serial) {
    ; // endless loop to wait for the damn thing to connect. REMOVE FROM FINAL IMPLEMENTATION
  }


  Serial.print("Initializing SD card...");

  if (!SD.begin()) {
    Serial.println("SD card initialization failed.");
    //while (1); //endless loop in case this crap fails. REMOVE FROM FINAL IMPLEMENTATION
  }
  Serial.println("SD card initialization done.");

  // create and open new file, apply read/write permissions, and close it.
  Serial.println("Creating data.txt file...");
  flightData = SD.open("data.txt", FILE_WRITE);
  flightData.close();

  // Check to see if the file exists:
  if (SD.exists("data.txt")) {
    Serial.println("data.txt exists.");
  }
  else {
    Serial.println("data.txt doesn't exist.");
  }

  //GPS initialization
  Serial3.begin(9600,SERIAL_8N1); //uart3 pins 7 and 8 to GPS
  delay(5000);
  Serial3.write("$PMTK103,0"); //cold start command
  delay(40000); //wait 40 seconds for cold start to finish
  Serial3.write("$PMTK353,1,0,0,0,0"); //GPS only mode - CHECK IF THIS WORKS
  Serial.write("GPS initialization complete."); //Show serial monitor that dinner's ready
}

//REAL SHIT
void loop() {
  
  int count = 0;
  char gps_output;

  //If UART3 got things to say we listen politely
  if(Serial3.available()){
    gps_output = Serial3.read();
    //guarantee first read character is a '$' delimiter:
    while(gps_output != '$'){
      gps_output = Serial3.read();
    }
    count = 1; //Indicate that the '$' got found. WE GOT CASH.
    
    //Read in and store the type of transmission protocol
    if(gps_output == '$'){
      for(count = 0; count <= 5; count++){
        gps_output = Serial3.read();
        gpsType[count] = gps_output;
      }
    }
      count = 0;

    //Get GPS data based on data type

    //GPMRC
    if(strcmp(gpsType, "GPRMC,")){
      while(gps_output != '\n'){
          gps_output = Serial3.read();
          gpsData[count] = gps_output;
          count++;
      }
    }
    //GPGGA
    else if(strcmp(gpsType, "GPGGA,")){
      while(gps_output != '\n'){
          gps_output = Serial3.read();
          gpsData[count] = gps_output;
          count++;
      }
    }
    //These are other formats we won't likely be using
    //I HAVE NOT WRITTEN DECODING CODE FOR THESE FORMATS,
    //ONLY GPMRC AND GPGGA!!!
    //GPGSA
    /*else if(strcmp(gpsType, "GPGSA,")){
      while(gps_output != '\n'){
          gps_output = Serial3.read();
          gpsData[count] = gps_output;
          count++;
      }
    }
    //GPGSV
    else if(strcmp(gpsType, "GPGSV,")){
      while(gps_output != '\n'){
          gps_output = Serial3.read();
          gpsData[count] = gps_output;
          count++;
      }
    }
    //GPBOD
    else if(strcmp(gpsType, "GPBOD,")){
      while(gps_output != '\n'){
          gps_output = Serial3.read();
          gpsData[count] = gps_output;
          count++;
      }
    }
    //GPRMB
    else if(strcmp(gpsType, "GPRMB,")){
      while(gps_output != '\n'){
          gps_output = Serial3.read();
          gpsData[count] = gps_output;
          count++;
      }
    }
    //GPGLL
    else if(strcmp(gpsType, "GPGLL,")){
      while(gps_output != '\n'){
          gps_output = Serial3.read();
          gpsData[count] = gps_output;
          count++;
      }
    }*/
    //Shit data, iterate through the shit until we get to not-shit data
    else {
      while(gps_output != '\n'){
          gps_output = Serial3.read();
      }
    }

      write_to_SD(gpsType, gpsData);
      count = 0; //set count back to 0
   }  
}

//WRITE TO SD CARD
void write_to_SD(char gpsType[], char gpsData[]){
  if(flightData){
    Serial.write("Writing data to SD card...");
    flightData = SD.open("data.txt", FILE_WRITE);
    flightData.println(gpsType);
    flightData.println(gpsData);
    flightData.println('\n');
    flightData.close();
    Serial.write("Done writing to SD card!");
  }
}
