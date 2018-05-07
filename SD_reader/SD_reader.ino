#include <SPI.h>
#include <SD.h>

File flightData;
char dataString[12];
char gpsType[6];       //Holds type of GPS data received

void setup() {
  Serial.begin(9600);
  memset(gpsType, '\0', sizeof(gpsType));
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.print("Initializing SD card...");
  if (!SD.begin()) {
    Serial.println("Initialization failed!");
    while (1);
  }
  Serial.println("Initialization done.");
}



void loop() {

char fileOutput; //hold the latest character output from data.txt
int count = 0; //keep track of amount of characters in dataString[]
int commaCount = 0; //keep track of comma delimiters
int i = 0;

  flightData = SD.open("data.txt"); //read permissions only
  if (flightData) {
    Serial.println("Beginning Transmission: \n");
    while (flightData.available()) {
        fileOutput = flightData.read();
        if(fileOutput == '$'){
          for(i = 0; i <= 5; i++){ //get the data type and store it in gpsType[] to compare later
            fileOutput = flightData.read();
            gpsType[i] = fileOutput;
          }
          count = 0;
        }
        if(gpsType != '\0'){ //if gpsType isn't null (meaning it's gone through the for loop above...)
          if(strcmp(gpsType, "GPRMC,")){ //Check to see if gpsType[] is GPMRC
            //print out the protocol to serial
            Serial.println("Protocol: " + gpsType[0] + gpsType[1] + gpsType[2] + gpsType[3] + gpsType[4] + gpsType[5]);
            while(fileOutput != '\n'){ //while not new line (meaning still on the same protocol...)
                fileOutput = flightData.read(); //fill dataString[] with data until...
                dataString[count] = fileOutput;
                if(fileOutput == ','){ //you run into a delimiter.
                  commaCount++; //increment the amount of commas you've run into. THIS DOES NOT INCLUDE THE VERY FIRST COMMA THAT IS AFTER THE PROTOCOL TYPE. THAT IS ALREADY ACCOUNT FOR IN gpsData[6]
                  count = 0; //reset the count to 0 after running into a comma (for a new piece of data in a particular string of data piece)
                  if(commaCount == 1){ //Display whatever data you're looking at based on how many commas you've run into so far
                    Serial.println("Fix taken at: " + dataString[0] + dataString[1] + ':' + 
                                                      dataString[2] + dataString[3] + ':' + 
                                                      dataString[4] + dataString[5] + '\n');
                  }
                  else if(commaCount == 2){
                    Serial.println("Status [Active] or [Void]: " + dataString[0] + '\n');
                  }
                  else if(commaCount == 3){
                    Serial.println("Latitude: " + dataString[0] + dataString[1] +  'd' + 
                                                  dataString[2] + dataString[3] + '.' + 
                                                  dataString[4] + dataString[5] + dataString[6] + '\n');
                  }
                  else if(commaCount == 4){
                    Serial.println("North/South: " + dataString[0] + '\n');
                  }
                  else if(commaCount == 5){
                    Serial.println("Longitude: " + dataString[0] + dataString[1] + dataString[2] + 'd' + 
                                                  dataString[3] + dataString[4] + '.' +
                                                  dataString[5] + dataString[6] + dataString[7] + '\n');
                  }
                  else if(commaCount == 6){
                    Serial.println("East/West: " + dataString[0] + '\n');
                  }
                  else if(commaCount == 7){
                    Serial.println("Ground Speed [Knots]: " + dataString[0] + dataString[1] +  
                                                  dataString[2] + '.' + dataString[3] + '\n');
                  }
                  else if(commaCount == 8){
                    Serial.println("Angle from True North [Deg]: " + dataString[0] + dataString[1] +  
                                                  dataString[2] + '.' + dataString[3] + '\n');
                  }
                  else if(commaCount == 9){
                    Serial.println("Date: " + dataString[0] + dataString[1] + '/' + 
                                                  dataString[2] + dataString[3] + '/' + 
                                                  dataString[4] + dataString[5] + '\n');
                  }
                }
                else { //if your next character in the string was not a comma or newline, just keep going and increase count
                  count++;
                }
            }
            commaCount = 0; //If you run into a newline, reset commaCount to zero
          }
          else if(strcmp(gpsType, "GPGGA,")){
            Serial.println("Protocol: " + gpsType[0] + gpsType[1] + gpsType[2] + gpsType[3] + gpsType[4] + gpsType[5]);
            while(fileOutput != '\n'){
                fileOutput = flightData.read();
                dataString[count] = fileOutput;
                if(fileOutput == ','){
                  commaCount++;
                  count = 0;
                  if(commaCount == 1){
                    Serial.println("Fix taken at: " + dataString[0] + dataString[1] + ':' + 
                                                      dataString[2] + dataString[3] + ':' + 
                                                      dataString[4] + dataString[5] + '\n');
                  }
                  else if(commaCount == 3){
                    Serial.println("Latitude: " + dataString[0] + dataString[1] + 'd' + 
                                                  dataString[2] + dataString[3] + '.' + 
                                                  dataString[4] + dataString[5] + dataString[6] + '\n');
                  }
                  else if(commaCount == 4){
                    Serial.println("North/South: " + dataString[0] + '\n');
                  }
                  else if(commaCount == 5){
                    Serial.println("Longitude: " + dataString[0] + dataString[1] + dataString[2] + 'd' + 
                                                  dataString[3] + dataString[4] + '.' +
                                                  dataString[5] + dataString[6] + dataString[7] + '\n');
                  }
                  else if(commaCount == 6){
                    Serial.println("East/West: " + dataString[0] + '\n');
                  }
                  else if(commaCount == 7){
                    Serial.println("Fix Quality: " + dataString[0] + '\n');
                  }
                  else if(commaCount == 8){
                    Serial.println("Satellites: " + dataString[0] + dataString[1] + '\n');
                  }
                  else if(commaCount == 9){
                    Serial.println("Horizontal Dilution: " + dataString[0] + dataString[1] + dataString[2] + '\n');
                  }
                  else if(commaCount == 10){
                    Serial.println("Sea Level Altitude: " + dataString[0] + dataString[1] + dataString[2]
                                                                   + dataString[3] +  dataString[4] + '\n');
                  }
                  else if(commaCount == 11){
                    Serial.println("Altitude Units: " + dataString[0] + '\n');
                  }
                }
                else {
                  count++;
                }
            }
            commaCount = 0;
          }
        }
    }
    flightData.close(); //at the end of the file, close it.
  }
  else {
    Serial.println("Error opening data.txt"); //If you have issues opening the file in the first place
  }
}
