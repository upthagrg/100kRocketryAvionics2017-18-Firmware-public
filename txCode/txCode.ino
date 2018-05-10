#include <SoftwareSerial.h>
#include <SD.h>

File txData;
const int chipSelect = BUILTIN_SDCARD;
const int sentence = 128;
char tx_Data[sentence];

void setup() {
  Serial.begin(9600);
  Serial2.begin(19200);
  Serial.println("Initializing SD card...");
  txData = SD.open("data.txt");
  SD.begin(chipSelect);
  if (SD.exists("data.txt")) {
    Serial.println("data.txt exists.");
  }
  else {
    Serial.println("data.txt doesn't exist.");
  }
  txData.close();
}

void loop() {

char currentChar = '\0';

txData = SD.open("data.txt");
  if(txData){
    while(txData.available()){
      delay(1);
      currentChar = txData.read();
      Serial2.write(currentChar);
      Serial.print(currentChar);
    }
    txData.close();
    Serial.println("Reached end of file.");
  }
  else {
    Serial.println("Error opening file.");
  }
}
