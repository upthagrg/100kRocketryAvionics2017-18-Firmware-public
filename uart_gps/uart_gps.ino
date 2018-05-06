#include <SoftwareSerial.h>
//#include <Wire.h>
#include <SD.h>
#include <SPI.h>

//pin 7 = RX3
//pin 8 = TX3
//GPS expects
//Force on pin 5 - pulled low by default
//RTS pin 6 - ready to send
//CTS pin 20 - clear to send
//GPIO 12 pin 2 - sleep function 

//SoftwareSerial gpsSerial(7,8);  //FOR WHAT
const int gpsSentenceS = 80;
char sentence[gpsSentenceS];
char GPSTEST[128];
char CLEAR[128]; //clear buffer

void setup() {

  Serial.begin(9600); //USB serial
  Serial3.begin(9600,SERIAL_8N1); //uart3 pins 7 and 8 to GPS
  
//sprintf(GPSTEST, "$PMTK225,0");//,0,0,1000,0,1000"); //normal operation command
  delay(5000);
  Serial3.write("$PMTK225,0"); //normal operation command
  Serial.write("wait over"); //normal operation command to serial monitor to view
}

void loop(){
  if(Serial3.available()){ //UART3
    int gps = Serial3.read();
    Serial.println(gps);
  }
}
/*
void loop() {
  digitalWrite(13, HIGH);
  static int count =0;
  char index1array[7];
  int commaCounter = 0;
  char comma;
  int comma3Idx = 0;
  int comma6Idx = 0;
  char latDirection;
  char longDirection;
  bool clearToSend = false;
  char data[30];
  int countPrev=0;
  int numWrites = 100;
  
  countPrev=0;

  if(Serial3.available()) //UART3
   {
      char gpsS = Serial3.read();
      if(gpsS != '\n' && count<gpsSentenceS)
      {
        sentence[count] = gpsS;
        count++;    
      }
      else
      {
        sentence[count] = '\0';
        countPrev=count;
        count = 0;
        Serial.println(sentence);
        if(dataFile)
        {
          if(writeCounter == 10)
          {
            Serial.println("writing to file");
            dataFile.println(sentence);
            writeIdx++;
            writeCounter = 0;
            
            Serial.println(writeIdx);
            dataFile.close();
            dataFile = SD.open("datalog.txt", FILE_WRITE);
          }
          else
          {
            writeCounter++;
            Serial.println("writing to file");
            dataFile.println(sentence);
          }
        }
        for(int i=0; i<5; i++)
        {
          index1array[i] = sentence[i];
        }
        if( (index1array[0] == '$') && (index1array[3] == 'R'))
        {

          for(int k=0; k<countPrev; k++)
          {
            if(sentence[k] == ',')
            {
              commaCounter++;
            }
            if(commaCounter == 2)
            {
              comma3Idx = k;
              
            }
            if(commaCounter == 6)
            {
              comma6Idx = k; 
              
             }

          }
          Serial.println(comma3Idx);
          Serial.println(comma6Idx);

            for(int q= (comma3Idx+1); q < (comma6Idx+2); q++)
            {
              data[q-(comma3Idx+1)] = sentence[q];
            }
            //data[comma6Idx+2] = '\0';

          Serial.println(data);
          Serial1.println(data);
          }
      }
   }
  
}

void displayGPS()
{
  char field[20];
  Serial.print(" ");
  getField(field,0);//checks to see what data line is being received
  if(strcmp(field, "$GPRMC") == 0)
  {
    Serial.print("Lat: ");
    getField(field, 3); //latitude is the third part of the data sentence, so it is stored in field
    Serial.print(field);
    getField(field, 4); //N/S
    Serial.print(field);
    
    Serial.print(" Long: ");
    getField(field, 5);
    Serial.print(field);
    getField(field, 6);
    Serial.print(field);
  }
  if(strcmp(field, "$GPGGA") == 0)
  {
    Serial.print(" Alt: ");
    getField(field, 9);
    Serial.println(field);
  } 
}

void getField(char* buffer, int index)
{
  int sentencePos = 0;
  int fieldPos = 0;
  int commaCount = 0;
  while (sentencePos < gpsSentenceS)
  {
    if (sentence[sentencePos] == ',')
    {
      commaCount ++;
      sentencePos ++;
    }
    if (commaCount == index)
    {
      buffer[fieldPos] = sentence[sentencePos];
      fieldPos ++;
    }
    sentencePos ++;
  }
  buffer[fieldPos] = '\0';
} 

*/
