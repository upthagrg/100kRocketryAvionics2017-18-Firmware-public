#include <SoftwareSerial.h>
#define RELAY1 7
SoftwareSerial mySerial(2, 3); // RX, TX


void setup() {
  mySerial.begin(1200);
  pinMode(RELAY1, OUTPUT);
}

void loop() {
   
  if(mySerial.available() > 1){    
    int input = mySerial.parseInt();//read serial input and convert to integer (-32,768 to 32,767)    
    if(input == 2017){//if on code is received
      digitalWrite(RELAY1, HIGH);//turn LED on
      
    }
    if(input == 2001){//if off code is received
      digitalWrite(RELAY1, LOW);//turn LED off
      delay(50);
    }
  }
  mySerial.flush();//clear the serial buffer for unwanted inputs     
  
  delay(20);//delay little for better serial communication
 
}

