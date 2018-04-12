// OSU Wireless Ignition System
 

#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3); //RX, TX

int buttonPin = 8;
boolean onOff = 0;
void setup() {
  pinMode(buttonPin, INPUT);
  mySerial.begin(1200);
}

void loop() {
 
  int buttonState = digitalRead(buttonPin);//read button state
  
  if(buttonState == 1){//if button is down
    mySerial.println(2017);//send unique code to the receiver to turn on. In this case 1111
    onOff = 1;//set boolean to 1
  }
  if(buttonState == 0 && onOff == 1){//Verifier to send off signal once
    mySerial.println(2001);//send unique code to the receiver to turn off. In this case 0000
  }
  delay(20);//delay a little for better serial communication
}

