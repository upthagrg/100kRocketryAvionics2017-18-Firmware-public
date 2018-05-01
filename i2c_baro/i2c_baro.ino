#include <Wire.h>

void setup() 
{
  Wire.begin();
  Serial.begin(9600);
}

void loop() 
{
  byte firstbyte=0;
  byte secondbyte=0;
  byte thirdbyte=0;
  int numbyte=0;
  int bytesavailable=0;
  
  // read 3 bytes, from address 0xEE
  Wire.requestFrom(0xEE, 3);
  bytesavailable = Wire.available(); //check if bytes are available
  Serial.print("Available Bytes: ");
  Serial.println(bytesavailable, DEC); //print available bytes
  
  while(Wire.read() && numbyte <= 3) {
    numbyte++;
    if(numbyte == 1){
      firstbyte = Wire.receive();
    }
    else if(numbyte == 2){
      secondbyte = Wire.receive();
    }
    else if(numbyte == 3){
      thirdbyte = Wire.receive();
    }
    else {
      firstbyte = 0; //set all bytes to zero if numbyte counter fucks up
      secondbyte = 0;
      thirdbyte = 0;
      numbyte = 4;
    }
  }
  
  numbyte = 0; //reset numbyte to zero
  Serial.print("First Byte = ");
  Serial.println(firstbyte, DEC);
  Serial.print("Second Byte = ");
  Serial.println(secondbyte, DEC);
  Serial.print("Third Byte = ");
  Serial.println(thirdbyte, DEC);
  
  delay(2000);
}
