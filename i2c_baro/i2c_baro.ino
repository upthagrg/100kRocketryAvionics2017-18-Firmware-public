#include <Wire.h>
//#include <MS5xxx.h>

//MS5xxx sensor(&Wire);

void setup() 
{
  Wire.begin();
  Serial.begin(9600);
  /*if(sensor.connect()>0) {
    Serial.println("Error connecting...");
    delay(500);
    setup();
  }*/
}

void loop() 
{
  byte firstbyte=0;
  byte secondbyte=0;
  byte thirdbyte=0;
  int numbyte=1;
  int bytesavailable=0;
  
  // read 3 bytes, from address 0xEE
  Wire.requestFrom(0xEE, 3);
  bytesavailable = Wire.available(); //check if bytes are available
  Serial.print("Available Bytes: ");
  Serial.println(bytesavailable, DEC); //print available bytes
  
  while(Wire.read() && numbyte <= 3) {
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
    numbyte++;
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
/*
void loop() {
  sensor.ReadProm();
  sensor.Readout();
  Serial.print("Temperature [0.01 C]: ");
  Serial.println(sensor.GetTemp());
  Serial.print("Pressure [Pa]: ");
  Serial.println(sensor.GetPres());
  test_crc();
  Serial.println("---");
  delay(500);
}

void test_crc() {
  sensor.ReadProm();
  sensor.Readout(); 
  Serial.print("CRC=0x");
  Serial.print(sensor.Calc_CRC4(), HEX);
    Serial.print(" (should be 0x");
  Serial.print(sensor.Read_CRC4(), HEX);
  Serial.print(")\n");
  Serial.print("Test Code CRC=0x");
  Serial.print(sensor.CRCcodeTest(), HEX);
  Serial.println(" (should be 0xB)");
}*/
