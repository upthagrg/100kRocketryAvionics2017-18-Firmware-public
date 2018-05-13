/*  HC12 Send/Receive
    
    Connect HC12 "RXD" pin to Arduino Digital Pin 4
    Connect HC12 "TXD" pin to Arduino Digital Pin 5
    Connect HC12 "Set" pin to Arduino Digital Pin 6
   
    Power HC12 with a supply of at least 100 mA with 
    a 22 uF - 1000 uF reservoir capacitor.
    Upload code to two Arduinos connected to two computers.   
 */

#include <SoftwareSerial.h>
#define Relay 12                            //Relay pin

const byte HC12RxdPin = 4;                  // Recieve Pin on HC12
const byte HC12TxdPin = 5;                  // Transmit Pin on HC12

SoftwareSerial HC12(HC12TxdPin,HC12RxdPin); // Create Software Serial Port

void setup() {
  Serial.begin(9600);                       // Open serial port to computer
  HC12.begin(9600);                         // Open serial port to HC12
  pinMode(Relay, OUTPUT);
}

void loop() {
   
  if(HC12.available()){    
    int input = HC12.parseInt();  //read serial input and convert to integer (-32,768 to 32,767)    
    if(input == 15018){                //if on code is received
      digitalWrite(Relay, HIGH);     //Fire Rocket
      delay(1000);                    //Shorts E-Match for 1 second
      digitalWrite(Relay, LOW);      //Turns relay off
      input = 0;                      //Resets input variable   
    }
  }
  HC12.flush();                   //clear the serial buffer for unwanted inputs     
  
  delay(20);                          //small delay to improve serial communication
 
}

/* //code from block check offs
  if(HC12.available()){                      // If Arduino's HC12 rx buffer has data
    digitalWrite(12, HIGH);
    Serial.write(HC12.read());              // Send the data to the computer
    }
  if(Serial.available()){                   // If Arduino's computer rx buffer has data
    HC12.write(Serial.read());              // Send that data to serial
  }
//
*/

