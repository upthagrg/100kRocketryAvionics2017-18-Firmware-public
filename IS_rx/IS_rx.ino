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
#define Buzzer 11
#define light_comm 8                        //comm link (yellow)
#define light_power 9                       //power light (red)
#define light_safe 10                       //safe light (green)


const byte HC12RxdPin = 4;                  // Recieve Pin on HC12
const byte HC12TxdPin = 5;                  // Transmit Pin on HC12
const byte HC12setPin = 6;                  // Set Pin on HC12
const int  switch1 = 3;                     // Switch1 pin
bool       sw1 = 0;                         // Switch1 state
String     readBuffer = "";                 // HC12 buffer
byte       incomingByte;                    // Incoming byte to HC12

SoftwareSerial HC12(HC12TxdPin,HC12RxdPin); // Create Software Serial Port

void setup() {
  Serial.begin(9600);                       // Open serial port to computer
  HC12.begin(9600);                         // Open serial port to HC12
  pinMode(switch1, OUTPUT);
  pinMode(Relay, OUTPUT);
  pinMode(light_comm, OUTPUT);
  pinMode(light_power, OUTPUT);
  pinMode(light_safe, OUTPUT);
  pinMode(Buzzer, OUTPUT);
  pinMode(HC12setPin, OUTPUT);

  digitalWrite(light_power, HIGH);     //when powered on
  digitalWrite(light_safe, HIGH);      //when powered on
  
  //TRANSCEIVER CONFIGURATION---------------------------------------------------------------------
  //Example: Send “AT+B38400” to module, and the module returns “OK+B19200”.
  //3. AT+Cxxxx – Change wireless communication channel, from 001 to 100.
  //Default: Channel 001, with working frequency of 433.4MHz. Each next channel is 400KHz higher.
  //Example: If we want to set the module to channel 006, we need to send “AT+C006” command to the 
  //module, and the module will return “OK+C006”. The new working frequency will be 435.4MHz.

  //Channel 7 = 436.2MHz, use channel 2 433.8 if worried about range
  
  digitalWrite(HC12setPin, LOW);       // Set HC-12 into AT Command mode
  delay(200);                          // Wait for the HC-12 to enter AT Command mode
  HC12.print("AT+C007");               // Send AT Command to HC-12
  delay(200);
  digitalWrite(HC12setPin, HIGH);      // Exit AT Command mode
  
  //---------------------------------------------------------------------------------------------
  
  startup();                           //startup tone
    
}

void loop() {

  //checks safe light
  if(digitalRead(switch1) == HIGH){
    digitalWrite(light_safe, LOW);
    sw1 = 1;
  }
  else{
    digitalWrite(light_safe, HIGH);
    sw1 = 0;
  }

  //radio link handler-----------------  
  if(HC12.available()){    
    int input = HC12.parseInt();       //read serial input and convert to integer (-32,768 to 32,767)    
 
    //LAUNCH CONDITION
    if(sw1 == 1){                      //checks that switch turned
      if(input == 15018){              //if on code is received
        digitalWrite(Relay, HIGH);     //Fire Rocket
        delay(500);                    //Shorts E-Match for 1 second
        digitalWrite(Relay, LOW);      //Turns relay off
        input = 0;                     //Resets input variable   
      }
    }

    //COMM LINK STATUS
    if(input == 4444){
      comm_beep();                     //comm beep pattern
      input = 0;                       //resets input value
    }

    if(input == 5555){                 //arm beep upon arm code receive
      tone(Buzzer, 4000);
    }
    else noTone;                       //turns buzzer off then gets a command other than armed              

    /*if(input == 7777){                 //change frequency channel
      delay(20);
      incomingByte = HC12.read();      //store each incoming byte 
      readBuffer += char(incomingByte);//add each byte to buffer
      digitalWrite(HC12setPin, LOW);   //Set HC-12 into AT Command mode
      delay(100);                      //Wait for the HC-12 to enter AT Command mode
      HC12.print(readBuffer);          //sends AT command to HC12
      delay(200);
      digitalWrite(HC12setPin, HIGH);  //Exit AT Command mode
      input = 0;                       //resets input value
    }*/
    
    HC12.flush();                      //clear the serial buffer for unwanted inputs     
    delay(20);                         //small delay to improve serial communication
  }
}

void comm_beep(){

  digitalWrite(light_comm, HIGH);  //flashes comm yellow light
  tone(Buzzer, 4800);
  delay(50);
  noTone(Buzzer);
  delay(50);
  tone(Buzzer, 4800);
  delay(50);
  noTone(Buzzer);
  delay(50);
  tone(Buzzer, 4800);
  delay(50);
  noTone(Buzzer);
  delay(50);
  tone(Buzzer, 4800);
  delay(50);
  noTone(Buzzer);
  delay(50);
  tone(Buzzer, 4800);
  delay(50);
  noTone(Buzzer);
  digitalWrite(light_comm, LOW);
  delay(500);

  digitalWrite(light_comm, HIGH);  //flashes comm yellow light
  tone(Buzzer, 4800);
  delay(50);
  noTone(Buzzer);
  delay(50);
  tone(Buzzer, 4800);
  delay(50);
  noTone(Buzzer);
  delay(50);
  tone(Buzzer, 4800);
  delay(50);
  noTone(Buzzer);
  delay(50);
  tone(Buzzer, 4800);
  delay(50);
  noTone(Buzzer);
  delay(50);
  tone(Buzzer, 4800);
  delay(50);
  noTone(Buzzer);
  digitalWrite(light_comm, LOW);
  delay(150);
}

void startup(){

  tone(Buzzer, 4000);
  delay(150);
  noTone(Buzzer);
  delay(150);
  tone(Buzzer, 4000);
  delay(150);
  noTone(Buzzer);
  delay(150);
  tone(Buzzer, 4000);
  delay(150);
  noTone(Buzzer);
  delay(150);

  tone(Buzzer, 4800);
  delay(50);
  noTone(Buzzer);
  delay(50);
  tone(Buzzer, 4800);
  delay(50);
  noTone(Buzzer);
  delay(50);
  tone(Buzzer, 4800);
  delay(50);
  noTone(Buzzer);
  delay(50);
  tone(Buzzer, 4800);
  delay(50);
  noTone(Buzzer);
  delay(50);
  tone(Buzzer, 4800);
  delay(50);
  noTone(Buzzer);
  delay(150);



  tone(Buzzer, 4000);
  delay(250);
  noTone(Buzzer);
  delay(250);
  tone(Buzzer, 4000);
  delay(250);
  noTone(Buzzer);
  delay(250);
  tone(Buzzer, 4000);
  delay(250);
  noTone(Buzzer);
  delay(250);
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

