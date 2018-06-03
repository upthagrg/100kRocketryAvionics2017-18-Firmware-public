#include <SoftwareSerial.h>
#include <SimpleTimer.h>

const int launch_code = 15018; //launch code
const byte HC12RxdPin = 4;     // Recieve Pin on HC12
const byte HC12TxdPin = 5;     // Transmit Pin on HC12
const byte HC12setPin = 6;     // Set Pin on HC12
const int switch1 = 8;         //comm enable switch
const int switch2 = 9;         //turn key
const int switch3 = 10;        //button (active-low)
const int switch4 = 11;        //arm
const int relay8  = 3;         //comm enable light
const int relay7  = 2;         //buzzer alarm
const int relay6  = 1;         //arm light
const int relay5  = 0;         //power light
const int relay4  = 12;        //status light
const int relay3  = 13;        //RF power amplifier

bool comm_en      = 0;         //switch1
bool turn_key     = 0;         //switch2
bool send_it      = 1;         //switch3 (high because active low)
bool arm          = 0;         //swtich4
bool comm_light   = 0;         //relay8
bool buzzer       = 0;         //relay7
bool arm_light    = 0;         //relay6
bool power_light  = 0;         //relay5
bool status_light = 0;         //relay4 blinks when pings, shows error
bool rf_pwr_amp   = 0;         //relay3
//all relays active low

SoftwareSerial HC12(HC12TxdPin,HC12RxdPin); // Create Software Serial Port

int  arm_count    = 0;  //arm ping counter
bool beep_once    = 0;  //only beeps on initial key turn    
bool stay         = 0;  //keeps the lower setup loop running if start up conditions are not met
SimpleTimer timer;      //creates timer object

void setup() {
  
  // put your setup code here, to run once:
  pinMode(relay3, OUTPUT);
  pinMode(relay4, OUTPUT);
  pinMode(relay5, OUTPUT);
  pinMode(relay6, OUTPUT);
  pinMode(relay7, OUTPUT);
  pinMode(relay8, OUTPUT);
  pinMode(switch1, INPUT);
  pinMode(switch2, INPUT);
  pinMode(switch3, INPUT);
  pinMode(switch4, INPUT);
  pinMode(HC12setPin, OUTPUT);

  digitalWrite(relay3, HIGH);
  digitalWrite(relay4, HIGH);
  digitalWrite(relay5, LOW);  //power indicator
  digitalWrite(relay6, HIGH);
  digitalWrite(relay7, HIGH);
  digitalWrite(relay8, HIGH);

  //checks for initial extreme conditions upon start up
  //TX needs to have all initial states from above met upon power up
  do{
    
    if(digitalRead(switch1) != 0){
      digitalWrite(relay7, LOW); //buzzer sounds if comm en on upon startup
      status_light = 1;          //sets status light state high
      digitalWrite(relay4, LOW); //turns on status light 
      stay = 1;                  //keeps this loop running
    }
    else if(digitalRead(switch2) != 0){
      digitalWrite(relay7, LOW); //buzzer sounds if turn key on upon start up
      status_light = 1;          //sets status light state high
      digitalWrite(relay4, LOW); //turns on status light 
      stay = 1;
    }
    else if(digitalRead(switch4) != 0){
      digitalWrite(relay7, LOW); //buzzer sounds if arm on upon start up
      status_light = 1;          //sets status light state high
      digitalWrite(relay4, LOW); //turns on status light 
      stay = 1;
    }
    else{
      digitalWrite(relay7, HIGH);//stops buzzer
      status_light = 0;          //sets status light state high
      digitalWrite(relay4, HIGH);//turns on status light 
      stay = 0;                  //keeps this loop running
      
      //resets all global variables
      comm_en      = 0;  //switch1
      turn_key     = 0;  //switch2
      send_it      = 1;  //switch3 (high because active low)
      arm          = 0;  //swtich4
      comm_light   = 0;  //relay8
      buzzer       = 0;  //relay7
      arm_light    = 0;  //relay6
      power_light  = 1;  //relay5
      status_light = 0;  //relay4 blinks when pings, shows error
      rf_pwr_amp   = 0;  //relay3
      
      //resets all relays
      digitalWrite(relay3, HIGH);
      digitalWrite(relay4, HIGH);
      digitalWrite(relay5, LOW);  //power indicator
      digitalWrite(relay6, HIGH);
      digitalWrite(relay7, HIGH);
      digitalWrite(relay8, HIGH); 
    }

    HC12.begin(9600);     // Open serial port to HC12 after initial conditions checked
    
  }while(stay == 1);      //while the error persists 

  timer.setInterval(15000, ping); //calls ping every 15 seconds

  //TRANSCEIVER CONFIGURATION---------------------------------------------------------------------
  //Example: Send “AT+B38400” to module, and the module returns “OK+B19200”.
  //3. AT+Cxxxx – Change wireless communication channel, from 001 to 100.
  //Default: Channel 001, with working frequency of 433.4MHz. Each next channel is 400KHz higher.
  //Example: If we want to set the module to channel 006, we need to send “AT+C006” command to the 
  //module, and the module will return “OK+C006”. The new working frequency will be 435.4MHz.

  //Channel 7 = 436.2MHz, use channel 2 433.8 if worried about range
  //digitalWrite(relay3, LOW);           //turns on power amp
  //digitalWrite(relay4, LOW);           //flashes status light on
  //delay(500);
  
  //HC12.print(7777);                    // sends change channel command to RX box
  //delay(20);
  //HC12.print("AT+C007");               // sends channel code to RX box
  //delay(200);
  
  digitalWrite(HC12setPin, LOW);       // Set HC-12 into AT Command mode
  delay(200);                          // Wait for the HC-12 to enter AT Command mode
  HC12.print("AT+C007");               // Send AT Command to HC-12
  delay(200);
  digitalWrite(HC12setPin, HIGH);      // Exit AT Command mode
  
  //digitalWrite(relay3, HIGH);          //turns off power amp
  //digitalWrite(relay4, HIGH);          //flashes status light on
  //---------------------------------------------------------------------------------------------
  
}  

void loop() {
  
  //reads states of switches------------------------
  if(digitalRead(switch1) == 1){                    //reads comm enable switch
    comm_en = 1;
    digitalWrite(relay3, LOW);                      //turns on power amp
    timer.run();                                    //runs 15 second timer
  }
  else{
    comm_en = 0;
    digitalWrite(relay3, HIGH);                     //turns off power amp
  }
  
  if(digitalRead(switch2) == 1){                    //reads turn key position
    turn_key = 1;
    delay(100);                                     //turn key debounce delay
    if(beep_once == 0){                             //only beeps on initial key turn
      digitalWrite(relay7, LOW);                    //turns buzzer on
      delay(1000);                                  //sounds for half a second
      digitalWrite(relay7, HIGH);                   //turns buzzer off 
      beep_once = 1;
    }        
  }
  else{
    turn_key = 0;
    beep_once = 0;
  }
  
  if(digitalRead(switch4) == 1){                    //reads arm switch position
    arm = 1;
    delay(250);                                        
    if(arm_count < 4){
      digitalWrite(relay4, LOW);                    //flashes status light on
      HC12.println(5555);                           //sends arm code 4 times
      arm_count++;                                  //increment arm count by 1
      delay(40);                                    //small delay between transmissions
      digitalWrite(relay4, HIGH);                   //flashes the status light off
    }
  }
  else{
    arm = 0;                                        //resets arm state
    arm_count = 0;                                  //resets arm count
  }

  check_errors(); //errors
  update_LEDS();  //lights
  check_launch(); //senditt
   
}

void check_launch(){
  
  if(comm_en == 1 && turn_key == 1){ //comm_en and key turned
    check_errors();                  //checks errors again for good measure
    if(digitalRead(switch3) == 0){   //when red button pressed
      launch();                      //calls launch function
    }
  }
  
}

void ping(){
  
  //comm link pings if comm_en switch is turned and arm switch is off
  if(comm_en == 1 && arm == 0){
    digitalWrite(relay4, LOW);       //flashes status light
    //HC12.println("USER HAM RADIO CALLSIGN");          //sends callsign every 15 seconds as well
    delay(100);
    HC12.println(4444);              //sends comm link code
    delay(150);                      //makes light visable
    digitalWrite(relay4, HIGH);
  }
       
}

void check_errors(){
  
  do{
    //If Dallas tries to turn the key but he didn’t already arm the button switch
    //throw the status light and lockout until the key is turned back to the safe state
    if(turn_key == 1 && arm == 0){
      digitalWrite(relay7, LOW);    //buzzer sounds
      status_light = 1;             //sets status light state high
      digitalWrite(relay4, LOW);    //turns on status light
      stay = 1;                     //locks out user, forces reset to continue
    }
    //If someone tries to arm without comm enabled, throw an error
    if(comm_en == 0 && arm == 1){
      digitalWrite(relay7, LOW);    //buzzer sounds
      status_light = 1;             //sets status light state high
      digitalWrite(relay4, LOW);    //turns on status light
      stay = 1;                     //locks out user, forces reset to continue
    }
    
  }while(stay == 1);
  
}

void update_LEDS(){
  
  if(comm_en == 1) digitalWrite(relay8, LOW);      //turns on comm enable light
  else digitalWrite(relay8, HIGH);
  if(arm == 1) digitalWrite(relay6, LOW);          //turns on arm light
  else digitalWrite(relay6, HIGH);
  if(status_light == 1) digitalWrite(relay4, LOW); //turns on status light
  else digitalWrite(relay4, HIGH);
  
}

void launch(){
  
  //digitalWrite(relay7, LOW);  //turns buzzer on 
  digitalWrite(relay4, LOW);  //flashes status light
  
  HC12.println(launch_code);  //sends launch code
  
  delay(150);
  digitalWrite(relay4, HIGH); //turns status light off
  //digitalWrite(relay7, HIGH); //turns buzzer off
    
}

void full_reset(){
  
  //resets all global variables
  comm_en      = 0;  //switch1
  turn_key     = 0;  //switch2
  send_it      = 1;  //switch3 (high because active low)
  arm          = 0;  //swtich4
  comm_light   = 0;  //relay8
  buzzer       = 0;  //relay7
  arm_light    = 0;  //relay6
  power_light  = 1;  //relay5
  status_light = 0;  //relay4 blinks when pings, shows error
  rf_pwr_amp   = 0;  //relay3
      
  //resets all relays
  digitalWrite(relay3, HIGH);
  digitalWrite(relay4, HIGH);
  digitalWrite(relay5, LOW);  //power indicator
  digitalWrite(relay6, HIGH);
  digitalWrite(relay7, HIGH);
  digitalWrite(relay8, HIGH); 

}



