const int switch1 = 8;  //comm enable switch
const int switch2 = 9;  //turn key
const int switch3 = 10; //button (active-low)
const int switch4 = 11; //arm
const int relay8  = 3;  //comm enable light
const int relay7  = 2;  //buzzer alarm
const int relay6  = 1;  //arm light
const int relay5  = 0;  //power light
const int relay4  = 12; //status light
const int relay3  = 13; //RF power amplifier

bool comm_en      = 0;  //switch1
bool turn_key     = 0;  //switch2
bool send_it      = 1;  //switch3 (high because active low)
bool arm          = 0;  //swtich4
bool comm_light   = 0;  //relay8
bool buzzer       = 0;  //relay7
bool arm_light    = 0;  //relay6
bool power_light  = 0;  //relay5
bool status_light = 0;  //relay4 blinks when pings, shows error
bool rf_pwr_amp   = 0;  //relay3

//all relays active low

bool stay = 0; //keeps the lower setup loop running if start up conditions are not met

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
      //digitalWrite(relay7, LOW); //buzzer sounds if comm en on upon startup
      status_light = 1;          //sets status light state high
      digitalWrite(relay4, LOW); //turns on status light 
      stay = 1;                  //keeps this loop running
    }
    else if(digitalRead(switch2) != 0){
      //digitalWrite(relay7, LOW); //buzzer sounds if turn key on upon start up
      status_light = 1;          //sets status light state high
      digitalWrite(relay4, LOW); //turns on status light 
      stay = 1;
    }
    else if(digitalRead(switch4) != 0){
      //digitalWrite(relay7, LOW); //buzzer sounds if arm on upon start up
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
    
  }while(stay == 1);             //while the error persists 
}  

void loop() {
  
  //reads states of switches
  if(digitalRead(switch1) == 1) comm_en = 1;
  else comm_en = 0;
  if(digitalRead(switch2) == 1) turn_key = 1;
  else turn_key = 0;
  if(digitalRead(switch4) == 1) arm = 1;
  else arm = 0;

  //call error checker bf updating leds
  check_errors(); //errors
  update_LEDS();  //lights
}

void check_errors(){
  //If Dallas tries to turn the key but he didn’t already arm the button switch
  //throw the status light and lockout until the key is turned back to the safe state
  do{
    if(turn_key == 1 && arm == 0){
      //digitalWrite(relay7, LOW); //buzzer sounds
      status_light = 1;          //sets status light state high
      digitalWrite(relay4, LOW); //turns on status light
      stay = 1;                  //locks out user, forces reset to continue
    }
  }while(stay == 1);
}

void update_LEDS(){
  if(comm_en == 1) digitalWrite(relay8, LOW); //turns on comm enable light
  else digitalWrite(relay8, HIGH);
  if(arm == 1) digitalWrite(relay6, LOW); //turns on arm light
  else digitalWrite(relay6, HIGH);
  if(status_light == 1) digitalWrite(relay4, LOW); //turns on status light
  else digitalWrite(relay4, HIGH);
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

//if((digitalRead(switch2) == 1) && digitalRead(switch3) == 0) //SENDIT
