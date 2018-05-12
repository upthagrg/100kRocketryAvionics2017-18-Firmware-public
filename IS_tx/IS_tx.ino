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
  if(digitalRead(switch1) != 0) digitalWrite(relay7, HIGH); //buzzer sounds if comm en on upon startup 
  if(digitalRead(switch2) != 0) digitalWrite(relay7, HIGH); //buzzer sounds if turn key on upon start up
  if(digitalRead(switch4) != 0) digitalWrite(relay7, HIGH); //buzzer sounds if arm on upon start up
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
  update_LEDS(); //lights
}

void update_LEDS(){
  if(comm_en == 1) digitalWrite(relay8, LOW); //turns on comm enable light
  else digitalWrite(relay8, HIGH);
  if(arm == 1) digitalWrite(relay6, LOW); //turns on arm light
  else digitalWrite(relay6, HIGH);
  if(status_light == 1) digitalWrite(relay4, LOW); //turns on status light
  else digitalWrite(relay4, HIGH);
}

//if((digitalRead(switch2) == 1) && digitalRead(switch3) == 0) //SENDIT
