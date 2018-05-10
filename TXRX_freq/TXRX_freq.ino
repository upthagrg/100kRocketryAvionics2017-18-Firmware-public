void setup() {
  Serial.begin(9600);
  Serial2.begin(19200);
  pinMode(24, OUTPUT);
}

void loop() {
char feedback = '\0';

    Serial.println("Setup Beginning.");
    digitalWrite(24, LOW);
    Serial2.write(0x43);
    while(feedback != '>'){
      feedback = Serial2.read();
    }
    Serial.print(feedback);
    feedback = '\0';
    delay(50);
    Serial2.write(0x05);
    while(feedback != '>'){
      feedback = Serial2.read();
    }
    Serial.print(feedback);
    feedback = '\0';
    delay(50);
    digitalWrite(24, HIGH);
    Serial2.(0x58);
    feedback = '\0';
    delay(50);
    Serial.println("Setup Finished.");
  
}
