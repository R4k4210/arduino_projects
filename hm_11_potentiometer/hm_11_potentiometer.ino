#include <SoftwareSerial.h>

SoftwareSerial BT(10,11);

const int EN = 9;
const int VCC = 8;
const int POT = A1;
const int RED = 2;
const int GREEN = 3;
int red_value = 0;
int green_value = 0;
const byte BT_CONFIG_ON = 0;
String message;

void setup(){
  pinMode(EN, OUTPUT);
  pinMode(VCC, OUTPUT);
  pinMode(POT, INPUT);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  digitalWrite(EN, HIGH);
  digitalWrite(VCC, HIGH);
  delay(500);
  Serial.begin(9600);
  BT.begin(9600);
  
  if(BT_CONFIG_ON){
    Serial.print("Waiting for AT Commands:");
  }
}

void loop(){
  if(BT_CONFIG_ON){
    configBT();
  } else {
    readBT();
    turnLightsByPote();
    
    if(message == "red"){ // Use single quote for character - strcmp(message, "red") == 0
      red_value = !red_value;    
    } 
    
    if(message == "green") { //strcmp(message, "green") == 0  
      green_value = !green_value;      
    }

    cleanBT();
    digitalWrite(RED, red_value);
    digitalWrite(GREEN, green_value);
  }
}

void cleanBT(){
  //memset(message, '\0', sizeof(message)); //This is for char array
  message = "";
}

void readBT() {
  int i = 0;
  bool end_msg = false;
  while(BT.available()){
    delay(10);
    char msg =  BT.read();
    if(msg == " "){
      Serial.print("End of message");
      end_msg = true;
      break;
    }
    //message[i] = msg;
    message += msg;
    Serial.print(">");
    Serial.println(msg);
    i++;
  }
  i = 0;
}

void configBT() {
  if(Serial.available()){
    BT.write(Serial.read());
  }

  if(BT.available()){
    Serial.write(BT.read());
  }
}

void turnLightsByPote() {
  int value = analogRead(POT);
  delay(10);
  int percent = map(value, 0, 1023, 0, 100);
  
  if(percent >= 30 && percent <= 50){ // between 30 and 50 | turned off below 30
    red_value = 0;
    green_value = 0;
  }else if(percent > 50 && percent <= 80){ // between 51 and 80
    red_value = 1;
    green_value = 0;
  } else if(percent > 80) { // greater than 80
    green_value = 1;
  }
}