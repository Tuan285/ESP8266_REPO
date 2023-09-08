#include <ESP8266WiFi.h>

#define DS D5
#define ST_CP D6
#define SH_CP D7

#define led1 D1
#define led2 D2
#define led3 D3
#define led4 D4
//unsigned char led7doan = B11111100;
void setup() {
  pinMode (D1,OUTPUT);
  pinMode (D2,OUTPUT);
  pinMode (D3,OUTPUT);
  pinMode (D4,OUTPUT);
  pinMode (D5,OUTPUT);
  pinMode (D6,OUTPUT);
  pinMode (D7,OUTPUT);
}

void loop() {
  digitalWrite (ST_CP,LOW);
  digitalWrite(led1,HIGH);
  digitalWrite(led2,HIGH);
  digitalWrite(led3,HIGH);
  digitalWrite(led4,HIGH);
  shiftOut(DS,SH_CP,LSBFIRST,3);
  digitalWrite (ST_CP,HIGH);
  
}
