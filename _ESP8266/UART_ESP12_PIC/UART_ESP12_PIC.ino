#include <Arduino.h>
#include <SoftwareSerial.h>

int tx = 14;
int rx = 12;
int i =0;
String dt = "";
String data, datas;
SoftwareSerial pic(rx, tx);
void setup() {
  Serial.begin(9600); 
  pic.begin(9600);
  Serial.println("Starting ...");
}
void loop() {
  if(pic.available()){
    char c = pic.read();
    dt += c;
    if ( c == '\n'){
      data = dt;
      if(datas != data){
        datas = data;
        Serial.print(datas);
      }
      dt =  "";
    }
  }
}
