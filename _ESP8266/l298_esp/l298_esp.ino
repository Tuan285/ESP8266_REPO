#define ENA  D5         // Enable/speed motors Right        GPIO14(D5)
#define ENB  D6         // Enable/speed motors Left         GPIO12(D6)
#define IN_1  D8          // L298N in1 motors Right           GPIO15(D8)
#define IN_2  D7          // L298N in2 motors Right           GPIO13(D7)
#define IN_3  D4           // L298N in3 motors Left            GPIO2(D4)
#define IN_4  D3           // L298N in4 motors Left            GPIO0(D3)
 
#include <ArduinoJson.h>
#include "FirebaseESP8266.h"
#include<ESP8266WiFi.h>

FirebaseData firebaseData;
WiFiClient client;
String path = "/";
FirebaseJson json;
unsigned long t1 = 0;

//String command;             //String to store app command state.
int speedCar = 1023;         // 400 - 1023.
int speed_Coeff = 3;
 
//const char* ssid = "NodeMCU Car";
//ESP8266WebServer server(80);
char *ssid = "NGOC XUAN";
char *pass = "01021967";

void setup() {
 pinMode(ENA, OUTPUT);
 pinMode(ENB, OUTPUT);  
 pinMode(IN_1, OUTPUT);
 pinMode(IN_2, OUTPUT);
 pinMode(IN_3, OUTPUT);
 pinMode(IN_4, OUTPUT); 
  
// Connecting WiFi
 
 WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin (ssid, pass);
  Serial.begin(115200);
  Serial.println("Connecting...");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("...");
  }
  Firebase.begin("test1-fc9fb-default-rtdb.firebaseio.com","QP07qcznXFzfmh36bUj0tBIjFHOyeKBaA8JbzSLR");
  Firebase.reconnectWiFi (true);
  Serial.println(WiFi.localIP());
 
 // Starting WEB-server 
//     server.on ( "/", HTTP_handleRoot );
//     server.onNotFound ( HTTP_handleRoot );
//     server.begin();    
}
 
void goAhead(){ 
      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, HIGH);
      analogWrite(ENA, speedCar);
 
      digitalWrite(IN_3, LOW);
      digitalWrite(IN_4, HIGH);
      analogWrite(ENB, speedCar);
  }
 
void goBack(){ 
      digitalWrite(IN_1, HIGH);
      digitalWrite(IN_2, LOW);
      analogWrite(ENA, speedCar);
 
      digitalWrite(IN_3, HIGH);
      digitalWrite(IN_4, LOW);
      analogWrite(ENB, speedCar);
  }
 
void goRight(){ 
      digitalWrite(IN_1, HIGH);
      digitalWrite(IN_2, LOW);
      analogWrite(ENA, speedCar);
 
      digitalWrite(IN_3, LOW);
      digitalWrite(IN_4, HIGH);
      analogWrite(ENB, speedCar);
  }
 
void goLeft(){
      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, HIGH);
      analogWrite(ENA, speedCar);
 
      digitalWrite(IN_3, HIGH);
      digitalWrite(IN_4, LOW);
      analogWrite(ENB, speedCar);
  }
 
void goAheadRight(){
      
      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, HIGH);
      analogWrite(ENA, speedCar/speed_Coeff);
 
      digitalWrite(IN_3, LOW);
      digitalWrite(IN_4, HIGH);
      analogWrite(ENB, speedCar);
   }
 
void goAheadLeft(){
      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, HIGH);
      analogWrite(ENA, speedCar);
 
      digitalWrite(IN_3, LOW);
      digitalWrite(IN_4, HIGH);
      analogWrite(ENB, speedCar/speed_Coeff);
  }
 
void goBackRight(){ 
      digitalWrite(IN_1, HIGH);
      digitalWrite(IN_2, LOW);
      analogWrite(ENA, speedCar/speed_Coeff);
 
      digitalWrite(IN_3, HIGH);
      digitalWrite(IN_4, LOW);
      analogWrite(ENB, speedCar);
  }
 
void goBackLeft(){ 
      digitalWrite(IN_1, HIGH);
      digitalWrite(IN_2, LOW);
      analogWrite(ENA, speedCar);
 
      digitalWrite(IN_3, HIGH);
      digitalWrite(IN_4, LOW);
      analogWrite(ENB, speedCar/speed_Coeff);
  }
 
void stopRobot(){  
 
      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, LOW);
      analogWrite(ENA, speedCar);
 
      digitalWrite(IN_3, LOW);
      digitalWrite(IN_4, LOW);
      analogWrite(ENB, speedCar);
 }
 
void loop() {
  Firebase.getString(firebaseData,path + "/project_car/datane");
  String x = firebaseData.stringData();
  Serial.println(x);
//  
//  Firebase.getInt(firebaseData,path + "/test/datane");
//  int x = firebaseData.intData();
//  Serial.println(x);
  if (x == "\"1\""){ goAhead();}
  else if (x == "\"2\"") {goLeft();}
  else if (x == "\"3\"") {goBack();}
  else if (x == "\"4\"") {goRight();}
  else if (x == "\"5\"") {stopRobot();}
  
  
  
//    server.handleClient();
//    
//      command = server.arg("State");
//      if (command == "F") goAhead();
//      else if (command == "B") goBack();       //Các dòng này các bạn có thể điều chỉnh để phù hợp với app điều khiển của mình
//      else if (command == "L") goLeft();
//      else if (command == "R") goRight();
//      else if (command == "I") goAheadRight();
//      else if (command == "G") goAheadLeft();
//      else if (command == "J") goBackRight();
//      else if (command == "H") goBackLeft();
//      else if (command == "0") speedCar = 400;
//      else if (command == "1") speedCar = 470;
//      else if (command == "2") speedCar = 540;
//      else if (command == "3") speedCar = 610;
//      else if (command == "4") speedCar = 680;
//      else if (command == "5") speedCar = 750;
//      else if (command == "6") speedCar = 820;
//      else if (command == "7") speedCar = 890;
//      else if (command == "8") speedCar = 960;
//      else if (command == "9") speedCar = 1023;
//      else if (command == "S") stopRobot();
}
 
//void HTTP_handleRoot(void) {
// 
//if( server.hasArg("State") ){
//       Serial.println(server.arg("State"));
//  }
//  server.send ( 200, "text/html", "" );
//  delay(1);
//}
