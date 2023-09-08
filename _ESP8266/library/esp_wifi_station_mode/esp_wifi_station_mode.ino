#include <ArduinoJson.h>
#include "FirebaseESP8266.h"
#include<ESP8266WiFi.h>

char *ssid = "NGOC XUAN";
char *pass = "01021967";
FirebaseData firebaseData;
WiFiClient client;
String path = "/";
FirebaseJson json;
unsigned long t1 = 0;
//byte x = 0;
void setup() {
  pinMode (D4, OUTPUT);
  digitalWrite(D4,HIGH);
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
}
void loop() {
//  Firebase.setInt(firebaseData,path + "/VT1",1);
    Firebase.getString(firebaseData,path + "/test/datane");
    String x = firebaseData.stringData();
    Serial.println(x);
    if (x == "\"0\"")
    {
      digitalWrite(D4,LOW);
    }
    else 
    {
      digitalWrite(D4,HIGH);
    }
    delay(500);

}
