
/// wifi access point mode
// tao ra wifi cho dt truy cap 
#include <ESP8266WiFi.h>

 char *ssid = "tuan";
 char *pass = "12345678";
 IPAddress ip(192,168,1,1);
 IPAddress gateway(192,168,1,1);
 IPAddress subnet (255,255,255,0);

 void setup ()
 {
  WiFi.disconnect();
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(ip,gateway, subnet);
  WiFi.softAP(ssid, pass);
  Serial.begin(115200);
  Serial.println("esp8266 operating in the access poin mode!");
 }

  void loop()
  {
    
  }
