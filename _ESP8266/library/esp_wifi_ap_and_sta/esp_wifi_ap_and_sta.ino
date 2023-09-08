#include <ESP8266WiFi.h>
// access point mode inf
char *ssid_ap = ("tuan");                     //  wifi mode access point
char *pass_ap = ("12345678");
IPAddress ip(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);
// station mode inf
char *ssid_sta = ("FPT Telecom-0876");         // wifi mode station mode
char *pass_sta = ("tuan2805");
void setup() {
  WiFi.disconnect ();
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(ip, gateway, subnet);
  WiFi.softAP(ssid_ap,pass_ap);

  WiFi.begin(ssid_sta,pass_sta);
  
  Serial.begin(9600);
  Serial.println("connecting..");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("...");
  }
  Serial.println(WiFi.localIP());
 
}

void loop() {
  // put your main code here, to run repeatedly:

}
