#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
ESP8266WebServer webServer(80);
char *ssid = "NGOC XUAN";
char *pass = "01021967";
void setup() {
  pinMode (D4, OUTPUT );
  digitalWrite (D4, HIGH);
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin (ssid,pass);
  Serial.begin(9600);
  Serial.println("Connecting...");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("...");
    
  }
  Serial.println(WiFi.localIP());
  
  webServer.on ("/",[](){
    webServer.send (200,"text/html","<button><a href=\"/toggle\">TOGGLE</a> </button>");
  });
  webServer.on ("/toggle",[](){
    digitalWrite (D4, !digitalRead(D4));
    webServer.send (200,"text/html","<button><a href=\"/toggle\">TOGGLE</a> </button>");
  });
  webServer.begin();
}

void loop() {
  webServer.handleClient();
}
