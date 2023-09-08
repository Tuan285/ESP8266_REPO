#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

char *ssid = "NGOC XUAN";
char *pass = "01021967";
const char MainPage[] PROGMEM = R"=====
(
  <!DOCTYPE html>
<html>
    <head>
        <title>Web Server by Tuan</title>
        <style>
            body {text-align: center;}
            h1 {color: rgb(46, 187, 88);}
            a{text-emphasis: none; color: cadetblue;}
            .bt_on{height: 50px; width: 100px; margin: 10px; background-color: cyan;}
            .bt_off{height: 50px; width: 100px; margin: 10px; background-color: rgb(194, 169, 28);}

        </style>
        <meta  chaeset = "UTF-8">
    </head>
    <body>
        <h1>ESP8266 Web Server</h1>
        <div>trang thai chan 1</div>
        <div>
            <button class="bt_on"><a href="/onD1">ON</a></button>
            <button class="bt_off"><a href="/offD1">OFF</a></button>
        </div>
        <div>trang thai chan 2</div>
        <div>
            <button class="bt_on"><a href="/onD2"> ON </a></button>
            <button class="bt_off"><a href="/offD2"> OFF</a></button>
        </div>
    </body>
</html>
)====="

void setup() {
  pinMode (D4, OUTPUT);
  digitalWrite(D4,HIGH);
  Serial.begin(19200);
  WiFi.begin(ssid, pass);
  Serial.println("connecting ..");
  while (WiFi.status() != WL_CONNECTED )
  {
    delay(100);
    Serial.print ("...");
  }
  Serial.print("connected.");
  Serial.println(WiFi.localIP());
  
}

void loop() {
  digitalWrite(D4,HIGH);
  delay(100);
  digitalWrite(D4,HIGH);
  delay(100);
}
