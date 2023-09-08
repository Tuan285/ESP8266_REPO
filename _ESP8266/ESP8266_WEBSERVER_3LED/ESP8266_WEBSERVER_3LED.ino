#include <ESP8266WiFi.h>

const char* ssid = "NGOC XUAN";
const char* password = "01021967";

int LED1 = 16; // GPIO16 (D0)
int LED2 = 14; // GPIO16 (D5)
int LED3 = 12; // GPIO16 (D6)
WiFiServer server(80);
unsigned long t_prev = 0;
int led = 0;
int point = 10;
int tt = 0;
bool tt_s = 0;
int value = 0;
void setup() {
  Serial.begin(115200);
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, LOW);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED2, LOW);
  pinMode(LED3, OUTPUT);
  digitalWrite(LED3, LOW);
  Serial.print("Connecting to the Newtork");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  server.begin();
  Serial.println("Server started");
  Serial.print("IP Address of network: "); // will IP address on Serial Monitor
  Serial.println(WiFi.localIP());
  t_prev = millis();
}

void loop() {
  if (tt == 2) {
    led = random(1, 4);
    Serial.println(led);
    tt = 3;
  }
  if (tt != 2 && tt != 3)
  {
    if (millis() - t_prev >= 500) {
      led++;
      if (led > 3) {
        led = 0;
        tt++;
      }
      t_prev = millis();
    }
  }
  switch (led) {
    case 1:
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, LOW);
      digitalWrite(LED3, LOW);
      break;
    case 2:
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, HIGH);
      digitalWrite(LED3, LOW);
      break;
    case 3:
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, LOW);
      digitalWrite(LED3, HIGH);
      break;
  }
  WiFiClient client = server.available();
  if (!client) {
    return;
  }
  //  Serial.println("Waiting for new client");
  while (!client.available())
  {
    delay(1);

  }
  String request = client.readStringUntil('\r');
  //  Serial.println(request);
  client.flush();


  if (request.indexOf("/LEDRED") != -1) {
    tt_s = 1;
    value = 1;
  }

  if (request.indexOf("/LEDGREEN") != -1) {
    tt_s = 1;
    value = 2;
  }
  if (request.indexOf("/LEDBLUE") != -1) {
    tt_s = 1;
    value = 3;
  }
  if (tt_s == 1) {
    if (led == value) {
      point++;
      Serial.println(point);
      Serial.println("True");
      tt = 0;
      tt_s = 0;
    }
    else {
      point--;
      Serial.println(point);
      Serial.println("False");
      tt = 0;
      tt_s = 0;
    }
  }
  //*------------------HTML Page Code---------------------*//

  client.println("HTTP/1.1 200 OK"); //
  client.println("Content-Type: text/html");
  client.println("");
  client.println("<!DOCTYPE HTML>");
  client.println("<html>");
  client.print(" CONTROL LED: ");
  if (value == 1) {
    client.print("RED");
    Serial.println("RED");
  }
  if (value == 2)
  {
    client.print("GREEN");
    Serial.println("GREEN");
  }
  if (value == 3)
  {
    client.print("BLUE");
    Serial.println("BLUE");
  }
  client.println("<br><br>");
  client.print(" POINT: ");
  client.print(String(point));
  client.println("<br><br>");
  client.println("<a href=\"/LEDRED\"\"><button>RED</button></a>");
  client.println("<a href=\"/LEDGREEN\"\"><button>GREEN</button></a>");
  client.println("<a href=\"/LEDBLUE\"\"><button>BLUE</button></a><br />");
  client.println("</html>");
  delay(1);
}
