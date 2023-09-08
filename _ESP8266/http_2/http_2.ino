#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>

const char* ssid = "NGOC XUAN";
const char* password = "01021967";

void setup()
{
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting...");
  }
  Serial.println("cplt");
}

void loop()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    WiFiClient client;
    HTTPClient http;
    http.begin(client, "http://tuan.pythonanywhere.com/api/data");
    int httpCode = http.GET();

    if (httpCode == 200) {
      String response = http.getString();
      DynamicJsonBuffer jsonBuffer;
      JsonObject& root = jsonBuffer.parseObject(http.getString());
      const char* message = root["message"];
      int lightValue = root["data"]["light"];
      int tempValue = root["data"]["temp"];
      Serial.print("message: ");
      Serial.println(message);
      Serial.print("Light value: ");
      Serial.println(lightValue);
      Serial.print("Temperature value: ");
      Serial.println(tempValue);
    } else {
      Serial.print("Error getting data from API. Status code: ");
      Serial.println(httpCode);
    }
    http.end(); //Close connection
  }
  delay(2000);
}
