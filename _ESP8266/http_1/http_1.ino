#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>

const char* ssid = "NGOC XUAN";
const char* password = "01021967";
const char* serverUrl = "http://tuan.pythonanywhere.com/api/data";

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WiFi connected");
}

void loop() {
  // Đọc giá trị cảm biến và gán cho value1 và value2
  int value2 = random(0,1000);
  int value1 = random(20,70);

  // Tạo JSON payload từ value1 và value2
  String payload = "{\"temp\":" + String(value1) + ",\"light\":" + String(value2) + "}";

  // Gửi yêu cầu POST lên server
  WiFiClient client;
  HTTPClient http;
  http.begin(client, serverUrl);
  http.addHeader("Content-Type", "application/json");
  int httpResponseCode = http.POST(payload);

  // Kiểm tra kết quả trả về
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println(response);
  } else {
    Serial.printf("HTTP error: %s\n", http.errorToString(httpResponseCode).c_str());
  }

  // Chờ 5 giây trước khi gửi tiếp
  delay(5000);
}
