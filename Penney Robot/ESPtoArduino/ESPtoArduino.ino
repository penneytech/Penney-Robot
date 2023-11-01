#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <WebSocketsClient.h>

// SETUP INFORMATION FOR STUDENTS
char serverAddress[] = "192.168.0.132";
const char* clientID = "PenneyBot3000";


// VARIABLES AND OBJECTS FOR PROGRAM
SoftwareSerial mySerial(D5, D6);
String receivedData = "";
unsigned long lastMessageTime = 0;
unsigned long lastBlinkTime = 0;
unsigned long lastWifiDisconnectPrint = 0;
unsigned long lastWsDisconnectPrint = 0;
const long disconnectInterval = 5000;
char ssid[] = "PenneyTest";
char pass[] = "password";
int port = 8080;
unsigned long previousMillis = 0;
const long interval = 50;
int count = 0;
WiFiClient wifi;
WebSocketsClient client;

void setup() {

  // Begin Serial Port
  Serial.begin(9600);
  // Begin Software Serial Port
  mySerial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("STARTING");

  // Attempt To Connect to WIFI
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  // After WIFI connection, report my IP address
  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

// 
  client.begin(serverAddress, port);
  client.onEvent(webSocketEvent);
  client.setAuthorization("Authorization Header");
  delay(1000);
  String messageToSend = "ID:" + String(clientID);
}

void loop() {
  unsigned long currentMillis = millis();

  if (WiFi.status() != WL_CONNECTED && currentMillis - lastWifiDisconnectPrint >= disconnectInterval) {
    lastWifiDisconnectPrint = currentMillis;
    Serial.println("WiFi disconnected");
  }

  client.loop();

  if (!client.isConnected() && currentMillis - lastWsDisconnectPrint >= disconnectInterval) {
    lastWsDisconnectPrint = currentMillis;
    Serial.println("WebSocket disconnected");
    client.begin(serverAddress, port);
    client.setAuthorization("Authorization Header");
    delay(1000);
    String messageToSend = "ID:" + String(clientID);
    client.sendTXT(messageToSend);
  }

  while (mySerial.available()) {
    char c = mySerial.read();
    if (c == '\n') {
      Serial.println("Received: " + receivedData);
      client.sendTXT(receivedData);
      receivedData = "";
    } else {
      receivedData += c;
    }
  }

  if (currentMillis - lastBlinkTime >= 1000) {
    digitalWrite(LED_BUILTIN, LOW);
  }
  if (currentMillis - lastBlinkTime >= 1200) {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  if (currentMillis - lastBlinkTime >= 1300) {
    digitalWrite(LED_BUILTIN, LOW);
    lastBlinkTime = currentMillis;
  }
}

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_TEXT:
      Serial.print("Received a message: ");
      Serial.println((char*)payload);
      break;
    case WStype_CONNECTED:
      {
        String messageToSend = "ID:" + String(clientID);
        client.sendTXT(messageToSend);
        Serial.println("WebSocket connected");
      }
      break;
    case WStype_DISCONNECTED:
      break;
  }
}
