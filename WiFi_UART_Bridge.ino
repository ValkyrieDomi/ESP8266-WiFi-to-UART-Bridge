#include <ESP8266WiFi.h>

// ========== Configuration ==========
#define UART_BAUD 115200        // For better debug performance
#define packTimeout 5           // ms
#define bufferSize 1024         // Reduced for memory safety
#define WIFI_TIMEOUT_MS 15000

#define MODE_STA                // Use STA mode
// #define MODE_AP

#define PROTOCOL_UDP            // Use UDP
// #define PROTOCOL_TCP

#define WIFI_LED_PIN 2          // GPIO2 (onboard LED on NodeMCU, active LOW)

// ---------- WiFi Credentials ----------
#ifdef MODE_AP
const char *ssid = "mywifi";
const char *pw   = "qwerty123";
IPAddress ip(192, 168, 0, 1);
IPAddress netmask(255, 255, 255, 0);
const int port = 9876;
#endif

#ifdef MODE_STA
const char *ssid = "ExpressLRS TX Backpack B5596C";
const char *pw   = "expresslrs";
const int port = 14550;
#endif

// ========== Protocol Setup ==========
#ifdef PROTOCOL_TCP
#include <WiFiClient.h>
WiFiServer server(port);
WiFiClient client;
#endif

#ifdef PROTOCOL_UDP
#include <WiFiUdp.h>
WiFiUDP udp;
IPAddress remoteIp;
#endif

uint8_t buf1[bufferSize];  // Incoming from WiFi
uint16_t i1 = 0;

uint8_t buf2[bufferSize];  // Outgoing to WiFi
uint16_t i2 = 0;

bool wifiReady = false;
bool uartDisabled = false;

// For reconnection attempt
unsigned long lastWiFiCheck = 0;
const unsigned long wifiCheckInterval = 5000; // every 5 seconds

// ========== Setup ==========
void setup() {
  delay(500);
  pinMode(WIFI_LED_PIN, OUTPUT);
  digitalWrite(WIFI_LED_PIN, HIGH); // LED off (active LOW)

  Serial.begin(UART_BAUD);

#ifdef MODE_AP
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(ip, ip, netmask);
  WiFi.softAP(ssid, pw);
  Serial.println("Starting AP...");

  unsigned long start = millis();
  while (WiFi.softAPgetStationNum() == 0 && millis() - start < WIFI_TIMEOUT_MS) {
    delay(100);
  }

  if (WiFi.softAPgetStationNum() > 0) {
    wifiReady = true;
    Serial.println("Client connected to AP");

  #ifdef PROTOCOL_TCP
    server.begin();
    Serial.println("TCP server started (AP)");
  #endif

  #ifdef PROTOCOL_UDP
    udp.begin(port);
    Serial.println("UDP server started (AP)");
  #endif

  } else {
    Serial.println("No client connected - disabling UART");
    Serial.flush();
    Serial.end();
    uartDisabled = true;
  }
#endif // MODE_AP

#ifdef MODE_STA
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pw);
  Serial.println("Connecting to WiFi...");

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_TIMEOUT_MS) {
    delay(100);
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiReady = true;
    Serial.print("Connected. IP: ");
    Serial.println(WiFi.localIP());

  #ifdef PROTOCOL_TCP
    server.begin();
    Serial.println("TCP server started (STA)");
  #endif

  #ifdef PROTOCOL_UDP
    udp.begin(port);
    Serial.println("UDP server started (STA)");
  #endif

  } else {
    Serial.println("WiFi connection failed - disabling UART");
    Serial.flush();
    Serial.end();
    uartDisabled = true;
  }
#endif // MODE_STA

  if (wifiReady) {
    digitalWrite(WIFI_LED_PIN, LOW); // LED ON (connected)
  }
}

// ========== Main Loop ==========
void loop() {
  unsigned long now = millis();

  // WiFi disconnected after initial success
  if (WiFi.status() != WL_CONNECTED && wifiReady) {
    wifiReady = false;
    digitalWrite(WIFI_LED_PIN, HIGH); // LED OFF

    Serial.println("WiFi lost.");
    Serial.flush();
    Serial.end();
    uartDisabled = true;
  }

  // Attempt reconnection if previously failed
  if (!wifiReady && uartDisabled && (now - lastWiFiCheck > wifiCheckInterval)) {
    lastWiFiCheck = now;

    WiFi.begin(ssid, pw);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_TIMEOUT_MS) {
      delay(100);
    }

    if (WiFi.status() == WL_CONNECTED) {
      wifiReady = true;
      uartDisabled = false;

      Serial.begin(UART_BAUD);                    // ✅ Only now!
      digitalWrite(WIFI_LED_PIN, LOW);           // Turn LED ON
      Serial.print("Reconnected. IP: ");
      Serial.println(WiFi.localIP());

    #ifdef PROTOCOL_TCP
      server.begin();
      Serial.println("TCP server restarted");
    #endif

    #ifdef PROTOCOL_UDP
      udp.begin(port);
      Serial.println("UDP server restarted");
    #endif

    } else {
      // ❌ Don't start Serial = UART stays off
      // Just silently fail
    }
  }

  // Skip protocol handling if WiFi isn't ready
  if (!wifiReady) return;

#ifdef PROTOCOL_TCP
  if (!client.connected()) {
    client = server.available();
    return;
  }

  if (client.available()) {
    while (client.available()) {
      buf1[i1] = (uint8_t)client.read();
      if (i1 < bufferSize - 1) i1++;
    }
    Serial.write(buf1, i1);
    i1 = 0;
  }

  if (Serial.available()) {
    while (1) {
      if (Serial.available()) {
        buf2[i2] = (uint8_t)Serial.read();
        if (i2 < bufferSize - 1) i2++;
      } else {
        delay(packTimeout);
        if (!Serial.available()) break;
      }
    }
    client.write(buf2, i2);
    i2 = 0;
  }
#endif

#ifdef PROTOCOL_UDP
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    remoteIp = udp.remoteIP();
    udp.read(buf1, bufferSize);
    Serial.write(buf1, packetSize);
  }

  if (Serial.available() && remoteIp != IPAddress(0, 0, 0, 0)) {
    while (1) {
      if (Serial.available()) {
        buf2[i2] = (uint8_t)Serial.read();
        if (i2 < bufferSize - 1) i2++;
      } else {
        delay(packTimeout);
        if (!Serial.available()) break;
      }
    }
    udp.beginPacket(remoteIp, port);
    udp.write(buf2, i2);
    udp.endPacket();
    i2 = 0;
  }
#endif
}
