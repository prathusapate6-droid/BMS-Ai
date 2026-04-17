// ======================================================================
// Smart EV BMS — ESP8266 Bridge prathamesh11
// D6 = RX (from STM32), D5 = TX (to STM32) @ 9600
// I2C LCD (PCF8574) on D2=SDA, D1=SCL, addr 0x27, 16 cols x 2 rows
// ======================================================================

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <LiquidCrystal_I2C.h>
#include <PubSubClient.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <WiFiClientSecureBearSSL.h>  // HiveMQ Cloud TLS ke liye
#include <Wire.h>

// ------------ USER SETTINGS ------------
const char *WIFI_SSID = "Home Network";
const char *WIFI_PASS = "asdfghjkl";

// ------------ MQTT BROKER : HiveMQ Cloud (Cloud-based, no IP issues!) ------------
#define MQTT_HOST     "e5c6d611df63436992755767b6967071.s1.eu.hivemq.cloud"
#define MQTT_PORT     8883   // TLS Encrypted port
#define MQTT_USERNAME "smartwater"
#define MQTT_PASSWORD "SmartWater2026!"

// Topics
const char *TOPIC_UP     = "bms/base";
const char *TOPIC_LAST   = "bms/last";
const char *TOPIC_CMD    = "bms/cmd";
const char *TOPIC_LWT    = "bms/online";
const char *TOPIC_INFO_IP  = "bms/bridge/ip";
const char *TOPIC_INFO_UP  = "bms/bridge/uptime";

// ------------ GLOBALS ------------
BearSSL::WiFiClientSecure net;  // TLS client
PubSubClient mqtt(net);         // MQTT over TLS
ESP8266WebServer http(80);

// D6=GPIO12 (RX), D5=GPIO14 (TX)
SoftwareSerial uart(D6, D5);  // RX, TX to STM32 @ 9600

// LCD: addr 0x27 (change to 0x3F if needed), 16x2
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Fire Spray Servo on D4 (GPIO2)
#define PIN_SERVO_FIRE D4
Servo fireSprayServo;
const float T_FIRE = 60.0f;  // Fire trigger temperature

String lastJson;
uint32_t lastJsonMs = 0;
uint32_t lastInfoMs = 0;

// streaming JSON frame state
bool inFrame = false;
int braceDepth = 0;
String frame;

struct BmsData {
  float v_pack = 0, v_solar = 0, i_pack = 0, t_amb = 0;
  float c1 = 0, c2 = 0, c3 = 0;  // NEW: Individual cell voltages
  int soc = 0, hold_s = 0;
  int fan = 0, ac = 0, solar = 0, cut = 0;
  int ov = 0, uv = 0, ot = 0, oc = 0, cell = 0;  // NEW: cell fault flag
  String pred = "SAFE", mode = "AC", cause = "";
  int lcd_page = 0;
} cur;

String chipId() {
  return String(ESP.getChipId(), HEX);
}
String hostName() {
  return String("bms-bridge-") + chipId();
}

// Fire Spray Servo Control
// on=true: Servo 90° to press spray nob
// on=false: Servo 0° rest position
void setFireSpray(bool on) {
  if (on) {
    fireSprayServo.write(90);  // Press spray
  } else {
    fireSprayServo.write(0);  // Rest
  }
}

// ------------ WIFI / MQTT ------------
void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED)
    return;

    // Optional: Enable static IP (comment out for DHCP auto-config)
    // #define USE_STATIC_IP

#ifdef USE_STATIC_IP
  // Static IP Configuration (only if needed)
  IPAddress staticIP(192, 168, 31, 37);  // Fixed IP for ESP8266
  IPAddress gateway(192, 168, 31, 1);    // Router IP
  IPAddress subnet(255, 255, 255, 0);    // Subnet mask
  IPAddress dns(192, 168, 31, 1);        // DNS (usually router)

  if (!WiFi.config(staticIP, gateway, subnet, dns)) {
    Serial.println("[WiFi] Static IP config failed!");
  }
  Serial.println("[WiFi] Using static IP mode");
#else
  Serial.println("[WiFi] Using DHCP mode (auto IP)");
#endif

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  WiFi.hostname(hostName());
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("[WiFi] Connecting");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(350);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(" OK  IP=");
    Serial.println(WiFi.localIP());
    Serial.print("[WiFi] Gateway=");
    Serial.println(WiFi.gatewayIP());
    if (MDNS.begin(hostName().c_str()))
      MDNS.addService("http", "tcp", 80);
  } else {
    Serial.println(" FAIL");
  }
}

void onMqttMsg(char *topic, byte *payload, unsigned int len) {
  String cmd;
  cmd.reserve(len);
  for (unsigned i = 0; i < len; i++)
    cmd += (char)payload[i];
  cmd.trim();
  if (cmd.length()) {
    uart.print("CMD:");
    uart.print(cmd);
    uart.print("\n");
    Serial.print("[MQTT->STM32] ");
    Serial.println(cmd);
  }
}

bool tryMQTTConnectOnce() {
  String cid = String("esp8266-") + chipId();
  // HiveMQ Cloud: username + password required
  bool ok = mqtt.connect(cid.c_str(), MQTT_USERNAME, MQTT_PASSWORD, TOPIC_LWT, 0, true, "offline");
  if (ok) {
    mqtt.subscribe(TOPIC_CMD);
    mqtt.publish(TOPIC_LWT, "online", true);
    mqtt.publish(TOPIC_INFO_IP, WiFi.localIP().toString().c_str(), true);
    if (lastJson.length())
      mqtt.publish(TOPIC_LAST, lastJson.c_str(), true);
  }
  return ok;
}

// Smart MQTT Broker Auto-Discovery 🌐
// Tests if an IP is running an MQTT Broker on port 1883
bool isBrokerListening(IPAddress ip) {
  WiFiClient testClient;
  testClient.setTimeout(100);  // 100ms timeout for operations (makes scanning 5x faster)
  if (testClient.connect(ip, MQTT_PORT)) {
    testClient.stop();
    return true;
  }
  return false;
}

String discoverMQTTBroker() {
  // 1. Try Configured IP First (Fastest if network hasn't changed)
  IPAddress confIP;
  if (confIP.fromString(MQTT_HOST)) {
    Serial.print("[Scanner] Checking saved IP: ");
    Serial.println(MQTT_HOST);
    if (isBrokerListening(confIP)) {
      Serial.println("[Scanner] ✅ Broker reachable at saved IP.");
      return String(MQTT_HOST);
    }
  }

  // 2. Not found or changed network. Start Subnet Scan
  IPAddress gateway = WiFi.gatewayIP();
  Serial.print("[Scanner] Broker missing. Scanning Subnet: ");
  Serial.print(gateway[0]);
  Serial.print(".");
  Serial.print(gateway[1]);
  Serial.print(".");
  Serial.print(gateway[2]);
  Serial.println(".x (1-254)...");

  // Check Gateway itself
  if (isBrokerListening(gateway)) {
    Serial.print("\n[Scanner] ✅ Found at Gateway: ");
    Serial.println(gateway);
    return gateway.toString();
  }

  // Scan full IP range (DHCP IPs can be anywhere from 2 to 254)
  IPAddress scanIP = gateway;
  for (int i = 1; i <= 254; i++) {
    scanIP[3] = i;
    if (i == gateway[3]) continue;

    Serial.print(".");                  // Progress bar
    if (i % 50 == 0) Serial.println();  // newline every 50 to avoid long serial lines

    if (isBrokerListening(scanIP)) {
      Serial.println();
      Serial.print("[Scanner] ✅ Found MQTT Broker at: ");
      Serial.println(scanIP);
      return scanIP.toString();
    }
  }

  Serial.println("\n[Scanner] ❌ Broker NOT found in full range 1-254.");
  return String(MQTT_HOST);  // Fallback
}

void ensureMQTT() {
  if (mqtt.connected())
    return;

  // HiveMQ Cloud: Direct connection, no scanning needed!
  // TLS fingerprint verification disabled for compatibility (setInsecure)
  net.setInsecure();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMsg);
  mqtt.setKeepAlive(60);
  mqtt.setSocketTimeout(15);
  mqtt.setBufferSize(1536);

  static uint32_t lastTry = 0;
  if (millis() - lastTry < 5000)  // 5s between attempts
    return;
  lastTry = millis();
  Serial.print("[MQTT] Connecting to HiveMQ Cloud... ");
  if (tryMQTTConnectOnce()) {
    Serial.println("OK ✅");
  } else {
    Serial.print("FAIL rc=");
    Serial.println(mqtt.state());
  }
}
static inline void lcdPrint16(uint8_t row, const String &s) {
  String x = s;
  if ((int)x.length() < 16)
    x += String(' ', 16 - x.length());  // pad to 16
  if ((int)x.length() > 16)
    x.remove(16);  // trim to 16
  lcd.setCursor(0, row);
  lcd.print(x);  // overwrite full row
}
void renderLCD() {
  // Clear both lines every update (JSON is slow, so this is fine)
  lcd.clear();

  char buf[17];
  bool anyAlarm = (cur.ov || cur.uv || cur.oc || cur.ot);

  if (anyAlarm) {
    // -------------------------
    // ERROR MODE DISPLAY
    // -------------------------
    // Line 0: big error heading
    lcdPrint16(0, "!!! BMS ERROR");

    // Line 1: list active alarms: OV UV OC OT
    String e = "ALM:";
    if (cur.ov)
      e += "OV ";
    if (cur.uv)
      e += "UV ";
    if (cur.oc)
      e += "OC ";
    if (cur.ot)
      e += "OT ";
    e.trim();  // remove last space
    lcdPrint16(1, e);

  } else {
    // -------------------------
    // NORMAL MODE DISPLAY
    // -------------------------

    // Line 0: pack voltage + solar voltage
    // e.g. "V:12.74Vs:18.00"
    snprintf(buf, sizeof(buf), "V:%4.2fVs:%4.2f", cur.v_pack, cur.v_solar);
    lcdPrint16(0, String(buf));

    // Status code from prediction: SAFE / WARN / DANGER
    String statusCode;
    if (cur.pred == "SAFE")
      statusCode = "SF";
    else if (cur.pred == "WARN")
      statusCode = "WR";
    else if (cur.pred == "DANGER")
      statusCode = "DG";
    else
      statusCode = cur.pred.substring(0, 2);

    // Line 1: current + temperature + status code
    // Example: "I:0.20 T:26.3 DG"
    snprintf(buf, sizeof(buf), "I:%4.2f T:%4.1f", cur.i_pack, cur.t_amb);
    String line1 = String(buf);  // usually shorter than 16 chars
    line1 += " ";
    line1 += statusCode;  // add SF / WR / DG at end

    lcdPrint16(1, line1);
  }
}

// parse JSON → cur + LCD refresh
void updateFromJson(const String &s) {
  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, s);
  if (err)
    return;

  cur.v_pack = doc["v_pack"] | cur.v_pack;
  cur.v_solar = doc["v_solar"] | cur.v_solar;
  cur.i_pack = doc["i_pack"] | cur.i_pack;
  cur.t_amb = doc["t_amb"] | cur.t_amb;

  // NEW: Parse individual cell voltages
  cur.c1 = doc["c1"] | cur.c1;
  cur.c2 = doc["c2"] | cur.c2;
  cur.c3 = doc["c3"] | cur.c3;

  cur.soc = doc["soc"] | cur.soc;
  cur.pred = (const char *)(doc["pred"] | cur.pred.c_str());
  cur.mode = (const char *)(doc["mode"] | cur.mode.c_str());
  cur.hold_s = doc["hold_s"] | cur.hold_s;

  JsonObject fl = doc["flags"];
  if (!fl.isNull()) {
    cur.fan = fl["fan"] | cur.fan;
    cur.ac = fl["ac"] | cur.ac;
    cur.solar = fl["solar"] | cur.solar;
    cur.cut = fl["cut"] | cur.cut;
  }

  JsonObject al = doc["alarms"];
  if (!al.isNull()) {
    cur.ov = al["ov"] | cur.ov;
    cur.uv = al["uv"] | cur.uv;
    cur.ot = al["ot"] | cur.ot;
    cur.oc = al["oc"] | cur.oc;
    cur.cell = al["cell"] | cur.cell;  // NEW: Parse cell fault flag
  }

  cur.cause = (const char *)(doc["cause"] | cur.cause.c_str());
  cur.lcd_page = doc["lcd_page"] | cur.lcd_page;  // from STM32 buttons

  // FIRE PROTECTION: Activate spray servo when temp >= 60°C
  if (cur.t_amb >= T_FIRE) {
    setFireSpray(true);  // Servo 90° to spray
  } else {
    setFireSpray(false);  // Servo 0° rest
  }

  renderLCD();
}

// ------------ HTTP HANDLERS ------------
void handleRoot() {
  String html;
  html.reserve(2300);
  html +=
    F("<!doctype html><html><head><meta name='viewport' "
      "content='width=device-width,initial-scale=1'>"
      "<title>BMS Bridge</title>"
      "<style>body{font-family:system-ui,Arial;padding:16px;background:#"
      "0f1218;color:#e8eaed}"
      "h2{margin:0 0 12px} .k{color:#9aa0a6} "
      "code,pre{background:#161a22;padding:8px;border-radius:8px;display:"
      "block;white-space:pre-wrap}"
      "input,button{padding:8px;border-radius:6px;border:1px solid "
      "#2a2e39;background:#161a22;color:#e8eaed}"
      "button{cursor:pointer} .ok{color:#6ee7a6} .bad{color:#f87171}</style>"
      "</head><body><h2>Smart EV BMS — ESP8266 Bridge</h2>");
  html += F("<div class='k'>Host: ");
  html += hostName();
  html += F(" — IP: ");
  html += WiFi.localIP().toString();
  html += F(" — MQTT: ");
  html += (mqtt.connected() ? "<span class='ok'>connected</span>"
                            : "<span class='bad'>disconnected</span>");
  html += F("</div><p class='k'>Last JSON @ ");
  html += String(lastJsonMs / 1000.0f, 1);
  html += F("s</p><h3>Last Reading</h3><pre id='j'>");
  html += (lastJson.length() ? lastJson : "{}");
  html += F("</pre><h3>Send Command</h3>"
            "<form onsubmit='sendCmd();return false;'><input id='t' "
            "placeholder='FAN_ON or CUT_OFF' style='width:260px'> "
            "<button type='submit'>Send</button></form>"
            "<p class='k'>API: <code>GET /status</code>, <code>POST "
            "/cmd?text=FAN_ON</code></p>"
            "<script>"
            "async function refresh(){try{let r=await fetch('/status');let "
            "j=await r.text();"
            "document.getElementById('j').textContent=j;}catch(e){}}"
            "async function sendCmd(){let v=document.getElementById('t').value;"
            "await fetch('/cmd?text='+encodeURIComponent(v),{method:'POST'});}"
            "setInterval(refresh,1000);"
            "</script></body></html>");
  http.send(200, "text/html; charset=utf-8", html);
}

void handleStatus() {
  http.send(200, "application/json; charset=utf-8",
            lastJson.length() ? lastJson : "{}");
}

void handleCmd() {
  String cmd = http.arg("text");
  cmd.trim();
  if (!cmd.length()) {
    http.send(400, "text/plain", "missing ?text=");
    return;
  }
  uart.print("CMD:");
  uart.print(cmd);
  uart.print("\n");
  Serial.print("[HTTP->STM32] ");
  Serial.println(cmd);
  http.send(200, "text/plain", "OK");
}

// ------------ SETUP / LOOP ------------
void setup() {
  Serial.begin(115200);
  delay(40);
  Serial.println("\n[Boot] Smart EV BMS — ESP8266 Bridge + 16x2 LCD");

  // Fire Spray Servo on D4
  fireSprayServo.attach(PIN_SERVO_FIRE);
  setFireSpray(false);  // Start at rest position
  Serial.println("[Servo] Fire spray ready on D4");

  // UART to STM32
  uart.begin(9600);
  while (uart.available())
    uart.read();

  // Wi-Fi / MQTT / HTTP / OTA
  ensureWiFi();
  ensureMQTT();
  http.on("/", HTTP_GET, handleRoot);
  http.on("/status", HTTP_GET, handleStatus);
  http.on("/cmd", HTTP_POST, handleCmd);
  http.on("/cmd", HTTP_GET, handleCmd);
  http.begin();
  Serial.println("[HTTP] Listening on :80");
  ArduinoOTA.setHostname(hostName().c_str());
  ArduinoOTA.begin();
  Serial.println("[OTA] Ready");

  // LCD init (I2C on D2=SDA, D1=SCL)
  Wire.begin(D2, D1);
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcdPrint16(0, "BMS LCD ONLINE");
  lcdPrint16(1, "Waiting data...");
}

void loop() {
  ensureWiFi();
  ensureMQTT();
  mqtt.loop();
  http.handleClient();
  ArduinoOTA.handle();
  MDNS.update();

  // Bridge uptime every ~10s
  if (mqtt.connected() && millis() - lastInfoMs > 10000) {
    lastInfoMs = millis();
    char buf[24];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)(millis() / 1000));
    mqtt.publish(TOPIC_INFO_UP, buf, true);
  }

  // Robust JSON framing from STM32
  while (uart.available()) {
    char c = (char)uart.read();

    if (!inFrame) {
      if (c == '{') {
        inFrame = true;
        braceDepth = 1;
        frame = "{";
      }
      continue;
    }

    if (c == '{')
      braceDepth++;
    frame += c;
    if (c == '}') {
      braceDepth--;
      if (braceDepth == 0) {
        lastJson = frame;
        lastJsonMs = millis();

        // publish
        if (mqtt.connected()) {
          mqtt.publish(TOPIC_UP, lastJson.c_str());
          mqtt.publish(TOPIC_LAST, lastJson.c_str(), true);
        }
        Serial.print("[UP] ");
        Serial.println(lastJson);

        // LCD update
        updateFromJson(lastJson);

        inFrame = false;
        frame = "";
      }
    }

    if (frame.length() > 1400) {
      inFrame = false;
      braceDepth = 0;
      frame = "";
    }
  }
}
