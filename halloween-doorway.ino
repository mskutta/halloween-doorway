#include <ESP8266WiFi.h> // WIFI support
#include <ESP8266mDNS.h> // For network discovery
#include <WiFiUdp.h> // OSC over UDP
#include <ArduinoOTA.h> // Updates over the air

// WiFi Manager
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h> 

// OSC
#include <OSCMessage.h> // for sending OSC messages

// I2C
#include <SPI.h>
#include <Wire.h>

// Sensor
#include <VL53L0X.h>

// Display (SSD1306)
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>

const unsigned long RUN_INTERVAL = 200;

/* Variables */
bool tripped = false;
int lastRange = 0;
unsigned int count = 0;
unsigned long nextRun = 0;

/* Display */
SSD1306AsciiWire oled;

/* WIFI */
const char* ESP_NAME = "qlab-esp";
const unsigned int OSC_PORT = 53000;
char hostname[16] = {0};

/* VL53L0X */
VL53L0X sensor;

/* OSC */
WiFiUDP Udp;

String qLabHostname;
IPAddress qLabIp;
unsigned int qLabPort;
char qLabMessage[64] = {0};

void setup()
{
  Serial.begin(9600);
  Wire.begin(D2, D1);

  delay(1000);

  /* Display */
  oled.begin(&Adafruit128x64, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)
  oled.setFont(System5x7);
  oled.setScrollMode(SCROLL_MODE_AUTO);
  oled.clear();

  /* WiFi */
  sprintf(hostname, "%s-%06X", ESP_NAME, ESP.getChipId());
  WiFiManager wifiManager;
  wifiManager.setAPCallback(configModeCallback);
  if(!wifiManager.autoConnect(hostname)) {
    oled.println("WiFi Connect Failed");
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  } 

  /* UDP */
  Udp.begin(OSC_PORT);

  oled.println(hostname);
  oled.print(F("  "));
  oled.print(WiFi.localIP());
  oled.print(F(":"));
  oled.println(Udp.localPort());
  oled.print(F("  "));
  oled.println(WiFi.macAddress());
  
  /* OTA */
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    oled.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    oled.println(F("\nEnd"));
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    oled.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    oled.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) { oled.println(F("Auth Failed")); }
    else if (error == OTA_BEGIN_ERROR) { oled.println(F("Begin Failed")); }
    else if (error == OTA_CONNECT_ERROR) { oled.println(F("Connect Failed")); }
    else if (error == OTA_RECEIVE_ERROR) { oled.println(F("Receive Failed")); } 
    else if (error == OTA_END_ERROR) { oled.println(F("End Failed")); }
  });
  ArduinoOTA.begin();

  /* mDNS */
  // Initialization happens inside ArduinoOTA;
  MDNS.addService(ESP_NAME, "udp", OSC_PORT);

  // Discover qLab
  int queryCount = 0;
  while (MDNS.queryService("qlab", "udp") == 0) {
    oled.printf("find qlab: %u\r", queryCount);
    ArduinoOTA.handle();
    delay(1000);
    queryCount++;
  }
  qLabHostname = MDNS.hostname(0);
  qLabIp = MDNS.IP(0);
  qLabPort = MDNS.port(0);
  sprintf(qLabMessage, "/cue/%06X/start", ESP.getChipId());

  oled.println(qLabHostname);
  oled.print(F("  "));
  oled.print(qLabIp);
  oled.print(F(":"));
  oled.println(qLabPort);
  oled.print(F("  "));
  oled.println(qLabMessage);

  /* Sensor */
  sensor.init();
  sensor.setTimeout(500);
  sensor.startContinuous();

  // Calibrate
  oled.print(F("Calibrating...\r"));
  delay(1000);
  int calibrateCount = 0;
  while((lastRange = sensor.readRangeContinuousMillimeters()) > 1200) {
    oled.printf("Calibrate: %u\r", calibrateCount);
    delay(100);
    calibrateCount++;
  }
  oled.printf("ref range: %u\n", lastRange);

  /* LED */
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void configModeCallback (WiFiManager *myWiFiManager) {
  oled.println(F("Config Mode"));
  oled.println(WiFi.softAPIP());
  oled.println(myWiFiManager->getConfigPortalSSID());
}

void loop()
{
  ArduinoOTA.handle();
  
  if (millis() < nextRun) {
    return;
  }
  nextRun = millis() + RUN_INTERVAL;

  int range = sensor.readRangeContinuousMillimeters();
  Serial.println(range);
  if (sensor.timeoutOccurred() || range > 1200 ) { 
    return;
  }

  // Detect if tripped
  if ((range < (lastRange - 200)) && !tripped) {
    sendQLabOSCMessage(qLabMessage);
    digitalWrite(LED_BUILTIN, LOW);
    tripped = true;
    count++;
  } 
  else if (range > (lastRange + 200) && tripped) {
    digitalWrite(LED_BUILTIN, HIGH);
    tripped = false;
  }

  oled.printf("%4d%6d\r", range, count);
  oled.invertDisplay(tripped);

  lastRange = range;
}

void sendQLabOSCMessage(const char* address) {
  OSCMessage msg(address);
  Udp.beginPacket(qLabIp, qLabPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();

  // Send message three times to ensure delivery.  Need to come up with a better approach.
  Udp.beginPacket(qLabIp, qLabPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();

  Udp.beginPacket(qLabIp, qLabPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
}
