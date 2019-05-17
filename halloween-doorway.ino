#include <ESP8266WiFi.h> // WIFI support
#include <ESP8266mDNS.h> // For network discovery
#include <WiFiUdp.h> // OSC over UDP
#include <ArduinoOTA.h> // Updates over the air

// WiFi Manager
#include <IotWebConf.h>

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

const unsigned long RUN_INTERVAL = 100;

/* Variables */
const char* SERVICE_NAME = "qlab-lidar";

bool tripped = false;
int maxRange = 0;
unsigned int count = 0;
unsigned long runTime = 0;

/* Display */
SSD1306AsciiWire oled;

/* VL53L0X */
VL53L0X sensor;

/* OSC */
WiFiUDP Udp;
const unsigned int OSC_PORT = 53000;
String qLabHostname;
IPAddress qLabIp;
unsigned int qLabPort;
char qLabMessage[64] = {0};

/* IotWebConf */
//char thingName[] = "qlab-lidar-000000";
const String espChipId = String(ESP.getChipId(), HEX);
const char * thingName = espChipId.c_str();
const char wifiInitialApPassword[] = "********"; // Initial password to connect to the Thing, when it creates an own Access Point.

#define CONFIG_PIN D8 // When CONFIG_PIN is pulled to ground on startup, the Thing will use the initial password to build an AP. (E.g. in case of lost password)
#define CONFIG_VERSION "1" // Configuration specific key. The value should be modified if config structure was changed.
#define STATUS_PIN LED_BUILTIN // Status indicator pin. First it will light up (kept LOW), on Wifi connection it will blink, when connected to the Wifi it will turn off (kept HIGH).

// Callback method declarations.
void configSaved();
boolean formValidator();
void wifiConnected();

DNSServer dnsServer;
ESP8266WebServer httpServer(80);
HTTPUpdateServer httpUpdater;

#define STRING_LEN 128
char oscCommandValue[STRING_LEN];

IotWebConf iotWebConf(thingName, &dnsServer, &httpServer, wifiInitialApPassword, CONFIG_VERSION);
IotWebConfParameter oscCommandParam = IotWebConfParameter("OSC command", "oscCommand", oscCommandValue, STRING_LEN);

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

  oled.println(thingName);

  /* IotWebConf */
  iotWebConf.setStatusPin(STATUS_PIN);
  iotWebConf.setConfigPin(CONFIG_PIN);
  iotWebConf.addParameter(&oscCommandParam);
  iotWebConf.setConfigSavedCallback(&configSaved);
  iotWebConf.setFormValidator(&formValidator);
  iotWebConf.setWifiConnectionCallback(&wifiConnected);
  iotWebConf.getApTimeoutParameter()->visible = true;
  iotWebConf.setupUpdateServer(&httpUpdater);
  
  iotWebConf.init();

  httpServer.on("/", handleRoot);
  httpServer.on("/config", []{ iotWebConf.handleConfig(); });
  httpServer.onNotFound([](){ iotWebConf.handleNotFound(); });
  
  /* UDP */
  Udp.begin(OSC_PORT);

  /* OTA */
  ArduinoOTA.setHostname(thingName);
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
  // MDNS.addService(SERVICE_NAME, "udp", OSC_PORT);

  // Discover qLab
  int queryCount = 0;
  while (MDNS.queryService("qlab", "udp") == 0) {
    oled.printf("find qlab: %u\r", queryCount);
    iotWebConf.doLoop();
    ArduinoOTA.handle();
    //delay(1000);
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
  iotWebConf.delay(1000);
  while((maxRange = sensor.readRangeContinuousMillimeters()) > 1200) {
    iotWebConf.delay(100);
  }
  oled.printf("max range: %u\n", maxRange);

  /* LED */
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop()
{
  iotWebConf.doLoop();
  ArduinoOTA.handle();
  
  if (millis() < runTime) {
    return;
  }

  int range = sensor.readRangeContinuousMillimeters();
  Serial.println(range);
  if (sensor.timeoutOccurred() || range > 1200 ) { 
    return;  
  }

  // Detect if tripped
  if (range < (maxRange - 200) && !tripped) {
    sendQLabOSCMessage(qLabMessage);
    digitalWrite(LED_BUILTIN, LOW);
    tripped = true;
    count++;
  } 
  else if (range > (maxRange - 100) && tripped) {
    digitalWrite(LED_BUILTIN, HIGH);
    tripped = false;
  }

  oled.printf("%4d%6d\r", range, count);
  oled.invertDisplay(tripped);
  
  runTime = millis() + RUN_INTERVAL;
}



void sendQLabOSCMessage(const char* address) {
  OSCMessage msg(address);
  Udp.beginPacket(qLabIp, qLabPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();

  // Send message three times to ensure delivery.  Need to come up with a better approach.
  delay(100);

  Udp.beginPacket(qLabIp, qLabPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();

  delay(100);

  Udp.beginPacket(qLabIp, qLabPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
}

void handleRoot() {
  // -- Let IotWebConf test and handle captive portal requests.
  if (iotWebConf.handleCaptivePortal())
  {
    // -- Captive portal request were already served.
    return;
  }
  String s = "<!DOCTYPE html><html lang=\"en\"><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/>";
  s += "<title>qLab LiDaR Parameters</title></head><body>Hello world!";
  s += "<ul>";
  s += "<li>OSC Command Value: ";
  s += oscCommandValue;
  s += "</li>";
  s += "</ul>";
  s += "Go to <a href='config'>configure page</a> to change values.";
  s += "</body></html>\n";

  httpServer.send(200, "text/html", s);
}

void configSaved() {
  oled.println(F("Config Updated"));
}

boolean formValidator() {
  oled.println(F("Validating Form"));
  boolean valid = true;
  return valid;
}

void wifiConnected()
{
  oled.print(F("  "));
  oled.print(WiFi.localIP());
  oled.print(F(":"));
  oled.println(Udp.localPort());
  oled.print(F("  "));
  oled.println(WiFi.macAddress());
}
