#include <FS.h>
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#define WIFI_MANAGER_USE_ASYNC_WEB_SERVER
#include "WiFiManager.h"          //https://github.com/tzapu/WiFiManager         //https://github.com/tzapu/WiFiManager
#include <wifiUDP.h>
#include <mDNSResolver.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>

#include <AlexaIntegration.h>

#include <WeatherDisplay.h>


Ticker ticker;

AsyncWebServer server(80);
DNSServer dns;

//#define AP_NAME "CONFIG_DEVICE"

#define RELAY_1 D5
#define RELAY_2 D6
#define RELAY_3 D7
#define RELAY_4 D8

#define LED_PIN D0
#define RESET_BUTTON D3

#define MQTT_VERBOSE false
//#if ASYNC_TCP_SSL_ENABLED
//#define MQTT_SECURE true
//#define MQTT_SERVER_FINGERPRINT {0x7e, 0x36, 0x22, 0x01, 0xf9, 0x7e, 0x99, 0x2f, 0xc5, 0xdb, 0x3d, 0xbe, 0xac, 0x48, 0x67, 0x5b, 0x5d, 0x47, 0x94, 0xd2} //#define MQTT_PORT 8883 //#else //#define MQTT_PORT 1883 //#endif

StaticJsonBuffer<200> jsonBuffer;

using namespace mDNSResolver;
WiFiUDP udp;
Resolver resolver(udp);
//IPAddress mqtt_server;

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

//flag for saving data
bool shouldSaveConfig = false;

//define your default values here, if there are different values in config.json, they are overwritten.
char mqtt_server[40];
char mqtt_port[6] = "1883";
char pub_topic[40];
char sub_topic[40];
char gpio_A[40];
char gpio_B[40];
char gpio_C[40];
char gpio_D[40];



// header
wifiRelay sprinkler; /* Overloaded Class Instantiation*/





void nstrcpy(char * a, const char * b) {
  // Why nstrcpy, you ask? Because strcpy is dangerous
  // Only execute if b is not null and length of b <= a
  if (b != NULL) {
    //if (strlen(b) <= strlen(a)) {
      strcpy(a, b);
    //}
  }

}


void tick() {
  //toggle state
  int state = digitalRead(LED_PIN);  // get the current state of GPIO1 pin
  digitalWrite(LED_PIN, !state);     // set pin to the opposite state
}




void setGPIO(const char * device_name, bool state) {

  int pin = -1;
  pin = sprinkler.getPin(device_name);

  // Turning off everything garantees that only desired relay is on
  sprinkler.resetPins();

  //Switching action on detection of device name
  if ( pin >= 0 ) {
    if (state)
    {
      digitalWrite(pin, HIGH);
    }
    else
    {
      digitalWrite(pin, LOW);
    }
    // Publish state change
    // TODO: Use JSON {device: , pin: , value:}



    int state;
    Serial.println("Device ");
    Serial.print(device_name);
    Serial.print(" state: ");

    if (digitalRead(pin) == HIGH) {  // Confirm state by reading pin

      Serial.println("ON");
      state = 1;

    } else {

      Serial.println("OFF");
      state = 0;

    }

    char payload[50];
    snprintf(payload,50,"{\"name\":\"%s\", \"value\":%d}",device_name, state);
    mqttClient.publish(pub_topic, 2, true, payload);
    Serial.print(" Payload: ");
    Serial.println(payload);

    ui.transitionToFrame(3); // switch to frame to indicate which zone is on

  } else {
    Serial.print("[Warning] No GPIO pin associated with ");
    Serial.println(device_name);
  }
}


//callback
void callback(uint8_t device_id, const char * device_name, bool state) {

  setGPIO(device_name, state);

}

void drawSoftAP(OLEDDisplay *display, String APIP, String APSSID) {
  display->clear();
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_10);
  display->drawString(64, 10, APIP);
  display->drawString(64, 30, APSSID);
  display->display();
}

void configModeCallback(WiFiManager *myWiFiManager) {

   Serial.println("Entered config mode");
   Serial.println("Soft AP's IP Address: ");
   Serial.println(WiFi.softAPIP());
   Serial.println("WiFi Manager: Please connect to AP:");
   Serial.println(myWiFiManager->getConfigPortalSSID());
   Serial.println("To setup WiFi Configuration");

   drawSoftAP(&display,WiFi.softAPIP().toString(), myWiFiManager->getConfigPortalSSID());

 }

void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}


void saveConfig() {
  Serial.println("Saving config...");
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["mqtt_server"] = mqtt_server;
  json["mqtt_port"] = mqtt_port;
  json["pub_topic"] = pub_topic; // PUB topic
  json["sub_topic"] = sub_topic; // SUB topic

  json["gpio_A"] = gpio_A;
  json["gpio_B"] = gpio_B;
  json["gpio_C"] = gpio_C;
  json["gpio_D"] = gpio_D;

  if (SPIFFS.begin()) {
    Serial.println("mounted FS");
    if (!SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("[Warning] Config file not found... attempting to create new file");
    }
    Serial.println("opening config file for writing...");
    File configFile = SPIFFS.open("/config.json", "w");
    if (configFile) {

      json.prettyPrintTo(Serial);
      json.printTo(configFile);
      configFile.close();
      Serial.println(" ==> CONFIG SAVED");
      Serial.println();

    } else {
      Serial.println("[Warning] Failed to open config file for writing");
    }
    //} else {
    //  Serial.println("[Warning] Config file not found");
    //}
  } else {
    Serial.println("[Warning] Failed to mount FS");
  }

}

void resetConfig() {
  nstrcpy(mqtt_server, "");
  nstrcpy(mqtt_port, "");
  nstrcpy(pub_topic, ""); // PUB topic
  nstrcpy(sub_topic, ""); // SUB topic

  nstrcpy(gpio_A, "");
  nstrcpy(gpio_B, "");
  nstrcpy(gpio_C, "");
  nstrcpy(gpio_D, "");

  saveConfig();
}

void initConfig() {
  nstrcpy(mqtt_server, "rpi3.local");
  nstrcpy(mqtt_port, "1883");
  nstrcpy(pub_topic, "home/sprinkler/out"); // PUB topic
  nstrcpy(sub_topic, "home/sprinkler/in"); // SUB topic

  nstrcpy(gpio_A, "Front Yard");
  nstrcpy(gpio_B, "Back Yard");
  nstrcpy(gpio_C, "Side Yard");
  nstrcpy(gpio_D, "Flower Bed");

  saveConfig();
}

void readConfig() {
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file...");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        configFile.close();
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.prettyPrintTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          nstrcpy(mqtt_server, json["mqtt_server"]);
          nstrcpy(mqtt_port, json["mqtt_port"]);
          nstrcpy(pub_topic, json["pub_topic"]); // PUB topic
          nstrcpy(sub_topic, json["sub_topic"]); // SUB topic

          nstrcpy(gpio_A, json["gpio_A"]);
          nstrcpy(gpio_B, json["gpio_B"]);
          nstrcpy(gpio_C, json["gpio_C"]);
          nstrcpy(gpio_D, json["gpio_D"]);

        } else {
          Serial.println("[Warning] Failed to load json config");
        }
      } else {
        Serial.println("[Warning] Failed to read config file (but was found)");
      }
    } else {
      Serial.println("[Warning] Config file not found... attempting to create");
      initConfig();
    }
  } else {
    Serial.println("[Warning] Failed to mount FS");
  }
  //end read
}

void resolveDNS() {
  // Check for *.local
  if (strstr(mqtt_server,".local") == NULL) {
    return;
  }
  drawProgress(&display, 25, "Resolving DNS...");
  // Otherwise continue

  IPAddress mqtt_server_ip;

  Serial.print("Resolving IP for ");
  //Serial.print(NAME_TO_RESOLVE);
  Serial.print(mqtt_server);
  Serial.print("...");

  resolver.setLocalIP(WiFi.localIP());
  mqtt_server_ip = resolver.search(mqtt_server);
  //mqtt_server_ip = resolver.search(NAME_TO_RESOLVE);
  //Serial.println(mqtt_server.toString());
  // if did not resolve to valid IP (ie 0.0.0.0)
  int count = 0;
  while(mqtt_server_ip[0]==0)
  {
    Serial.print(".");
    mqtt_server_ip = resolver.search(mqtt_server);
    //mqtt_server_ip = resolver.search(NAME_TO_RESOLVE);
    delay(100);
    if (count++ > 20)
    {
      Serial.println(" Timeout. Restarting");
      ESP.restart();
    }
  }
  nstrcpy(mqtt_server, mqtt_server_ip.toString().c_str());
  Serial.println(mqtt_server);
  Serial.println(" ==> RESOLVED" );
  Serial.println();
  drawProgress(&display, 30, "DNS Resolved");
}

void mqtt_setup() {
  resolveDNS();
  delay(100);

  drawProgress(&display, 40, "Connecting to MQTT...");
  //client.setServer(mqtt_server, 1883);
  //client.setCallback(mqtt_callback);
  Serial.print("Connecting to MQTT server at ");
  Serial.print(mqtt_server);
  Serial.print(" on port ");
  Serial.println(mqtt_port);
  mqttClient.setServer(mqtt_server, atoi(mqtt_port));

  mqttClient.connect();
  drawProgress(&display, 45, "Connected to MQTT");
}

void wifi_setup() {
  drawProgress(&display, 10, "Connecting to WiFi...");
  //read configuration from FS json
  readConfig();

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "1883", mqtt_port, 6);
  WiFiManagerParameter custom_pub_topic("publish", "pub/topic", pub_topic, 40); // PUB topic
  WiFiManagerParameter custom_sub_topic("subscribe", "sub/topic", sub_topic, 40); // SUB topic

  WiFiManagerParameter custom_gpio_A("gpio A", "name (gpio A)", gpio_A, 40);
  WiFiManagerParameter custom_gpio_B("gpio B", "name (gpio B)", gpio_B, 40);
  WiFiManagerParameter custom_gpio_C("gpio C", "name (gpio C)", gpio_C, 40);
  WiFiManagerParameter custom_gpio_D("gpio D", "name (gpio D)", gpio_D, 40);

  WiFiManagerParameter custom_mqtt_html("<div style=' margin-bottom: -20px; font-weight: bold; font-size: 15pt; margin-top: 10px; '>MQTT Setup</div>");
  WiFiManagerParameter custom_gpio_html("<div style=' margin-bottom: -20px; font-weight: bold; font-size: 15pt; margin-top: 10px; '>GPIO Setup</div>");


  WiFiManager wifiManager;//&server, &dns);

  //wifiManager.resetSettings();  // Only resets wifi network settings


  //add custom parameters here
  wifiManager.addParameter(&custom_mqtt_html);
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_pub_topic); // PUB topic
  wifiManager.addParameter(&custom_sub_topic); // SUB topic

  wifiManager.addParameter(&custom_gpio_html);
  wifiManager.addParameter(&custom_gpio_A);
  wifiManager.addParameter(&custom_gpio_B);
  wifiManager.addParameter(&custom_gpio_C);
  wifiManager.addParameter(&custom_gpio_D);


  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.autoConnect(); // No Input defaults AP Name = ESP + macAddress


  //read updated parameters
  nstrcpy(mqtt_server, custom_mqtt_server.getValue());
  nstrcpy(mqtt_port, custom_mqtt_port.getValue());
  nstrcpy(pub_topic, custom_pub_topic.getValue()); // PUB topic
  nstrcpy(sub_topic, custom_sub_topic.getValue()); // SUB topic

  nstrcpy(gpio_A, custom_gpio_A.getValue());
  nstrcpy(gpio_B, custom_gpio_B.getValue());
  nstrcpy(gpio_C, custom_gpio_C.getValue());
  nstrcpy(gpio_D, custom_gpio_D.getValue());


  //save the custom config to FS
  if (shouldSaveConfig) {

    saveConfig();

  }


  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
     Serial.print(".");
     delay(100);
  }

  // Report connection to host

  drawProgress(&display, 20, "Connected to WiFi");

  Serial.print(" ==> " );
  Serial.print(WiFi.hostname());
  Serial.println(" CONNECTED" );
  Serial.println();

}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  //Serial.println(" ==> Connected to Wi-Fi!");
  //Serial.println();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, wifi_setup);
}

void onMqttConnect(bool sessionPresent) {
  //Serial.println("Connected to MQTT.");

  Serial.println(" ==> CONNECTED to MQTT" );
  Serial.println();
  Serial.print(" Session present: ");
  if (sessionPresent == 1) {
    Serial.println("YES");
  } else {
    Serial.println("NO");
  }
  uint16_t packetIdSub = mqttClient.subscribe(sub_topic, 2);
  Serial.print(" Subscribing to ");
  Serial.print(sub_topic);
  Serial.print(" at QoS 2, packetId: ");
  Serial.println(packetIdSub);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (reason == AsyncMqttClientDisconnectReason::TLS_BAD_FINGERPRINT) {
    Serial.println("Bad server fingerprint.");
  } else {
    Serial.println();
  }

  mqttClient.disconnect(true);

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, mqtt_setup);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println(" Subscribe acknowledged.");
  Serial.print("   packetId: ");
  Serial.println(packetId);
  Serial.print("   qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println(" Unsubscribe acknowledged.");
  Serial.print("   packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println(" Publish received.");
  Serial.print("   topic: ");
  Serial.println(topic);
  #if MQTT_VERBOSE
  Serial.print("   qos: ");
  Serial.println(properties.qos);
  Serial.print("   dup: ");
  Serial.println(properties.dup);
  Serial.print("   retain: ");
  Serial.println(properties.retain);
  Serial.print("   len: ");
  Serial.println(len);
  Serial.print("   index: ");
  Serial.println(index);
  Serial.print("   total: ");
  Serial.println(total);
  #endif
  // ** Parse JSON payload **
  jsonBuffer.clear();
  JsonObject& root = jsonBuffer.parseObject(payload);
  // ** Test if parsing succeeds. **
  if (!root.success()) {
    for (uint i = 0; i < len; i++) {
      Serial.print((char)payload[i]);
    }
    Serial.println();
    //return;
  } else {
    int val = root["value"];
    int pin = root["pin"];
    char name[40];
    nstrcpy(name, root["name"]);
    Serial.print("   value: ");
    Serial.println(val);
    Serial.print("   pin: ");
    Serial.println(pin);
    Serial.print("   name: ");
    Serial.println(name);
    bool state;
    state = val > 0;


    setGPIO(name, state);


  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.println(" Publish acknowledged.");
  Serial.print("   packetId: ");
  Serial.println(packetId);
}

void drawZones(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  //char buff[30];
  //sprintf_P(buff, PSTR("%s: %s\n%s: %s\n%s: %s\n%s %s"), timeInfo->tm_hour, timeInfo->tm_min, timeInfo->tm_sec);
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_10);

  String temp = String(String(gpio_A) + ": ");
  display->drawString(x, y + 0, temp);
   temp = String(String(gpio_B) + ": ");
  display->drawString(x, y + 12, temp);
   temp = String(String(gpio_C) + ": ");
  display->drawString(x, y + 24, temp);
   temp = String(String(gpio_D) + ": ");
  display->drawString(x, y + 36, temp);

  #define DX 64

   temp = String(((digitalRead(RELAY_1) == HIGH) ? "ON" : "OFF"));
  display->drawString(x + DX, y + 0, temp);
   temp = String(((digitalRead(RELAY_2) == HIGH) ? "ON" : "OFF"));
  display->drawString(x + DX, y + 12, temp);
   temp = String(((digitalRead(RELAY_3) == HIGH) ? "ON" : "OFF"));
  display->drawString(x + DX, y + 24, temp);
   temp = String(((digitalRead(RELAY_4) == HIGH) ? "ON" : "OFF"));
  display->drawString(x + DX, y + 36, temp);
}

void display_setup() {
  // initialize display
  display.init();
  display.clear();
  display.display();

  //display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setContrast(255);

  drawProgress(&display, 5, "Starting...");
}


void weather_setup() {
  //Serial.begin(115200);
  //Serial.println();
  //Serial.println();
/*
  // initialize display
  display.init();
  display.clear();
  display.display();

  //display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setContrast(255);
*/
  configTime(TZ_SEC, DST_SEC, "pool.ntp.org");
  ui.setTargetFPS(30);
  ui.setActiveSymbol(activeSymbole);
  ui.setInactiveSymbol(inactiveSymbole);
  // You can change this to TOP, LEFT, BOTTOM, RIGHT
  ui.setIndicatorPosition(BOTTOM);
  // Defines where the first frame is located in the bar.
  ui.setIndicatorDirection(LEFT_RIGHT);
  // You can change the transition that is used
  // SLIDE_LEFT, SLIDE_RIGHT, SLIDE_TOP, SLIDE_DOWN
  ui.setFrameAnimation(SLIDE_LEFT);
  ui.setTimePerFrame(7*1000);
  ui.setTimePerTransition(500);

  frames[0] = drawDateTime;
  frames[1] = drawCurrentWeather;
  frames[2] = drawForecast;
  frames[3] = drawZones;
  numberOfFrames = 4;

  ui.setFrames(frames, numberOfFrames);
  ui.setOverlays(overlays, numberOfOverlays);
  // Inital UI takes care of initalising the display too.
  ui.init();
  Serial.println("");
  updateData(&display);

}

void weather_loop() {

  if (millis() - timeSinceLastWUpdate > (1000L*UPDATE_INTERVAL_SECS)) {
    setReadyForWeatherUpdate();
    timeSinceLastWUpdate = millis();
  }

  if (readyForWeatherUpdate && ui.getUiState()->frameState == FIXED) {
    updateData(&display);
  }

  int remainingTimeBudget = ui.update();

  if (remainingTimeBudget > 0) {
    // You can do some work here
    // Don't do stuff if you are below your
    // time budget.
    //delay(remainingTimeBudget);
  }


}


//****************************************************************************
//*                                SETUP                                     *
//****************************************************************************


void setup() {
    // put your setup code here, to run once:
  Serial.begin(115200);

  //resetConfig();
  //initConfig();

  display_setup(); // Starting loading screen

  //set flash pin to input
  pinMode(RESET_BUTTON, INPUT);
  //set led pin as output
  pinMode(LED_PIN, OUTPUT);
  // start ticker with 0.5 because we start in AP mode and try to connect
  ticker.attach(0.6, tick);

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);


  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);

  wifi_setup();  //
  delay(100);



  mqtt_setup();
  delay(100);

  //setup
  // Device Names for Simulated Wemo switches
  sprinkler.addDevice2(RELAY_1, gpio_A);
  sprinkler.addDevice2(RELAY_2, gpio_B);
  sprinkler.addDevice2(RELAY_3, gpio_C);
  sprinkler.addDevice2(RELAY_4, gpio_D);

  sprinkler.onMessage(callback);

  weather_setup();
  delay(100);

  ticker.detach();
  digitalWrite(LED_PIN, HIGH);  // Status light indicating setup complete }

}



//****************************************************************************
//*                               MAIN LOOP                                  *
//****************************************************************************
long lastPub = 0;
char payload[40];

void loop() {
  // put your main code here, to run repeatedly:

  resolver.loop();  // This is required or TCP connection will fail

  //loop
  sprinkler.handle();  // This may be required as well

  // is configuration portal requested?
  if ( digitalRead(RESET_BUTTON) == LOW ) {
     delay(1000);
     // If still pressed after 1 seconds
     if ( digitalRead(RESET_BUTTON) == LOW ) {
       WiFi.disconnect(true);
       delay(2000);
       ESP.restart();
       //AsyncWiFiManager wifiManager(&server, &dns);
       //reset settings - for testing
       //wifiManager.resetSettings();  // Only resets wifi network settings
       //delay(3000);
       //ESP.restart();
     }
     //resetConfig(); // Resets all wifi manager settings
  }

  long now = millis();
  if (now - lastPub > 10000) {
    lastPub = now;
    //nstrcpy(payload,"{\"name\":\"Front Yard\", \"value\":1}");
    //mqttClient.publish(pub_topic, 2, true, payload);
  }

  weather_loop();

}
