#include <Arduino.h>
#include <SimpleTimer.h>   //https://github.com/marcelloromani/Arduino-SimpleTimer/tree/master/SimpleTimer
#include <ESP8266WiFi.h>   //if you get an error here you need to install the ESP8266 board manager
#include <ESP8266mDNS.h>   //if you get an error here you need to install the ESP8266 board manager
#include <PubSubClient.h>  //https://github.com/knolleary/pubsubclient
#include <ArduinoOTA.h>    //https://github.com/esp8266/Arduino/tree/master/libraries/ArduinoOTA
#include <AH_EasyDriver.h> //http://www.alhin.de/arduino/downloads/AH_EasyDriver_20120512.zip
#include "SwartNinjaRSW.h"
#include "SwartNinjaSW.h"

#include "user_config.h" // Fixed user configurable options
#ifdef USE_CONFIG_OVERRIDE
#include "user_config_override.h" // Configuration overrides for my_user_config.h
#endif

#define USER_MQTT_CLIENT_NAME "CurtainsMCU" // Used to define MQTT topics, MQTT Client ID, and ArduinoOTA

///////////////////////////////////////////////////////////////////////////
//   General Declarations
///////////////////////////////////////////////////////////////////////////

char ESP_CHIP_ID[7] = {0};
char OTA_HOSTNAME[sizeof(ESP_CHIP_ID) + sizeof(OTA_HOSTNAME_TEMPLATE) - 2] = {0};

///////////////////////////////////////////////////////////////////////////
//   WiFi
///////////////////////////////////////////////////////////////////////////
// function declaration
void setupWiFi(void);
void connectWiFi(void);
void onConnected(const WiFiEventStationModeConnected &event);
void onDisconnect(const WiFiEventStationModeDisconnected &event);
void onGotIP(const WiFiEventStationModeGotIP &event);
void loopWiFiSensor(void);
int getWiFiSignalStrength(void);

// variables declaration
int previousWiFiSignalStrength = -1;
unsigned long previousMillis = 0;
int reqConnect = 0;
int isConnected = 0;
const long interval = 500;
const long reqConnectNum = 15; // number of intervals to wait for connection
WiFiEventHandler mConnectHandler;
WiFiEventHandler mDisConnectHandler;
WiFiEventHandler mGotIpHandler;

// Initialize the Ethernet mqttClient object
WiFiClient wifiClient;

/* -------------------------------------------------
 *  MQTT
 * ------------------------------------------------- */
// function declaration
void setupMQTT();
void connectToMQTT();
void checkInMQTT();
void subscribeToMQTT(char *p_topic);
void publishToMQTT(char *p_topic, char *p_payload, bool retain = true);
void handleMQTTMessage(char *topic, byte *payload, unsigned int length);

// variables declaration
bool boot = true;
char MQTT_PAYLOAD[8] = {0};
char MQTT_AVAILABILITY_TOPIC[sizeof(ESP_CHIP_ID) + sizeof(MQTT_AVAILABILITY_TOPIC_TEMPLATE) - 2] = {0};
char MQTT_WIFI_QUALITY_TOPIC[sizeof(ESP_CHIP_ID) + sizeof(MQTT_SENSOR_TOPIC_TEMPLATE) + sizeof(WIFI_QUALITY_SENSOR_NAME) - 4] = {0};
char MQTT_DOOR_SENSOR_TOPIC[sizeof(ESP_CHIP_ID) + sizeof(MQTT_SENSOR_TOPIC_TEMPLATE) + sizeof(DOOR_SENSOR_STATE_TOPIC) - 4] = {0};
char MQTT_STEPPER_TOPIC[sizeof(ESP_CHIP_ID) + sizeof(MQTT_SENSOR_TOPIC_TEMPLATE) + sizeof(STEPPER_STATE_TOPIC) - 4] = {0};
char MQTT_STEPPER_ACTION_COMMAND_TOPIC[sizeof(ESP_CHIP_ID) + sizeof(MQTT_SENSOR_COMMAND_TOPIC_TEMPLATE) + sizeof(STEPPER_CMD_ACTION) - 4] = {0};
char MQTT_STEPPER_POSITION_COMMAND_TOPIC[sizeof(ESP_CHIP_ID) + sizeof(MQTT_SENSOR_COMMAND_TOPIC_TEMPLATE) + sizeof(STEPPER_CMD_POSITION) - 4] = {0};
char MQTT_DOOR_LOCK_TOPIC[sizeof(ESP_CHIP_ID) + sizeof(MQTT_SENSOR_TOPIC_TEMPLATE) + sizeof(DOOR_LOCK_STATE_TOPIC) - 4] = {0};
char MQTT_DOOR_LOCK_COMMAND_TOPIC[sizeof(ESP_CHIP_ID) + sizeof(MQTT_SENSOR_COMMAND_TOPIC_TEMPLATE) + sizeof(DOOR_LOCK_STATE_TOPIC) - 4] = {0};

// Initialize the mqtt mqttClient object
PubSubClient mqttClient(wifiClient);

///////////////////////////////////////////////////////////////////////////
//   Stepper
///////////////////////////////////////////////////////////////////////////
// function declaration
void processStepper();

// variables declaration
int currentPosition = 0;
int newPosition = 0;
char positionPublish[50];
bool moving = false;

// Initialize the AH_EasyDriver object
AH_EasyDriver doorStepper(STEPPER_STEPS_PER_REV, STEPPER_DIR_PIN, STEPPER_STEP_PIN, STEPPER_MICROSTEP_1_PIN, STEPPER_MICROSTEP_2_PIN, STEPPER_SLEEP_PIN);

///////////////////////////////////////////////////////////////////////////
//   SwartNinjaSensors
///////////////////////////////////////////////////////////////////////////
// function declaration
void handleSwartNinjaSensorUpdate(char *value, int pin, const char *event);

// initialize the SwartNinaRSW object
SwartNinjaRSW doorSensor(DOOR_PIN, handleSwartNinjaSensorUpdate);

///////////////////////////////////////////////////////////////////////////
//   SwartNinjaRelay
///////////////////////////////////////////////////////////////////////////
// initialize the reed switch objects
SwartNinjaSW doorLock(DOOR_LOCK_PIN);

///////////////////////////////////////////////////////////////////////////
//   SimpleTimer
///////////////////////////////////////////////////////////////////////////
SimpleTimer timer;

///////////////////////////////////////////////////////////////////////////
//   MAIN SETUP AND LOOP
///////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(115200);

  // Set the chip ID
  sprintf(ESP_CHIP_ID, "%06X", ESP.getChipId());

  doorStepper.setMicrostepping(STEPPER_MICROSTEPPING); // 0 -> Full Step
  doorStepper.setSpeedRPM(STEPPER_SPEED);              // set speed in RPM, rotations per minute
#if DRIVER_INVERTED_SLEEP == 1
  doorStepper.sleepOFF();
#endif
#if DRIVER_INVERTED_SLEEP == 0
  doorStepper.sleepON();
#endif

  // WIFI
  setupWiFi();

  // MQTT
  setupMQTT();

  // Over the air
  sprintf(OTA_HOSTNAME, OTA_HOSTNAME_TEMPLATE, ESP_CHIP_ID);
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.begin();

  delay(10);
  doorSensor.setup();
  doorLock.setup();

  timer.setInterval(((1 << STEPPER_MICROSTEPPING) * 5800) / STEPPER_SPEED, processStepper);
  timer.setInterval(120000, checkInMQTT);
}

void loop()
{
  // WIFI
  connectWiFi();

  // Code will only run if connected to WiFi
  if (isConnected == 2)
  {
    // MQTT
    if (!mqttClient.connected())
    {
      connectToMQTT();
    }
    mqttClient.loop();

    // Over the air
    ArduinoOTA.handle();

    doorSensor.loop();

    // Check WiFi signal
    loopWiFiSensor();

    timer.run();
  }
}

///////////////////////////////////////////////////////////////////////////
//   Stepper
///////////////////////////////////////////////////////////////////////////
void processStepper()
{
  if (!moving && doorLock.getState()){
    #ifdef DEBUG
      Serial.print("[main.cpp]: processStepper - EXIT, door is locked.");
    #endif
    return;
  }

  if (newPosition > currentPosition)
  {
    #if DRIVER_INVERTED_SLEEP == 1
        doorStepper.sleepON();
    #endif
    #if DRIVER_INVERTED_SLEEP == 0
        doorStepper.sleepOFF();
    #endif
    doorStepper.revolve(1);
    currentPosition++;
    moving = true;
  }

  if (newPosition < currentPosition)
  {
    #if DRIVER_INVERTED_SLEEP == 1
        doorStepper.sleepON();
    #endif
    #if DRIVER_INVERTED_SLEEP == 0
        doorStepper.sleepOFF();
    #endif
    doorStepper.revolve(-1);
    currentPosition--;
    moving = true;
  }
  if (newPosition == currentPosition && moving == true)
  {
    #if DRIVER_INVERTED_SLEEP == 1
        doorStepper.sleepOFF();
    #endif
    #if DRIVER_INVERTED_SLEEP == 0
        doorStepper.sleepON();
    #endif
    String temp_str = String(currentPosition);
    temp_str.toCharArray(positionPublish, temp_str.length() + 1);
    publishToMQTT(MQTT_STEPPER_TOPIC, positionPublish);

    moving = false;
  }
}

///////////////////////////////////////////////////////////////////////////
//   SwartNinjaSensors
///////////////////////////////////////////////////////////////////////////
void handleSwartNinjaSensorUpdate(char *value, int pin, const char *event)
{
  if (event == SN_RSW_SENSOR_EVT)
  {
    // publish motion
    publishToMQTT(MQTT_DOOR_SENSOR_TOPIC, value);
  }
}

///////////////////////////////////////////////////////////////////////////
//   WiFi
///////////////////////////////////////////////////////////////////////////

/*
 * Function called to setup WiFi module
 */
void setupWiFi(void)
{
  WiFi.disconnect();
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);

  mConnectHandler = WiFi.onStationModeConnected(onConnected);
  mDisConnectHandler = WiFi.onStationModeDisconnected(onDisconnect);
  mGotIpHandler = WiFi.onStationModeGotIP(onGotIP);
}

/*
 * Function called to connect to WiFi
 */
void connectWiFi(void)
{
  if (WiFi.status() != WL_CONNECTED && reqConnect > reqConnectNum && isConnected < 2)
  {
    reqConnect = 0;
    isConnected = 0;
    WiFi.disconnect();

    Serial.println();
    Serial.print("[WIFI]: Attempting to connect to WPA SSID: ");
    Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.println("[WIFI]: Connecting...");
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    reqConnect++;
  }
}

/*
 * Function called to handle WiFi events
 */
void onConnected(const WiFiEventStationModeConnected &event)
{
  char macAdddress[20];
  sprintf(macAdddress, "%02X:%02X:%02X:%02X:%02X:%02X", event.bssid[5], event.bssid[4], event.bssid[3], event.bssid[2], event.bssid[1], event.bssid[0]);
  Serial.print(F("[WIFI]: You're connected to the AP. (MAC - "));
  Serial.print(macAdddress);
  Serial.println(")");
  isConnected = 1;
}

void onDisconnect(const WiFiEventStationModeDisconnected &event)
{
  Serial.println("[WIFI]: Disconnected");
  Serial.print("[WIFI]: Reason: ");
  Serial.println(event.reason);
  isConnected = 0;
}

void onGotIP(const WiFiEventStationModeGotIP &event)
{
  Serial.print("[WIFI]: IP Address : ");
  Serial.println(event.ip);
  Serial.print("[WIFI]: Subnet     : ");
  Serial.println(event.mask);
  Serial.print("[WIFI]: Gateway    : ");
  Serial.println(event.gw);

  isConnected = 2;
}

/*
 * Function to check WiFi signal strength
 */
void loopWiFiSensor(void)
{
  static unsigned long lastWiFiQualityMeasure = 0;
  if (lastWiFiQualityMeasure + WIFI_QUALITY_INTERVAL <= millis() || previousWiFiSignalStrength == -1)
  {
    lastWiFiQualityMeasure = millis();
    int currentWiFiSignalStrength = getWiFiSignalStrength();
    if (isnan(previousWiFiSignalStrength) || currentWiFiSignalStrength <= previousWiFiSignalStrength - WIFI_QUALITY_OFFSET_VALUE || currentWiFiSignalStrength >= previousWiFiSignalStrength + WIFI_QUALITY_OFFSET_VALUE)
    {
      previousWiFiSignalStrength = currentWiFiSignalStrength;
      dtostrf(currentWiFiSignalStrength, 2, 2, MQTT_PAYLOAD);
      publishToMQTT(MQTT_WIFI_QUALITY_TOPIC, MQTT_PAYLOAD);
    }
  }
}

/*
 * Helper function to get the current WiFi signal strength
 */
int getWiFiSignalStrength(void)
{
  if (WiFi.status() != WL_CONNECTED)
    return -1;
  int dBm = WiFi.RSSI();
  if (dBm <= -100)
    return 0;
  if (dBm >= -50)
    return 100;
  return 2 * (dBm + 100);
}

///////////////////////////////////////////////////////////////////////////
//   MQTT
///////////////////////////////////////////////////////////////////////////

/*
 * Function called to setup MQTT topics
 */
void setupMQTT()
{
  sprintf(MQTT_AVAILABILITY_TOPIC, MQTT_AVAILABILITY_TOPIC_TEMPLATE, ESP_CHIP_ID);

  Serial.print(F("[MQTT]: MQTT availability topic: "));
  Serial.println(MQTT_AVAILABILITY_TOPIC);

  sprintf(MQTT_WIFI_QUALITY_TOPIC, MQTT_SENSOR_TOPIC_TEMPLATE, ESP_CHIP_ID, WIFI_QUALITY_SENSOR_NAME);
  Serial.print(F("[MQTT]: MQTT WiFi Quality topic: "));
  Serial.println(MQTT_WIFI_QUALITY_TOPIC);

  sprintf(MQTT_DOOR_SENSOR_TOPIC, MQTT_SENSOR_TOPIC_TEMPLATE, ESP_CHIP_ID, DOOR_SENSOR_STATE_TOPIC);
  Serial.print(F("[MQTT]: MQTT Door topic: "));
  Serial.println(MQTT_DOOR_SENSOR_TOPIC);

  sprintf(MQTT_STEPPER_TOPIC, MQTT_SENSOR_TOPIC_TEMPLATE, ESP_CHIP_ID, STEPPER_STATE_TOPIC);
  Serial.print(F("[MQTT]: MQTT Stepper topic: "));
  Serial.println(MQTT_STEPPER_TOPIC);

  sprintf(MQTT_STEPPER_ACTION_COMMAND_TOPIC, MQTT_SENSOR_COMMAND_TOPIC_TEMPLATE, ESP_CHIP_ID, STEPPER_CMD_ACTION);
  Serial.print(F("[MQTT]: MQTT Stepper action topic: "));
  Serial.println(MQTT_STEPPER_ACTION_COMMAND_TOPIC);

  sprintf(MQTT_STEPPER_POSITION_COMMAND_TOPIC, MQTT_SENSOR_COMMAND_TOPIC_TEMPLATE, ESP_CHIP_ID, STEPPER_CMD_POSITION);
  Serial.print(F("[MQTT]: MQTT Stepper position topic: "));
  Serial.println(MQTT_STEPPER_POSITION_COMMAND_TOPIC);

  sprintf(MQTT_DOOR_LOCK_TOPIC, MQTT_SENSOR_TOPIC_TEMPLATE, ESP_CHIP_ID, DOOR_LOCK_STATE_TOPIC);
  Serial.print(F("[MQTT]: MQTT Door Lock topic: "));
  Serial.println(MQTT_DOOR_LOCK_TOPIC);

  sprintf(MQTT_DOOR_LOCK_COMMAND_TOPIC, MQTT_SENSOR_COMMAND_TOPIC_TEMPLATE, ESP_CHIP_ID, DOOR_LOCK_STATE_TOPIC);
  Serial.print(F("[MQTT]: MQTT Door Lock Command topic: "));
  Serial.println(MQTT_DOOR_LOCK_COMMAND_TOPIC);

  mqttClient.setServer(MQTT_SERVER, MQTT_SERVER_PORT);
  mqttClient.setCallback(handleMQTTMessage);
}

void handleMQTTMessage(char *topic, byte *payload, unsigned int length)
{
  String newTopic = topic;
  payload[length] = '\0';
  String newPayload = String((char *)payload);

#ifdef DEBUG
  Serial.print("[MQTT]: handleMQTTMessage - Message arrived, topic: ");
  Serial.print(topic);
  Serial.print(", payload: ");
  Serial.println(newPayload);
  Serial.println();
#endif

  if (newTopic == MQTT_STEPPER_ACTION_COMMAND_TOPIC)
  {
    if (newPayload == "CLOSE")
    {
      newPosition = STEPS_TO_CLOSE;
    }
    if (newPayload == "OPEN")
    {
      newPosition = 0;
    }
    if (newPayload == "STOP")
    {
      newPosition = currentPosition;
    }
  }
  if (newTopic == MQTT_STEPPER_POSITION_COMMAND_TOPIC)
  {
    int intPayload = newPayload.toInt();
    if (boot == true)
    {
      newPosition = intPayload;
      currentPosition = intPayload;
      boot = false;
    }
    if (boot == false)
    {
      newPosition = intPayload;
    }
  }
  if (newTopic == MQTT_DOOR_LOCK_COMMAND_TOPIC)
  {
    Serial.print(F("[MQTT]: MQTT_DOOR_LOCK_COMMAND_TOPIC: "));
    Serial.println(newTopic);
    Serial.print(F("[MQTT]: equalsIgnoreCase: "));
    Serial.println(newPayload.equalsIgnoreCase(MQTT_PAYLOAD_ON) );

    if (doorLock.setState(newPayload.equalsIgnoreCase(MQTT_PAYLOAD_ON) )){
      publishToMQTT(MQTT_DOOR_LOCK_TOPIC, doorLock.getState());
    }
  }
}

/*
  Function called to connect/reconnect to the MQTT broker
*/
void connectToMQTT()
{
  int retries = 0;
  // Loop until we're connected / reconnected
  while (!mqttClient.connected())
  {
    if (retries < 150)
    {
      Serial.println("[MQTT]: Attempting MQTT connection...");
      if (mqttClient.connect(ESP_CHIP_ID, MQTT_USERNAME, MQTT_PASSWORD, MQTT_AVAILABILITY_TOPIC, 0, 1, "offline"))
      {

        Serial.println(F("[MQTT]: The mqttClient is successfully connected to the MQTT broker"));
        publishToMQTT(MQTT_AVAILABILITY_TOPIC, "online");
        if (boot == false)
        {
          Serial.println(F("[MQTT]: Reconnected"));
        }
        if (boot == true)
        {
          Serial.println(F("[MQTT]: Rebooted"));
        }
        
        publishToMQTT(MQTT_DOOR_LOCK_TOPIC, doorLock.getState());

        handleSwartNinjaSensorUpdate(doorSensor.getCurrentState(), doorSensor.getPinNumber(), SN_RSW_SENSOR_EVT);

        // ... and resubscribe
        subscribeToMQTT(MQTT_STEPPER_ACTION_COMMAND_TOPIC);
        subscribeToMQTT(MQTT_STEPPER_POSITION_COMMAND_TOPIC);
        subscribeToMQTT(MQTT_DOOR_LOCK_COMMAND_TOPIC);
      }
      else
      {
        retries++;
#ifdef DEBUG
        Serial.println(F("[MQTT]: ERROR - The connection to the MQTT broker failed"));
        Serial.print(F("[MQTT]: MQTT username: "));
        Serial.println(MQTT_USERNAME);
        Serial.print(F("[MQTT]: MQTT password: "));
        Serial.println(MQTT_PASSWORD);
        Serial.print(F("[MQTT]: MQTT broker: "));
        Serial.println(MQTT_SERVER);
        Serial.print(F("[MQTT]: Retries: "));
        Serial.println(retries);
#endif
        // Wait 5 seconds before retrying
        delay(5000);
      }
    }
    if (retries > 149)
    {
      ESP.restart();
    }
  }
}

void checkInMQTT()
{
  publishToMQTT(MQTT_AVAILABILITY_TOPIC, "online", false);
  timer.setTimeout(120000, checkInMQTT);
}

/*
  Function called to subscribe to a MQTT topic
*/
void subscribeToMQTT(char *p_topic)
{
  if (mqttClient.subscribe(p_topic))
  {
    Serial.print(F("[MQTT]: subscribeToMQTT - Sending the MQTT subscribe succeeded for topic: "));
    Serial.println(p_topic);
  }
  else
  {
    Serial.print(F("[MQTT]: subscribeToMQTT - ERROR, Sending the MQTT subscribe failed for topic: "));
    Serial.println(p_topic);
  }
}

/*
  Function called to publish to a MQTT topic with the given payload
*/
void publishToMQTT(char *p_topic, char *p_payload, bool retain)
{
  if (mqttClient.publish(p_topic, p_payload, retain))
  {
    Serial.print(F("[MQTT]: publishToMQTT - MQTT message published successfully, topic: "));
    Serial.print(p_topic);
    Serial.print(F(", payload: "));
    Serial.println(p_payload);
  }
  else
  {
    Serial.println(F("[MQTT]: publishToMQTT - ERROR, MQTT message not published, either connection lost, or message too large. Topic: "));
    Serial.print(p_topic);
    Serial.print(F(" , payload: "));
    Serial.println(p_payload);
  }
}
