#include <Arduino.h>
#include <Servo.h>
#include <ESP8266WiFi.h>  // if you get an error here you need to install the ESP8266 board manager
#include <ESP8266mDNS.h>  // if you get an error here you need to install the ESP8266 board manager
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient
#include <ArduinoOTA.h>   // https://github.com/esp8266/Arduino/tree/master/libraries/ArduinoOTA
#include <ArduinoJson.h>  // https://github.com/bblanchon/ArduinoJson

#include "SwartNinjaRSW.h"
#include "SwartNinjaSW.h"
#include "SwartNinjaSMD.h"

#include "user_config.h" // Fixed user configurable options
#ifdef USE_CONFIG_OVERRIDE
#include "user_config_override.h" // Configuration overrides for my_user_config.h
#endif

///////////////////////////////////////////////////////////////////////////
//   General Declarations
///////////////////////////////////////////////////////////////////////////
void systemCheckAndSet();
char ESP_CHIP_ID[7] = {0};

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

// variables declaration
char buffer [sizeof(int)*8+1];
int previousWiFiSignalStrength = 0;
unsigned long previousMillis = 0;
int reqConnect = 0;
int isConnected = 0;
const long interval = 500;
const long reqConnectNum = 15; // number of intervals to wait for connection
WiFiEventHandler mConnectHandler;
WiFiEventHandler mDisConnectHandler;
WiFiEventHandler mGotIpHandler;

String ipAddress;
String subnet;
String gateway;
String connectedAPMac;
// Initialize the Ethernet mqttClient object
WiFiClient wifiClient;

/* -------------------------------------------------
 *  MQTT
 * ------------------------------------------------- */
// function declaration
void setupMQTT();
void publishAllState();
void publishDeviceInfo();
void connectToMQTT();
void checkInMQTT();
void subscribeToMQTT(char *p_topic);
bool publishToMQTT(const char *p_topic, const char *p_payload, bool retain = true);
void handleMQTTMessage(char *topic, byte *payload, unsigned int length);

// variables declaration
bool boot = true;
char MQTT_PAYLOAD[8] = {0};
char MQTT_DEVICE_INFO_TOPIC[sizeof(DEVICE_NAME) + sizeof(MQTT_DEVICE_INFO_TEMPLATE) - 2] = {0};
char MQTT_DEVICE_COMMAND_TOPIC[sizeof(DEVICE_NAME) + sizeof(MQTT_DEVICE_COMMAND_TEMPLATE) - 2] = {0};

// status
char MQTT_DEVICE_STATUS_BASE[sizeof(MQTT_BINARY_SENSOR_TEMPLATE) + sizeof(DEVICE_NAME) + sizeof(DEVICE_STATUS) - 4] = {0};
char MQTT_DEVICE_STATUS_STATE_TOPIC[sizeof(MQTT_STATE_TEMPLATE) + sizeof(MQTT_DEVICE_STATUS_BASE) - 2] = {0};
char MQTT_DEVICE_STATUS_DISCOVERY_TOPIC[sizeof(MQTT_DISCOVERY_TEMPLATE) + sizeof(MQTT_DEVICE_STATUS_BASE) - 2] = {0};

// reset
char MQTT_DEVICE_RESET_BASE[sizeof(MQTT_SWITCH_TEMPLATE) + sizeof(DEVICE_NAME) + sizeof(MQTT_CMD_RESET) - 4] = {0};
char MQTT_DEVICE_RESET_STATE_TOPIC[sizeof(MQTT_STATE_TEMPLATE) + sizeof(MQTT_DEVICE_RESET_BASE) - 2] = {0};
char MQTT_DEVICE_RESET_DISCOVERY_TOPIC[sizeof(MQTT_DISCOVERY_TEMPLATE) + sizeof(MQTT_DEVICE_RESET_BASE) - 2] = {0};

// wifi sensor
char MQTT_WIFI_SIGNAL_STRENGTH_BASE[sizeof(MQTT_SENSOR_TEMPLATE) + sizeof(DEVICE_NAME) + sizeof(WIFI_SIGNAL_STRENGTH_SENSOR_NAME) - 4] = {0};
char MQTT_WIFI_SIGNAL_STRENGTH_STATE_TOPIC[sizeof(MQTT_STATE_TEMPLATE) + sizeof(MQTT_WIFI_SIGNAL_STRENGTH_BASE) - 2] = {0};
char MQTT_WIFI_SIGNAL_STRENGTH_DISCOVERY_TOPIC[sizeof(MQTT_DISCOVERY_TEMPLATE) + sizeof(MQTT_WIFI_SIGNAL_STRENGTH_BASE) - 2] = {0};

// siren
char MQTT_SIREN_BASE[sizeof(MQTT_SWITCH_TEMPLATE) + sizeof(DEVICE_NAME) + sizeof(SIREN_NAME) - 4] = {0};
char MQTT_SIREN_STATE_TOPIC[sizeof(MQTT_STATE_TEMPLATE) + sizeof(MQTT_SIREN_BASE) - 2] = {0};
char MQTT_SIREN_COMMAND_TOPIC[sizeof(MQTT_COMMAND_TEMPLATE) + sizeof(MQTT_SIREN_BASE) - 2] = {0};
char MQTT_SIREN_DISCOVERY_TOPIC[sizeof(MQTT_DISCOVERY_TEMPLATE) + sizeof(MQTT_SIREN_BASE) - 2] = {0};

// door sensor
char MQTT_DOOR_BASE[sizeof(MQTT_BINARY_SENSOR_TEMPLATE) + sizeof(DEVICE_NAME) + sizeof(DOOR_SENSOR_NAME) - 4] = {0};
char MQTT_DOOR_STATE_TOPIC[sizeof(MQTT_STATE_TEMPLATE) + sizeof(MQTT_DOOR_BASE) - 2] = {0};
char MQTT_DOOR_DISCOVERY_TOPIC[sizeof(MQTT_DISCOVERY_TEMPLATE) + sizeof(MQTT_DOOR_BASE) - 2] = {0};

// lock
char MQTT_LOCK_BASE[sizeof(MQTT_LOCK_TEMPLATE) + sizeof(DEVICE_NAME) + sizeof(LOCK_NAME) - 4] = {0};
char MQTT_LOCK_STATE_TOPIC[sizeof(MQTT_STATE_TEMPLATE) + sizeof(MQTT_LOCK_BASE) - 2] = {0};
char MQTT_LOCK_COMMAND_TOPIC[sizeof(MQTT_COMMAND_TEMPLATE) + sizeof(MQTT_LOCK_BASE) - 2] = {0};
char MQTT_LOCK_DISCOVERY_TOPIC[sizeof(MQTT_DISCOVERY_TEMPLATE) + sizeof(MQTT_LOCK_BASE) - 2] = {0};

// cover
char MQTT_COVER_BASE[sizeof(MQTT_COVER_TEMPLATE) + sizeof(DEVICE_NAME) + sizeof(STEPPER_NAME) - 4] = {0};
char MQTT_COVER_STATE_TOPIC[sizeof(MQTT_STATE_TEMPLATE) + sizeof(MQTT_COVER_BASE) - 2] = {0};
char MQTT_COVER_COMMAND_TOPIC[sizeof(MQTT_COMMAND_TEMPLATE) + sizeof(MQTT_COVER_BASE) - 2] = {0};
char MQTT_COVER_DISCOVERY_TOPIC[sizeof(MQTT_DISCOVERY_TEMPLATE) + sizeof(MQTT_COVER_BASE) - 2] = {0};
// cover position
char MQTT_COVER_POSITION_BASE[sizeof(MQTT_JOIN_TEMPLATE) + sizeof(MQTT_COVER_BASE) + sizeof(STEPPER_POSITION_NAME) - 4] = {0};
char MQTT_COVER_POSITION_STATE_TOPIC[sizeof(MQTT_STATE_TEMPLATE) + sizeof(MQTT_COVER_POSITION_BASE) - 2] = {0};
char MQTT_COVER_POSITION_COMMAND_TOPIC[sizeof(MQTT_COMMAND_TEMPLATE) + sizeof(MQTT_COVER_POSITION_BASE) - 2] = {0};

// Initialize the mqtt mqttClient object
PubSubClient mqttClient(wifiClient);

///////////////////////////////////////////////////////////////////////////
//   Stepper
///////////////////////////////////////////////////////////////////////////
// function declaration
void processStepper();
void publishStepperState();
void publishStepperPosition();

// variables declaration
int currentPosition = STEPPER_POSITION_CLOSED;
int newPosition = STEPPER_POSITION_CLOSED;
int lastPublishedStepperPosition;

String stepperState = MQTT_PAYLOAD_CLOSE;
String lastPublishedStepperState;

// Initialize the SwartNinjaSMD object
SwartNinjaSMD doorStepper(SwartNinjaSMDMode::TB6600_eight, STEPPER_DIR_PIN, STEPPER_PUL_PIN, STEPPER_ENE_PIN);

///////////////////////////////////////////////////////////////////////////
//   SwartNinjaSensors
///////////////////////////////////////////////////////////////////////////
// function declaration
void handleSwartNinjaSensorUpdate(char *value, int pin, const char *event);

// initialize the SwartNinaRSW object
SwartNinjaRSW doorSensor(DOOR_PIN, handleSwartNinjaSensorUpdate, false);
SwartNinjaSW siren(SIREN_PIN);

///////////////////////////////////////////////////////////////////////////
//   Lock
///////////////////////////////////////////////////////////////////////////
void publishLockState();
// initialize the servo objects
Servo doorLock;
bool isLocked = false;

///////////////////////////////////////////////////////////////////////////
//   Home Assistant
///////////////////////////////////////////////////////////////////////////
void hassAutoConfig();
bool registerSensor(DynamicJsonDocument doc, char *topic);
void unregisterSensors();

///////////////////////////////////////////////////////////////////////////
//   MAIN SETUP AND LOOP
///////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(115200);

  // Set the chip ID
  sprintf(ESP_CHIP_ID, "%06X", ESP.getChipId());

  // WIFI
  setupWiFi();

  // MQTT
  setupMQTT();

  // Over the air
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.begin();

  Serial.print(F("[OTA] HOSTNAME: "));
  Serial.println(OTA_HOSTNAME);

  delay(10);
  siren.setup();
  doorSensor.setup();
  doorLock.attach(LOCK_PIN);
  doorLock.write(LOCK_POSITION_OPEN);

  // stepper motor
  doorStepper.disableDriver();
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
    processStepper();

    // Check WiFi signal
    loopWiFiSensor();
    checkInMQTT();
  }
}

///////////////////////////////////////////////////////////////////////////
//   Stepper
///////////////////////////////////////////////////////////////////////////
void processStepper()
{
  if (isLocked)
  {
#ifdef DEBUG
    Serial.println("[DEBUG] processStepper - EXIT, door is locked.");
#endif
    return;
  }

  if (newPosition > currentPosition)
  {
    stepperState = MQTT_STATE_OPENING;
    doorStepper.enableDriver();
    doorStepper.revolve(-1);
    currentPosition++;
  }
  else if (newPosition < currentPosition)
  {
    stepperState = MQTT_STATE_CLOSING;
    doorStepper.enableDriver();
    doorStepper.revolve(1);
    currentPosition--;
  }
  else if (newPosition == currentPosition && (stepperState != MQTT_STATE_CLOSED || stepperState != MQTT_STATE_CLOSED))
  {
    doorStepper.disableDriver();
    stepperState = (currentPosition == STEPPER_POSITION_CLOSED) ? MQTT_STATE_CLOSED : MQTT_STATE_OPEN;
  }

  if (!stepperState.equalsIgnoreCase(lastPublishedStepperState))
  {
    publishStepperState();
  }

  if (currentPosition != lastPublishedStepperPosition)
  {
    publishStepperPosition();
  }
}

void publishStepperState()
{
  char payload[stepperState.length() + 1];
  stepperState.toCharArray(payload, stepperState.length() + 1);

  if (publishToMQTT(MQTT_COVER_STATE_TOPIC, payload, true))
  {
    lastPublishedStepperState = stepperState;
  }
}

void publishStepperPosition()
{
  char payload[sizeof(int) * 8 + 1];
  itoa(currentPosition, payload, 10);

  if (publishToMQTT(MQTT_COVER_POSITION_STATE_TOPIC, payload, true))
  {
    lastPublishedStepperPosition = currentPosition;
  }
}

///////////////////////////////////////////////////////////////////////////
//   Lock
///////////////////////////////////////////////////////////////////////////
void publishLockState()
{
  char *statePayload = strdup((isLocked) ? MQTT_STATE_LOCKED : MQTT_STATE_UNLOCKED);
  publishToMQTT(MQTT_LOCK_STATE_TOPIC, statePayload, true);
}

///////////////////////////////////////////////////////////////////////////
//   SwartNinjaSensors
///////////////////////////////////////////////////////////////////////////
void handleSwartNinjaSensorUpdate(char *value, int pin, const char *event)
{
  if (strcmp(event, SN_RSW_SENSOR_EVT) == 0)
  {
    // publish door open or closed
    publishToMQTT(MQTT_DOOR_STATE_TOPIC, value, true);
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
  sprintf(macAdddress, "%02X:%02X:%02X:%02X:%02X:%02X", event.bssid[0], event.bssid[1], event.bssid[2], event.bssid[3], event.bssid[4], event.bssid[5]);
  connectedAPMac = macAdddress;

  Serial.print(F("[WIFI]: You're connected to the AP. (MAC - "));
  Serial.print(macAdddress);
  Serial.println(")");
  isConnected = 1;
}

void onDisconnect(const WiFiEventStationModeDisconnected &event)
{
  String reason;
  switch (event.reason)
  {
  case WiFiDisconnectReason::WIFI_DISCONNECT_REASON_UNSPECIFIED:
    reason = "UNSPECIFIED";
    break;
  case WiFiDisconnectReason::WIFI_DISCONNECT_REASON_AUTH_EXPIRE:
    reason = "AUTH EXPIRE";
    break;
  case WiFiDisconnectReason::WIFI_DISCONNECT_REASON_AUTH_LEAVE:
    reason = "AUTH LEAVE";
    break;
  case WiFiDisconnectReason::WIFI_DISCONNECT_REASON_ASSOC_EXPIRE:
    reason = "ASSOC EXPIRE";
    break;
  case WiFiDisconnectReason::WIFI_DISCONNECT_REASON_ASSOC_TOOMANY:
    reason = "ASSOC TOOMANY";
    break;
  case WiFiDisconnectReason::WIFI_DISCONNECT_REASON_NOT_AUTHED:
    reason = "NOT AUTHED";
    break;
  case WiFiDisconnectReason::WIFI_DISCONNECT_REASON_NOT_ASSOCED:
    reason = "NOT ASSOCED";
    break;
  case WiFiDisconnectReason::WIFI_DISCONNECT_REASON_ASSOC_LEAVE:
    reason = "ASSOC LEAVE";
    break;
  case WiFiDisconnectReason::WIFI_DISCONNECT_REASON_ASSOC_NOT_AUTHED:
    reason = "ASSOC NOT AUTHED";
    break;
  case WiFiDisconnectReason::WIFI_DISCONNECT_REASON_DISASSOC_PWRCAP_BAD:
    reason = "DISASSOC PWRCAP BAD";
    break;
  case WiFiDisconnectReason::WIFI_DISCONNECT_REASON_DISASSOC_SUPCHAN_BAD:
    reason = "DISASSOC SUPCHAN BAD";
    break;
  case WiFiDisconnectReason::WIFI_DISCONNECT_REASON_IE_INVALID:
    reason = "IE INVALID";
    break;
  case WiFiDisconnectReason::WIFI_DISCONNECT_REASON_MIC_FAILURE:
    reason = "MIC FAILURE";
    break;
  case WiFiDisconnectReason::WIFI_DISCONNECT_REASON_4WAY_HANDSHAKE_TIMEOUT:
    reason = "4WAY HANDSHAKE TIMEOUT";
    break;
  case WiFiDisconnectReason::WIFI_DISCONNECT_REASON_GROUP_KEY_UPDATE_TIMEOUT:
    reason = "GROUP KEY UPDATE TIMEOUT";
    break;
  case WiFiDisconnectReason::WIFI_DISCONNECT_REASON_IE_IN_4WAY_DIFFERS:
    reason = "IE IN 4WAY DIFFERS";
    break;
  case WiFiDisconnectReason::WIFI_DISCONNECT_REASON_GROUP_CIPHER_INVALID:
    reason = "GROUP CIPHER INVALID";
    break;
  case WiFiDisconnectReason::WIFI_DISCONNECT_REASON_PAIRWISE_CIPHER_INVALID:
    reason = "PAIRWISE CIPHER INVALID";
    break;
  case WiFiDisconnectReason::WIFI_DISCONNECT_REASON_AKMP_INVALID:
    reason = "AKMP INVALID";
    break;
  case WiFiDisconnectReason::WIFI_DISCONNECT_REASON_UNSUPP_RSN_IE_VERSION:
    reason = "UNSUPP RSN IE VERSION";
    break;
  case WiFiDisconnectReason::WIFI_DISCONNECT_REASON_INVALID_RSN_IE_CAP:
    reason = "INVALID RSN IE CAP";
    break;
  case WiFiDisconnectReason::WIFI_DISCONNECT_REASON_802_1X_AUTH_FAILED:
    reason = "802 1X AUTH FAILED";
    break;
  case WiFiDisconnectReason::WIFI_DISCONNECT_REASON_CIPHER_SUITE_REJECTED:
    reason = "CIPHER SUITE REJECTED";
    break;
  case WiFiDisconnectReason::WIFI_DISCONNECT_REASON_BEACON_TIMEOUT:
    reason = "BEACON TIMEOUT";
    break;
  case WiFiDisconnectReason::WIFI_DISCONNECT_REASON_NO_AP_FOUND:
    reason = "NO AP FOUND";
    break;
  case WiFiDisconnectReason::WIFI_DISCONNECT_REASON_AUTH_FAIL:
    reason = "AUTH FAIL";
    break;
  case WiFiDisconnectReason::WIFI_DISCONNECT_REASON_ASSOC_FAIL:
    reason = "ASSOC FAIL";
    break;
  case WiFiDisconnectReason::WIFI_DISCONNECT_REASON_HANDSHAKE_TIMEOUT:
    reason = "HANDSHAKE TIMEOUT";
    break;
  default:
    reason = "Unknown";
    break;
  }

  Serial.println("[WIFI]: Disconnected");
  Serial.print("[WIFI]: Reason: ");
  Serial.println(reason);
  isConnected = 0;
}

void onGotIP(const WiFiEventStationModeGotIP &event)
{
  ipAddress = event.ip.toString();
  subnet = event.mask.toString();
  gateway = event.gw.toString();

  Serial.print("[WIFI]: IP Address : ");
  Serial.println(ipAddress);
  Serial.print("[WIFI]: Subnet     : ");
  Serial.println(subnet);
  Serial.print("[WIFI]: Gateway    : ");
  Serial.println(gateway);

  isConnected = 2;
}

/*
 * Function to check WiFi signal strength
 */
void loopWiFiSensor(void)
{
  static unsigned long lastWiFiQualityMeasure = 0;
  if (lastWiFiQualityMeasure + WIFI_SIGNAL_STRENGTH_INTERVAL <= millis() || previousWiFiSignalStrength == 0)
  {
    lastWiFiQualityMeasure = millis();
    int currentWiFiSignalStrength = WiFi.RSSI();
    if (isnan(previousWiFiSignalStrength) || currentWiFiSignalStrength <= previousWiFiSignalStrength - WIFI_SIGNAL_STRENGTH_OFFSET_VALUE || currentWiFiSignalStrength >= previousWiFiSignalStrength + WIFI_SIGNAL_STRENGTH_OFFSET_VALUE)
    {
      previousWiFiSignalStrength = currentWiFiSignalStrength;
      itoa(currentWiFiSignalStrength, MQTT_PAYLOAD, 10);
      publishToMQTT(MQTT_WIFI_SIGNAL_STRENGTH_STATE_TOPIC, MQTT_PAYLOAD);
    }
  }
}

///////////////////////////////////////////////////////////////////////////
//   Home Assistant
///////////////////////////////////////////////////////////////////////////

void hassAutoConfig()
{
  char unique_id[40];
  DynamicJsonDocument config(1024);
  Serial.println("hassAutoConfig - Start");

  config.clear();
  sprintf(unique_id, "%s_reset", DEVICE_NAME);
  config["uniq_id"] = unique_id;
  config["name"] = "Office Door System Reset";
  config["ic"] = "mdi:restart";
  config["cmd_t"] = MQTT_DEVICE_COMMAND_TOPIC;       //command_topic
  config["stat_t"] = MQTT_DEVICE_RESET_STATE_TOPIC;  //state_topic
  config["stat_on"] = MQTT_PAYLOAD_ON;               //payload_off
  config["stat_off"] = MQTT_PAYLOAD_ON;              //payload_off
  config["pl_off"] = MQTT_PAYLOAD_OFF;               //payload_off
  config["pl_on"] = MQTT_CMD_RESET;                  //payload_on
  registerSensor(config, MQTT_DEVICE_RESET_DISCOVERY_TOPIC);

  config.clear();
  sprintf(unique_id, "%s_status", DEVICE_NAME);
  config["uniq_id"] = unique_id;
  config["name"] = "Office Door System Status";
  config["dev_cla"] = "connectivity";                 //device_class
  config["stat_t"] = MQTT_DEVICE_STATUS_STATE_TOPIC;  //state_topic
  config["json_attr_t"] = MQTT_DEVICE_INFO_TOPIC;     //json_attributes_topic
  config["pl_off"] = MQTT_PAYLOAD_NOT_AVAILABLE;      //payload_off
  config["pl_on"] = MQTT_PAYLOAD_AVAILABLE;           //payload_on
  registerSensor(config, MQTT_DEVICE_STATUS_DISCOVERY_TOPIC);

  config.clear();
  sprintf(unique_id, "%s_wifi_signal_strength", DEVICE_NAME);
  config["uniq_id"] = unique_id;
  config["name"] = "Office Door System WiFi";
  config["ic"] = "mdi:wifi";
  config["dev_cla"] = "signal_strength";                    // device_class
  config["unit_of_meas"] = "dB";                            // unit_of_measurement
  config["stat_t"] = MQTT_WIFI_SIGNAL_STRENGTH_STATE_TOPIC; // state_topic
  registerSensor(config, MQTT_WIFI_SIGNAL_STRENGTH_DISCOVERY_TOPIC);

  config.clear();
  sprintf(unique_id, "%s_door", DEVICE_NAME);
  config["uniq_id"] = unique_id;
  config["name"] = "Office Door System Door";
  config["dev_cla"] = "door";               //device_class
  config["stat_t"] = MQTT_DOOR_STATE_TOPIC; //state_topic
  config["pl_off"] = MQTT_PAYLOAD_OFF;      //payload_off
  config["pl_on"] = MQTT_PAYLOAD_ON;        //payload_on
  registerSensor(config, MQTT_DOOR_DISCOVERY_TOPIC);

  config.clear();
  sprintf(unique_id, "%s_siren", DEVICE_NAME);
  config["uniq_id"] = unique_id;
  config["name"] = "Office Door System Siren";
  config["ic"] = "mdi:alarm-bell";
  config["~"] = MQTT_SIREN_BASE; //topic base
  config["stat_t"] = "~/state";  //state_topic
  config["cmd_t"] = "~/set";     //command_topic
  registerSensor(config, MQTT_SIREN_DISCOVERY_TOPIC);

  config.clear();
  sprintf(unique_id, "%s_lock", DEVICE_NAME);
  config["uniq_id"] = unique_id;
  config["name"] = "Office Door System Lock";
  config["~"] = MQTT_LOCK_BASE;                  //topic base
  config["stat_t"] = "~/state";                  //state_topic
  config["stat_locked"] = MQTT_STATE_LOCKED;     //state_locked
  config["stat_unlocked"] = MQTT_STATE_UNLOCKED; //state_unlocked
  config["cmd_t"] = "~/set";                     //command_topic
  config["pl_lock"] = MQTT_PAYLOAD_LOCK;         //payload_lock
  config["pl_unlk"] = MQTT_PAYLOAD_UNLOCK;       //payload_unlock
  registerSensor(config, MQTT_LOCK_DISCOVERY_TOPIC);

  config.clear();
  sprintf(unique_id, "%s_cover", DEVICE_NAME);
  config["uniq_id"] = unique_id;
  config["name"] = "Office Door System Cover";
  config["dev_cla"] = "door";                   //device_class
  config["~"] = MQTT_COVER_BASE;                //topic base
  config["stat_t"] = "~/state";                 //state_topic
  config["cmd_t"] = "~/set";                    //state_topic
  config["pos_t"] = "~/position/state";         //position_topic
  config["set_pos_t"] = "~/position/set";       //set_position_topic
  config["pl_cls"] = MQTT_PAYLOAD_CLOSE;        //payload_close
  config["pl_open"] = MQTT_PAYLOAD_OPEN;        //payload_open
  config["pl_stop"] = MQTT_PAYLOAD_STOP;        //payload_stop
  config["pos_clsd"] = STEPPER_POSITION_CLOSED; //position_closed
  config["pos_open"] = STEPPER_POSITION_OPEN;   //position_open
  config["stat_clsd"] = MQTT_STATE_CLOSED;      //state_closed
  config["stat_closing"] = MQTT_STATE_CLOSING;  //state_closing
  config["stat_open"] = MQTT_STATE_OPEN;        //state_open
  config["stat_opening"] = MQTT_STATE_OPENING;  //state_opening
  registerSensor(config, MQTT_COVER_DISCOVERY_TOPIC);
}

/*
 * Add device to descovery topic and send config
 */
bool registerSensor(DynamicJsonDocument doc, char *topic)
{
  doc["avty_t"] = MQTT_DEVICE_STATUS_STATE_TOPIC; // availability_topic
  doc["pl_avail"] = MQTT_PAYLOAD_AVAILABLE;             // payload_available
  doc["pl_not_avail"] = MQTT_PAYLOAD_NOT_AVAILABLE;     // payload_not_available

  JsonObject device = doc.createNestedObject("dev");       // device
  JsonArray identifiers = device.createNestedArray("ids"); // identifiers
  identifiers.add(ESP_CHIP_ID);
  identifiers.add(DEVICE_NAME);
  device["name"] = DEVICE_FRIENDLY_NAME; // name
  device["mf"] = DEVICE_MANUFACTURER;    // manufacturer
  device["mdl"] = DEVICE_MODEL;          // model
  device["sw"] = DEVICE_VERSION;         // sw_version

  String output;
  serializeJson(doc, output);

  char *buffer = new char[output.length() + 1];
  strcpy(buffer, output.c_str());

#if defined(DEBUG)
  bool result = true;
  Serial.println("-----------------------------------------------");
  Serial.print("topic: ");
  Serial.println(topic);
  Serial.print("payload: ");
  Serial.println(buffer);
  Serial.println("-----------------------------------------------");
#else
  bool result = publishToMQTT(topic, buffer, true);
#endif

  // Cleanup
  delete[] buffer;

  return result;
}

void unregisterSensors()
{
  if (!publishToMQTT(MQTT_DEVICE_STATUS_DISCOVERY_TOPIC, "",true))
  {
    Serial.println("Failed to unregister availability sensor");
  }
  if (!publishToMQTT(MQTT_DEVICE_RESET_DISCOVERY_TOPIC, "",true))
  {
    Serial.println("Failed to unregister cover");
  }
  if (!publishToMQTT(MQTT_WIFI_SIGNAL_STRENGTH_DISCOVERY_TOPIC, "",true))
  {
    Serial.println("Failed to unregister wifi sensor");
  }

  if (!publishToMQTT(MQTT_DOOR_DISCOVERY_TOPIC, "",true))
  {
    Serial.println("Failed to unregister door sensor");
  }

  if (!publishToMQTT(MQTT_SIREN_DISCOVERY_TOPIC, "",true))
  {
    Serial.println("Failed to unregister siren");
  }

  if (!publishToMQTT(MQTT_LOCK_DISCOVERY_TOPIC, "",true))
  {
    Serial.println("Failed to unregister lock");
  }

  if (!publishToMQTT(MQTT_COVER_DISCOVERY_TOPIC, "",true))
  {
    Serial.println("Failed to unregister cover");
  }
}

///////////////////////////////////////////////////////////////////////////
//   MQTT
///////////////////////////////////////////////////////////////////////////

/*
 * Function called to setup MQTT topics
 */
void setupMQTT()
{
  Serial.println();
  Serial.println("[MQTT] -------------------------------- MQTT TOPICS --------------------------------");
  Serial.println();
  Serial.println("[MQTT] --- Device");
  sprintf(MQTT_DEVICE_STATUS_BASE, MQTT_BINARY_SENSOR_TEMPLATE, DEVICE_NAME, DEVICE_STATUS);

  sprintf(MQTT_DEVICE_STATUS_DISCOVERY_TOPIC, MQTT_DISCOVERY_TEMPLATE, MQTT_DEVICE_STATUS_BASE);
  Serial.print(F("[MQTT] Config: "));
  Serial.println(MQTT_DEVICE_STATUS_DISCOVERY_TOPIC);

  sprintf(MQTT_DEVICE_STATUS_STATE_TOPIC, MQTT_STATE_TEMPLATE, MQTT_DEVICE_STATUS_BASE);
  Serial.print(F("[MQTT] Status: "));
  Serial.println(MQTT_DEVICE_STATUS_STATE_TOPIC);

  sprintf(MQTT_DEVICE_INFO_TOPIC, MQTT_DEVICE_INFO_TEMPLATE, DEVICE_NAME);
  Serial.print(F("[MQTT] Info: "));
  Serial.println(MQTT_DEVICE_INFO_TOPIC);

  sprintf(MQTT_DEVICE_COMMAND_TOPIC, MQTT_DEVICE_COMMAND_TEMPLATE, DEVICE_NAME);
  Serial.print(F("[MQTT] Command: "));
  Serial.println(MQTT_DEVICE_COMMAND_TOPIC);

  Serial.println();
  Serial.println("[MQTT] --- Reset");
  sprintf(MQTT_DEVICE_RESET_BASE, MQTT_SWITCH_TEMPLATE, DEVICE_NAME, MQTT_CMD_RESET);

  sprintf(MQTT_DEVICE_RESET_DISCOVERY_TOPIC, MQTT_DISCOVERY_TEMPLATE, MQTT_DEVICE_RESET_BASE);
  Serial.print(F("[MQTT] Config: "));
  Serial.println(MQTT_SIREN_DISCOVERY_TOPIC);

  sprintf(MQTT_DEVICE_RESET_STATE_TOPIC, MQTT_STATE_TEMPLATE, MQTT_DEVICE_RESET_BASE);
  Serial.print(F("[MQTT] State: "));
  Serial.println(MQTT_DEVICE_RESET_STATE_TOPIC);

  Serial.println();
  Serial.println("[MQTT] --- WiFi Signal Strength");
  sprintf(MQTT_WIFI_SIGNAL_STRENGTH_BASE, MQTT_SENSOR_TEMPLATE, DEVICE_NAME, WIFI_SIGNAL_STRENGTH_SENSOR_NAME);

  sprintf(MQTT_WIFI_SIGNAL_STRENGTH_DISCOVERY_TOPIC, MQTT_DISCOVERY_TEMPLATE, MQTT_WIFI_SIGNAL_STRENGTH_BASE);
  Serial.print(F("[MQTT] Config: "));
  Serial.println(MQTT_WIFI_SIGNAL_STRENGTH_DISCOVERY_TOPIC);

  sprintf(MQTT_WIFI_SIGNAL_STRENGTH_STATE_TOPIC, MQTT_STATE_TEMPLATE, MQTT_WIFI_SIGNAL_STRENGTH_BASE);
  Serial.print(F("[MQTT] State: "));
  Serial.println(MQTT_WIFI_SIGNAL_STRENGTH_STATE_TOPIC);

  Serial.println();
  Serial.println("[MQTT] --- Door");
  sprintf(MQTT_DOOR_BASE, MQTT_BINARY_SENSOR_TEMPLATE, DEVICE_NAME, DOOR_SENSOR_NAME);

  sprintf(MQTT_DOOR_DISCOVERY_TOPIC, MQTT_DISCOVERY_TEMPLATE, MQTT_DOOR_BASE);
  Serial.print(F("[MQTT] Config: "));
  Serial.println(MQTT_DOOR_DISCOVERY_TOPIC);

  sprintf(MQTT_DOOR_STATE_TOPIC, MQTT_STATE_TEMPLATE, MQTT_DOOR_BASE);
  Serial.print(F("[MQTT] State: "));
  Serial.println(MQTT_DOOR_STATE_TOPIC);

  Serial.println();
  Serial.println("[MQTT] --- Siren");
  sprintf(MQTT_SIREN_BASE, MQTT_SWITCH_TEMPLATE, DEVICE_NAME, SIREN_NAME);

  sprintf(MQTT_SIREN_DISCOVERY_TOPIC, MQTT_DISCOVERY_TEMPLATE, MQTT_SIREN_BASE);
  Serial.print(F("[MQTT] Config: "));
  Serial.println(MQTT_SIREN_DISCOVERY_TOPIC);

  sprintf(MQTT_SIREN_STATE_TOPIC, MQTT_STATE_TEMPLATE, MQTT_SIREN_BASE);
  Serial.print(F("[MQTT] State: "));
  Serial.println(MQTT_SIREN_STATE_TOPIC);

  sprintf(MQTT_SIREN_COMMAND_TOPIC, MQTT_COMMAND_TEMPLATE, MQTT_SIREN_BASE);
  Serial.print(F("[MQTT] Command: "));
  Serial.println(MQTT_SIREN_COMMAND_TOPIC);

  Serial.println();
  Serial.println("[MQTT] --- Lock");
  sprintf(MQTT_LOCK_BASE, MQTT_LOCK_TEMPLATE, DEVICE_NAME, LOCK_NAME);

  sprintf(MQTT_LOCK_DISCOVERY_TOPIC, MQTT_DISCOVERY_TEMPLATE, MQTT_LOCK_BASE);
  Serial.print(F("[MQTT] Config: "));
  Serial.println(MQTT_LOCK_DISCOVERY_TOPIC);

  sprintf(MQTT_LOCK_STATE_TOPIC, MQTT_STATE_TEMPLATE, MQTT_LOCK_BASE);
  Serial.print(F("[MQTT] State: "));
  Serial.println(MQTT_LOCK_STATE_TOPIC);

  sprintf(MQTT_LOCK_COMMAND_TOPIC, MQTT_COMMAND_TEMPLATE, MQTT_LOCK_BASE);
  Serial.print(F("[MQTT] Command: "));
  Serial.println(MQTT_LOCK_COMMAND_TOPIC);

  Serial.println();
  Serial.println("[MQTT] --- Cover");
  sprintf(MQTT_COVER_BASE, MQTT_COVER_TEMPLATE, DEVICE_NAME, STEPPER_NAME);
  sprintf(MQTT_COVER_POSITION_BASE, MQTT_JOIN_TEMPLATE, MQTT_COVER_BASE, STEPPER_POSITION_NAME);

  sprintf(MQTT_COVER_DISCOVERY_TOPIC, MQTT_DISCOVERY_TEMPLATE, MQTT_COVER_BASE);
  Serial.print(F("[MQTT] Config: "));
  Serial.println(MQTT_COVER_DISCOVERY_TOPIC);

  sprintf(MQTT_COVER_STATE_TOPIC, MQTT_STATE_TEMPLATE, MQTT_COVER_BASE);
  Serial.print(F("[MQTT] State: "));
  Serial.println(MQTT_COVER_STATE_TOPIC);

  sprintf(MQTT_COVER_COMMAND_TOPIC, MQTT_COMMAND_TEMPLATE, MQTT_COVER_BASE);
  Serial.print(F("[MQTT] Command: "));
  Serial.println(MQTT_COVER_COMMAND_TOPIC);

  sprintf(MQTT_COVER_POSITION_STATE_TOPIC, MQTT_STATE_TEMPLATE, MQTT_COVER_POSITION_BASE);
  Serial.print(F("[MQTT] Position State: "));
  Serial.println(MQTT_COVER_POSITION_STATE_TOPIC);

  sprintf(MQTT_COVER_POSITION_COMMAND_TOPIC, MQTT_COMMAND_TEMPLATE, MQTT_COVER_POSITION_BASE);
  Serial.print(F("[MQTT] Position Command: "));
  Serial.println(MQTT_COVER_POSITION_COMMAND_TOPIC);

  Serial.println("----------------------------------------------------------------------------");

  mqttClient.setServer(MQTT_SERVER, MQTT_SERVER_PORT);
  mqttClient.setCallback(handleMQTTMessage);
}

void publishDeviceInfo()
{
  DynamicJsonDocument device(1024);

  device["id"] = ESP_CHIP_ID;
  device["name"] = DEVICE_FRIENDLY_NAME;
  device["manufacturer"] = DEVICE_MANUFACTURER;
  device["model"] = DEVICE_MODEL;
  device["version"] = DEVICE_VERSION;
  device["ip_address"] = ipAddress;
  device["subnet"] = subnet;
  device["gateway"] = gateway;
  device["connected_ap_mac"] = connectedAPMac;

  String output;
  serializeJson(device, output);

  char *payload = new char[output.length() + 1];
  strcpy(payload, output.c_str());

  publishToMQTT(MQTT_DEVICE_INFO_TOPIC, payload, true);
}

void publishAllState()
{
  publishDeviceInfo();

  // wifi signal strength
  previousWiFiSignalStrength = WiFi.RSSI();
  itoa(previousWiFiSignalStrength, MQTT_PAYLOAD, 10);
  publishToMQTT(MQTT_WIFI_SIGNAL_STRENGTH_STATE_TOPIC, MQTT_PAYLOAD);

  // status
  publishToMQTT(MQTT_DEVICE_STATUS_STATE_TOPIC, MQTT_PAYLOAD_AVAILABLE, true);
  // reset
  publishToMQTT(MQTT_DEVICE_RESET_STATE_TOPIC, MQTT_PAYLOAD_OFF, true);

  // siren
  publishToMQTT(MQTT_SIREN_STATE_TOPIC, siren.getState(), true);

  // binary door sensor
  publishToMQTT(MQTT_DOOR_STATE_TOPIC, doorSensor.getCurrentState(), true);

  // lock state
  publishLockState();

  // cover state and position
  publishStepperState();
  publishStepperPosition();
}

void systemCheckAndSet()
{
  bool isClosed = !doorSensor.getCurrentRawState();

#ifdef DEBUG
  Serial.print("[DEBUG] System Setup - Door is ");
  Serial.println((isClosed) ? "CLOSED" : "OPEN");
#endif

  newPosition = (isClosed) ? STEPPER_POSITION_CLOSED : STEPPER_POSITION_OPEN;
  currentPosition = newPosition;
  stepperState = (currentPosition == STEPPER_POSITION_CLOSED) ? MQTT_STATE_CLOSED : MQTT_STATE_OPEN;

  doorLock.write(LOCK_POSITION_OPEN);
  isLocked = (doorLock.read() != LOCK_POSITION_OPEN);
}

void checkInMQTT()
{
  static unsigned long lastCheckIn = 0;
  if (lastCheckIn + MQTT_CHECKIN_INTERVAL <= millis())
  {
    lastCheckIn = millis();
    publishToMQTT(MQTT_DEVICE_STATUS_STATE_TOPIC, MQTT_PAYLOAD_AVAILABLE, true);
  }
}

void handleMQTTMessage(char *topic, byte *payload, unsigned int length)
{
  String strTopic = topic;
  payload[length] = '\0';
  String strPayload = String((char *)payload);

#ifdef MQTT_DEBUG
  Serial.print("[MQTT_DEBUG] handleMQTTMessage - Message arrived, topic: ");
  Serial.print(strTopic);
  Serial.print(", payload: ");
  Serial.println(strPayload);
  Serial.println();
#endif

  if (strTopic.equals(MQTT_DEVICE_COMMAND_TOPIC))
  {
    if (strPayload.equalsIgnoreCase(MQTT_CMD_RESET))
    {
      Serial.println("Reset device");
      systemCheckAndSet();
      publishAllState();
    }
    else if (strPayload.equalsIgnoreCase(MQTT_CMD_RESTART))
    {
      Serial.println("Restarting device");
      ESP.restart();
    }
    else if (strPayload.equalsIgnoreCase(MQTT_CMD_STATE))
    {
      Serial.println("Sending all sensor state");
      publishAllState();
    }
    else if (strPayload.equalsIgnoreCase(MQTT_CMD_REGISTER))
    {
      Serial.println("Forcing registration of sensor");
      hassAutoConfig();
      publishAllState();
    }
    else if (strPayload.equalsIgnoreCase(MQTT_CMD_UNREGISTER))
    {
      Serial.println("Forcing unregistration of sensor");
      unregisterSensors();
    }
  }
  else if (strTopic.equals(MQTT_COVER_COMMAND_TOPIC))
  {
    // if the door is locked dont change anything
    if (isLocked)
    {
      return;
    }
    if (strPayload.equalsIgnoreCase(MQTT_PAYLOAD_CLOSE))
    {
      newPosition = STEPPER_POSITION_CLOSED;
    }
    else if (strPayload.equalsIgnoreCase(MQTT_PAYLOAD_OPEN))
    {
      newPosition = STEPPER_POSITION_OPEN;
    }
    else if (strPayload.equalsIgnoreCase(MQTT_PAYLOAD_STOP))
    {
      newPosition = currentPosition;
    }
  }
  else if (strTopic.equals(MQTT_COVER_POSITION_COMMAND_TOPIC))
  {
    // if the door is locked dont change anything
    if (isLocked)
    {
      return;
    }
    newPosition = strPayload.toInt();
  }
  else if (strTopic.equals(MQTT_LOCK_COMMAND_TOPIC))
  {
    // check the door sensor to make sure the door is closed.
    if (strPayload.equalsIgnoreCase(MQTT_PAYLOAD_LOCK) && !doorSensor.getCurrentRawState())
    {
      doorLock.write(LOCK_POSITION_CLOSED);
    }
    else if (strPayload.equalsIgnoreCase(MQTT_PAYLOAD_UNLOCK))
    {
      doorLock.write(LOCK_POSITION_OPEN);
    }
    isLocked = (doorLock.read() != LOCK_POSITION_OPEN);
    publishLockState();
  }
  else if (strTopic.equals(MQTT_SIREN_COMMAND_TOPIC))
  {
    if (siren.setState(strPayload.equalsIgnoreCase(MQTT_PAYLOAD_ON)))
    {
      publishToMQTT(MQTT_SIREN_STATE_TOPIC, siren.getState(), true);
    }
  }
  else if (strTopic.equals(HOME_ASSISTANT_LWT_TOPIC))
  {
    if (strPayload.equalsIgnoreCase(MQTT_PAYLOAD_AVAILABLE))
    {
      systemCheckAndSet();
      hassAutoConfig();
      publishAllState();
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
      if (mqttClient.connect(DEVICE_NAME, MQTT_USERNAME, MQTT_PASSWORD, MQTT_DEVICE_STATUS_STATE_TOPIC, 0, 1, MQTT_PAYLOAD_NOT_AVAILABLE))
      {

        Serial.println(F("[MQTT]: The mqttClient is successfully connected to the MQTT broker"));
        publishToMQTT(MQTT_DEVICE_STATUS_STATE_TOPIC, MQTT_PAYLOAD_AVAILABLE, true);
        if (boot)
        {
          Serial.println(F("[MQTT]: Connected"));
          systemCheckAndSet();
          hassAutoConfig();
          // publish all states for sensors
          publishAllState();
          boot = false;
        }
        else
        {
          Serial.println(F("[MQTT]: Reconnected"));
          // publish device info
          publishDeviceInfo();
        }

        // resubscribe to mqtt command topics
        subscribeToMQTT(MQTT_DEVICE_COMMAND_TOPIC);
        subscribeToMQTT(MQTT_SIREN_COMMAND_TOPIC);
        subscribeToMQTT(MQTT_LOCK_COMMAND_TOPIC);
        subscribeToMQTT(MQTT_COVER_COMMAND_TOPIC);
        subscribeToMQTT(MQTT_COVER_POSITION_COMMAND_TOPIC);
        subscribeToMQTT(HOME_ASSISTANT_LWT_TOPIC);
      }
      else
      {
        retries++;

        Serial.println(F("[MQTT]: ERROR - The connection to the MQTT broker failed"));
        Serial.print(F("[MQTT]: MQTT username: "));
        Serial.println(MQTT_USERNAME);
        Serial.print(F("[MQTT]: MQTT password: "));
        Serial.println(MQTT_PASSWORD);
        Serial.print(F("[MQTT]: MQTT broker: "));
        Serial.println(MQTT_SERVER);
        Serial.print(F("[MQTT]: Retries: "));
        Serial.println(retries);
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

/*
  Function called to subscribe to a MQTT topic
*/
void subscribeToMQTT(char *p_topic)
{
  if (mqttClient.subscribe(p_topic))
  {
#ifdef MQTT_DEBUG
    Serial.print(F("[MQTT_DEBUG] subscribeToMQTT - Sending the MQTT subscribe succeeded for topic: "));
    Serial.println(p_topic);
#endif
  }
  else
  {
#ifdef MQTT_DEBUG
    Serial.print(F("[MQTT_DEBUG] subscribeToMQTT - ERROR, Sending the MQTT subscribe failed for topic: "));
    Serial.println(p_topic);
#endif
  }
}

/*
  Function called to publish to a MQTT topic with the given payload
*/
bool publishToMQTT(const char *p_topic, const char *p_payload, bool retain)
{
  if (mqttClient.publish(p_topic, p_payload, retain))
  {
#ifdef MQTT_DEBUG
    Serial.print(F("[MQTT_DEBUG] publishToMQTT - MQTT message published successfully, topic: "));
    Serial.print(p_topic);
    Serial.print(F(", payload: "));
    Serial.println(p_payload);
#endif
    return true;
  }
  else
  {
#ifdef MQTT_DEBUG
    Serial.println(F("[MQTT_DEBUG] publishToMQTT - ERROR, MQTT message not published, either connection lost, or message too large. Topic: "));
    Serial.print(p_topic);
    Serial.print(F(" , payload: "));
    Serial.println(p_payload);
#endif
    return false;
  }
}