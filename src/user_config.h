#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_

#define DEVICE_NAME_TEMPLATE "office_door_system_%s"
#define DEVICE_FRIENDLY_NAME "Office: Door System"
#define DEVICE_MANUFACTURER "SwartNinja"
#define DEVICE_MODEL "LoLin NodeMCU V3"
#define DEVICE_VERSION "1.0.0"

///////////////////////////////////////////////////////////////////////////
//   WIFI
///////////////////////////////////////////////////////////////////////////
#define WIFI_SSID "wifi_ssid"
#define WIFI_PASSWORD "wifi_password"
#define WIFI_SIGNAL_STRENGTH_OFFSET_VALUE 2
#define WIFI_SIGNAL_STRENGTH_INTERVAL 60000 // [ms]
#define WIFI_SIGNAL_STRENGTH_SENSOR_NAME "wifi"

///////////////////////////////////////////////////////////////////////////
//   MQTT
///////////////////////////////////////////////////////////////////////////
#define MQTT_SERVER "xxx.xxx.xxx.xxx"
#define MQTT_SERVER_PORT 1883
#define MQTT_USERNAME "mqtt_user_name"
#define MQTT_PASSWORD "mqtt_password"
#define MQTT_CHECKIN_INTERVAL 60000

#define MQTT_DEVICE_AVAILABILITY_TEMPLATE "homeassistant/%s/state" // MQTT availability: online/offline
#define MQTT_DEVICE_INFO_TEMPLATE "homeassistant/%s/info"
#define MQTT_DEVICE_COMMAND_TEMPLATE "homeassistant/%s/set"

#define MQTT_BINARY_SENSOR_TEMPLATE "homeassistant/binary_sensor/%s/%s"
#define MQTT_COVER_TEMPLATE "homeassistant/cover/%s/%s"
#define MQTT_LOCK_TEMPLATE "homeassistant/lock/%s/%s"
#define MQTT_SENSOR_TEMPLATE "homeassistant/sensor/%s/%s"
#define MQTT_SWITCH_TEMPLATE "homeassistant/switch/%s/%s"

#define MQTT_DISCOVERY_TEMPLATE "%s/config"
#define MQTT_COMMAND_TEMPLATE "%s/set"
#define MQTT_STATE_TEMPLATE "%s/state"
#define MQTT_JOIN_TEMPLATE "%s/%s"

#define MQTT_PAYLOAD_ON "ON"
#define MQTT_PAYLOAD_OFF "OFF"
#define MQTT_PAYLOAD_AVAILABLE "online"
#define MQTT_PAYLOAD_NOT_AVAILABLE "offline"
#define MQTT_PAYLOAD_LOCK "LOCK"
#define MQTT_PAYLOAD_UNLOCK "UNLOCK"
#define MQTT_PAYLOAD_CLOSE "CLOSE"
#define MQTT_PAYLOAD_OPEN "OPEN"
#define MQTT_PAYLOAD_STOP "STOP"

#define MQTT_STATE_CLOSED "closed"
#define MQTT_STATE_CLOSING "closing"
#define MQTT_STATE_OPEN "open"
#define MQTT_STATE_OPENING "opening"
#define MQTT_STATE_LOCKED "LOCKED"
#define MQTT_STATE_UNLOCKED "UNLOCKED"

// Message text for device commands
#define MQTT_CMD_RESET "reset"           // command that resets the device
#define MQTT_CMD_RESTART "restart"       // command that resets the device
#define MQTT_CMD_STATE "state"           // command to resend all state
#define MQTT_CMD_REGISTER "register"     // command to force reregistration
#define MQTT_CMD_UNREGISTER "unregister" // command to force unregistration

///////////////////////////////////////////////////////////////////////////
//    LOCK
///////////////////////////////////////////////////////////////////////////
#define LOCK_NAME "lock"
#define LOCK_PIN D0
#define LOCK_POSITION_OPEN 0
#define LOCK_POSITION_CLOSED 150

///////////////////////////////////////////////////////////////////////////
//   DOOR SENSOR
///////////////////////////////////////////////////////////////////////////
#define DOOR_SENSOR_NAME "door"
#define DOOR_PIN D6

///////////////////////////////////////////////////////////////////////////
//   SIREN
///////////////////////////////////////////////////////////////////////////
#define SIREN_NAME "siren"
#define SIREN_PIN D5

///////////////////////////////////////////////////////////////////////////
//   STEPPER MOTOR
///////////////////////////////////////////////////////////////////////////
#define STEPPER_NAME "door"
#define STEPPER_POSITION_NAME "position"
#define STEPPER_DIR_PIN D2
#define STEPPER_PUL_PIN D1
#define STEPPER_ENE_PIN D7
#define STEPPER_POSITION_OPEN 29
#define STEPPER_POSITION_CLOSED 0

///////////////////////////////////////////////////////////////////////////
//   Over-the-Air update (OTA)
///////////////////////////////////////////////////////////////////////////
#define OTA_HOSTNAME_TEMPLATE DEVICE_NAME_TEMPLATE // Used to define ArduinoOTA
#define OTA_PORT 8266                              // port 8266 by default

#endif // _USER_CONFIG_H_