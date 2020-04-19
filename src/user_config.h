#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_

///////////////////////////////////////////////////////////////////////////
//   WIFI
///////////////////////////////////////////////////////////////////////////
#define WIFI_SSID "wifi_ssid"
#define WIFI_PASSWORD "wifi_password"
#define WIFI_QUALITY_OFFSET_VALUE 2
#define WIFI_QUALITY_INTERVAL 50000 // [ms]
#define WIFI_QUALITY_SENSOR_NAME "wifi"

///////////////////////////////////////////////////////////////////////////
//   MQTT
///////////////////////////////////////////////////////////////////////////
#define MQTT_SERVER "xxx.xxx.xxx.xxx"
#define MQTT_SERVER_PORT 1883
#define MQTT_USERNAME "mqtt_user_name"
#define MQTT_PASSWORD "mqtt_password"

#define MQTT_AVAILABILITY_TOPIC_TEMPLATE "%s/status" // MQTT availability: online/offline
#define MQTT_SENSOR_TOPIC_TEMPLATE "%s/sensor/%s"
#define MQTT_SENSOR_STATE_TOPIC_TEMPLATE "%s/%s/state"
#define MQTT_SENSOR_COMMAND_TOPIC_TEMPLATE "cmnd/%s/%s"

#define MQTT_PAYLOAD_ON "ON"
#define MQTT_PAYLOAD_OFF "OFF"
#define MQTT_DOOR_LOCK_STATE_LOCKED "LOCKED"
#define MQTT_DOOR_LOCK_STATE_UNLOCKED "UNLOCKED"
#define MQTT_DOOR_LOCK_PAYLOAD_LOCK "LOCK"
#define MQTT_DOOR_LOCK_PAYLOAD_UNLOCK "UNLOCK"

///////////////////////////////////////////////////////////////////////////
//   DOOR LOCK
///////////////////////////////////////////////////////////////////////////
#define DOOR_LOCK_STATE_TOPIC     "doorlock"
#define DOOR_LOCK_PIN             D5
#define DOOR_LOCK_STEPS_TO_OPEN 0
#define DOOR_LOCK_STEPS_TO_CLOSE 150

///////////////////////////////////////////////////////////////////////////
//   DOOR SENSOR
///////////////////////////////////////////////////////////////////////////
#define DOOR_SENSOR_STATE_TOPIC "door"
#define DOOR_PIN D6

///////////////////////////////////////////////////////////////////////////
//   SIREN
///////////////////////////////////////////////////////////////////////////
#define STEPPER_STATE_TOPIC       "position"
#define STEPPER_CMD_ACTION        "action"
#define STEPPER_CMD_POSITION      "position"
#define STEPPER_SPEED             300                   //Defines the speed in RPM for your stepper motor
#define STEPPER_STEPS_PER_REV     200                   //Defines the number of pulses that is required for the stepper to rotate 360 degrees
#define STEPPER_MICROSTEPPING     2                     //Defines microstepping 0 = no microstepping, 1 = 1/2 stepping, 2 = 1/4 stepping, 4 = 1/8 stepping
#define DRIVER_INVERTED_SLEEP     1                     //Defines sleep while pin high.  If your motor will not rotate freely when on boot, comment this line out.

#define STEPS_TO_CLOSE            57                    //Defines the number of steps needed to open or close fully

#define STEPPER_DIR_PIN           D2                    //Marked as D2 on the NodeMCU 4
#define STEPPER_STEP_PIN          D1                    //Marked as D1 on the NodeMCU 5
#define STEPPER_SLEEP_PIN         D7                   //Marked as D7 on the NodeMCU 13
#define STEPPER_MICROSTEP_1_PIN   14                   //Does not need to be connected unless you want to dynamically change microstepping (not normal)
#define STEPPER_MICROSTEP_2_PIN   12                   //Does not need to be connected unless you want to dynamically change microstepping (not normal)

///////////////////////////////////////////////////////////////////////////
//   Over-the-Air update (OTA)
///////////////////////////////////////////////////////////////////////////
#define OTA_HOSTNAME_TEMPLATE "OfficeDoor_%s"
#define OTA_PORT 8266  // port 8266 by default

#endif  // _USER_CONFIG_H_