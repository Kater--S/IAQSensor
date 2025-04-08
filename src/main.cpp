////////////////////////////////////////////////////////////////////////////////////
//
//  WoZiSensor.ino    particle matter sensornode with PMS5003 and BME680 sensors
//                    – based on test sketch for Adafruit PM2.5 sensor
//
//  2022-03-22 Kater_S
//
//  Libraries needed:
//    Adafruit PM25 AQI Sensor (1.0.6) + dependencies
//    BSEC Software Library by Bosch, for BME680 (1.8.149)
//
// Hardware Setup:  ESP8266 (Wemos D1 mini)
//
//  D0
//  D1: I2C_SCL   BME680
//  D2: I2C_SDA   ~
//  D3:
//  D4: Tx -> PMS5003 Rx (Pin 4)
//  D5: Rx <- PMS5003 Tx (Pin 5)
//  D6:
//  D7:
//  D8:
//  RX:
//  TX:
//  A0:
//
//  PMS5003: Pin 1 = Vcc, Pin 2 = Gnd, Pin 4 = Rx, Pin 5 = Tx
//
//  Version history:
//    0.4   2023-06-13    adapted for V1.8.149 of BSEC lib
//    0.5   2023-06-25    ported to PlatformIO
//

//////////
// setup
#define APPNAME "WoZiSensor"
#define APPVERSION "V0.5"

#define LOG_TELNET  0

#define I2C_PIN_SDA D2
#define I2C_PIN_SCL D1

/////////////
// includes

#include <Arduino.h>

#include "network_config.h"
#include "i2c_check.h"

// ESP8266 WiFi
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

// MQTT client
#include <PubSubClient.h>

// OTA mechanism
#include "OTA.h"

// sensor libraries
#include <SoftwareSerial.h>
#include "Adafruit_PM25AQI.h"
#include "bsec.h"
#define BME68X  0


// logging

#ifndef LOG_TELNET
#define LOG_TELNET  0
#endif

#if LOG_TELNET
#include <TelnetStream.h>
#define LogTarget TelnetStream
#else
#define LogTarget Serial
#endif


// variables

int loglevel = 4;
bool hasConnection = false;
WiFiClient espClient;
PubSubClient mqttClient(espClient);
const char* mqttServer = MQTT_SERVER;

String topicBase;

const int cycletime = 500;
const long publishInterval = 2 * 60 * 1000; // ms

long t_lastPublish = 0;

// for averaging:
double pm10_sum, pm25_sum, pm100_sum, press_sum;
int num_samples;


// PMS5003

SoftwareSerial pmSerial(D5, D6);
Adafruit_PM25AQI aqi = Adafruit_PM25AQI();
bool pms_ok = false;


// BME680 / BSEC
void checkIaqSensorStatus(void);
void errLeds(void);

Bsec iaqSensor;
bool bme_ok = false;

// Helper function definitions
void checkIaqSensorStatus(void)
{
  bme_ok = true;
#if BME68X
  if (iaqSensor.bsecStatus != BSEC_OK) {
    bme_ok = false;
    if (iaqSensor.bsecStatus < BSEC_OK) {
      Serial.println(String() + "BSEC error code : " + iaqSensor.bsecStatus);
      //for (;;) errLeds(); /* Halt in case of failure */
      delay(10000);
    } else {
      "BSEC warning code : " + String(iaqSensor.bsecStatus);
      Serial.println(String() + "BSEC warning code : " + iaqSensor.bsecStatus);
    }
  }

  if (iaqSensor.bme68xStatus != BME68X_OK) {
    bme_ok = false;
    if (iaqSensor.bme68xStatus < BME68X_OK) {
      Serial.println(String() + "BME680 error code : " + iaqSensor.bme68xStatus);
      //for (;;) errLeds(); /* Halt in case of failure */
      delay(10000);
    } else {
      Serial.println(String() + "BME680 warning code : " + iaqSensor.bme68xStatus);
    }
  }
#else
  if (iaqSensor.status != BSEC_OK) {
    bme_ok = false;
    if (iaqSensor.status < BSEC_OK) {
      Serial.println(String() + "BSEC error code : " + iaqSensor.status);
      //for (;;) errLeds(); /* Halt in case of failure */
      delay(10000);
    } else {
      "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(String() + "BSEC warning code : " + iaqSensor.status);
    }
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    bme_ok = false;
    if (iaqSensor.bme680Status < BME680_OK) {
      Serial.println(String() + "BME680 error code : " + iaqSensor.bme680Status);
      //for (;;) errLeds(); /* Halt in case of failure */
      delay(10000);
    } else {
      Serial.println(String() + "BME680 warning code : " + iaqSensor.bme680Status);
    }
  }
#endif
}

void errLeds(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}


void callback(char* topic, byte* payload, unsigned int length) {
  LogTarget.print("Message arrived [");
  LogTarget.print(topic);
  LogTarget.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    LogTarget.print((char)payload[i]);
  }
  LogTarget.println();

  if (strcmp(topic, "---") == 0) {
    payload[length] = 0;
    //gasCount = strtol((const char*)payload, (char **)NULL, 10);
  }
}

////////////
// setup

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  digitalWrite(LED_BUILTIN, 1);

  Serial.begin(115200);
  delay(1000);
  while (!Serial)
    delay(100);

  Serial.println(String("\n\n\n### ") + APPNAME + " " + APPVERSION + " ###");

  setupOTA("wozisensor");
  TelnetStream.begin();


  // PMS5003 sensor

  delay(1000);    // Wait one second for sensor to boot up!
  pmSerial.begin(9600);
  if (! aqi.begin_UART(&pmSerial)) { // connect to the sensor over software serial
    LogTarget.println("Could not find PM sensor!");
    pms_ok = false;
    while (1) delay(10);
  }
  LogTarget.println("PM sensor found!");
  pms_ok = true;

  // BME680 sensor

  Wire.begin(I2C_PIN_SDA, I2C_PIN_SCL);
  i2c_scan(true);
  
  #if BME68X
  iaqSensor.begin(BME68X_I2C_ADDR_HIGH, Wire);
  #else
  //iaqSensor.begin(BME680_I2C_ADDR_PRIMARY, Wire);
  iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);
  #endif
  
  LogTarget.println(String()
                    + "BSEC library version " + iaqSensor.version.major
                    + "." + iaqSensor.version.minor
                    + "." + iaqSensor.version.major_bugfix
                    + "." + iaqSensor.version.minor_bugfix);
  checkIaqSensorStatus();

  bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };
  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();

  // MQTT

  mqttClient.setServer(mqttServer, 1883);
  mqttClient.setCallback(callback);
  topicBase = String(MQTT_PREFIX "/" MQTT_LOCATION "/");
  mqttClient.publish("temp/wozisensor", "start");

  // averaging

  pm10_sum = 0.0;
  pm25_sum = 0.0;
  pm100_sum = 0.0;
  press_sum = 0.0;
  num_samples = 0;
}

#define TOPIC_STATUS  MQTT_PREFIX "/" MQTT_LOCATION "/" "status"
#define TOPIC_IN      "temp/commands"

void mqtt_reconnect() {
  LogTarget.println("mqtt_reconnect()");
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    LogTarget.print("Not connected, state = ");
    LogTarget.println(mqttClient.state());
    LogTarget.print("Attempting to connect to MQTT broker... ");
    // Attempt to connect
    if (mqttClient.connect(fullhostname, NULL, NULL,
                           TOPIC_STATUS, 2, true, "0")) {
      LogTarget.println("connected");
      delay(1000);
      // Once connected, publish our online status...
      //int ret = 
      mqttClient.publish(TOPIC_STATUS, "1");
      // ... and resubscribe
      //mqttClient.subscribe(topicGasCount);
    } else {
      LogTarget.print("failed, state = ");
      LogTarget.print(mqttClient.state());
      LogTarget.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


/*void setupTopics(const char* server, const char* topicPath, const char* clientLocation) {
  snprintf (topicPM10,   100,      "%s/%s/m_PM1.0",   topicPath, clientLocation);
  snprintf (topicPM25,   100,      "%s/%s/m_PM2.5",   topicPath, clientLocation);
  snprintf (topicPM100,  100,      "%s/%s/m_PM10.0",  topicPath, clientLocation);
  }*/

void loop() {
#ifndef ESP32_RTOS
  ArduinoOTA.handle();
#endif

  if (!mqttClient.connected()) {
    LogTarget.println("no MQTT connection! - Trying to reconnect");
    mqtt_reconnect();
  }
  mqttClient.loop();

  delay(cycletime); // actual cycletime is larger because of other delays on top
  long now = millis();


  // BME680 readout
  if (iaqSensor.run()) { // If new data is available
    LogTarget.print((String)"IAQ "
                    + (iaqSensor.temperature) + " "
                    + (iaqSensor.humidity) + " "
                    + (iaqSensor.pressure / 100.0) + " "
                    + (iaqSensor.gasResistance / 10000.0) + " "
                    + (iaqSensor.iaq) + " "
                    + (iaqSensor.iaqAccuracy) + " "
                    + (iaqSensor.staticIaq) + " "
                    + (iaqSensor.co2Equivalent / 10.0) + " "
                    + (iaqSensor.breathVocEquivalent * 100.0) + " "
                    + " # "
                   );
  } else {
    checkIaqSensorStatus();
    delay(100);  // try again in a bit!
    return;
  }

  // PMS5003 readout
  PM25_AQI_Data data;
  if (! aqi.read(&data)) {
    LogTarget.println();  // complete printed line of BME values
    LogTarget.println(" [no PM sensor data]");
    delay(100);  // try again in a bit!
    return;
  }
  LogTarget.println((String)"PM " +
                    data.pm10_standard + " " +
                    data.pm25_standard + " " +
                    data.pm100_standard + " " +
                    data.pm10_env + " " +
                    data.pm25_env + " " +
                    data.pm100_env + " " +
                    /**
                      data.particles_03um + " " +
                      data.particles_05um + " " +
                      data.particles_10um + " " +
                      data.particles_25um + " " +
                      data.particles_50um + " " +
                      data.particles_100um + " " +
                    **/
                    "");

  pm10_sum += data.pm10_env;
  pm25_sum += data.pm25_env;
  pm100_sum += data.pm100_env;
  press_sum += (iaqSensor.pressure / 100.0);
  num_samples++;


  if (now - t_lastPublish > publishInterval) {
    t_lastPublish = now;
    // build + publish MQTT messages
    mqttClient.publish((topicBase + "m_PM1.0").c_str(),       String(pm10_sum / num_samples).c_str());
    mqttClient.publish((topicBase + "m_PM2.5").c_str(),       String(pm25_sum / num_samples).c_str());
    mqttClient.publish((topicBase + "m_PM10.0").c_str(),      String(pm100_sum / num_samples).c_str());

    mqttClient.publish((topicBase + "temp").c_str(),          String(iaqSensor.temperature).c_str());
    mqttClient.publish((topicBase + "rhum").c_str(),          String(iaqSensor.humidity).c_str());
    mqttClient.publish((topicBase + "press").c_str(),         String(press_sum / num_samples).c_str());
    mqttClient.publish((topicBase + "gasR").c_str(),          String(iaqSensor.gasResistance / 1000.0).c_str());
    mqttClient.publish((topicBase + "IAQ").c_str(),           String(iaqSensor.iaq).c_str());
    mqttClient.publish((topicBase + "IAQacc").c_str(),        String(iaqSensor.iaqAccuracy).c_str());
    mqttClient.publish((topicBase + "staticIAQ").c_str(),     String(iaqSensor.staticIaq).c_str());
    mqttClient.publish((topicBase + "CO2eq").c_str(),         String(iaqSensor.co2Equivalent).c_str());
    mqttClient.publish((topicBase + "breathVOCeq").c_str(),   String(iaqSensor.breathVocEquivalent).c_str());

    // start again with averaging
    pm10_sum = 0.0;
    pm25_sum = 0.0;
    pm100_sum = 0.0;
    press_sum = 0.0;
    num_samples = 0;
  }
}
