# IAQSensor

This is a simple Air Quality Sensor for indoor installation. It uses a BME680 air quality sensor and a PMS5003 particulate matter sensor. The controller is an Wemos D1 mini (ESP8266).

The sensor connects to a local WiFi network and publishes its measurements via MQTT – that's it.

Hardware and software might become more elaborated in future versions (e.g. WiFiManager configuration, display, …).

## Wiring

Wemos D1 mini:
D1: I2C_SCL   BME680 SCL
D2: I2C_SDA   BME680 SDA
D4: Tx -> PMS5003 Rx (Pin 4)
D5: Rx <- PMS5003 Tx (Pin 5)
– all others not connected.

BME680:
VCC(5V), GND, SCL, SDA, SDO(NC), CS(NC)

PMS5003:
pin 1 = Vcc(5V), pin 2 = Gnd, pin 4 = Rx, pin 5 = Tx, other pins NC
