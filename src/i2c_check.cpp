
#include <Arduino.h>
#include <Wire.h>

#include "i2c_check.h"

const char* i2cname(int address) {
  switch (address) {
    case 0x29:  return "TCS 34735: color sensor"; break;
    case 0x38:  return "VEML 6070: UV sensor"; break;
    case 0x39:  return "VEML 6070: UV sensor"; break;
    case 0x3C:  return "SSD 1306: OLED"; break;
    case 0x3F:  return "PCF/LCD"; break;
    case 0x44:  return "SHT 31: temp/hum sensor"; break;
    case 0x5A:  return "MLX 90614: IR temp sensor"; break;
    case 0x60:  return "SI 1145: IR sensor"; break;
    case 0x61:  return "SCD 30: CO2 sensor"; break;
    case 0x69:  return "SPS 30: particle matter sensor"; break;
    case 0x76:  return "BME/P280 or BME680: pressure/temp/[hum]/[iaq] sensor"; break;
    case 0x77:  return "BME/P280 or BME680: pressure/temp/[hum]/[iaq] sensor"; break;
    default:    ;
  }
  return "<unknown/other>";
}

bool i2c_hasDevice(byte address)
{
  Wire.beginTransmission(address);
  return (Wire.endTransmission() == 0);
}

int i2c_scan(bool verbose /* = true */)
{
  if (!UTIL_DEBUG) return -1;
  byte error, address;
  int nDevices;

  Serial.println("Scanning I²C bus...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I²C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      if (verbose) {
        Serial.print(" = ");
        Serial.print(i2cname(address));
      }
      Serial.println(",");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("!");
    }
    /*else
      {
      Serial.print("No device at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(",");
      }*/
  }
  if (nDevices == 0) {
    Serial.println("No I²C devices found\n");
  } else {
    Serial.print(nDevices);
    Serial.println(" found - done.\n");
  }
  return nDevices;
}
