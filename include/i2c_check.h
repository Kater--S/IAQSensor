#pragma once

#define UTIL_DEBUG 1

const char* i2cname(int address);
bool i2c_hasDevice(byte address);
int i2c_scan(bool verbose = true);