#pragma once

#include <stdbool.h>
#include "epoll_timerfd_utilities.h"

#define LSM6DSO_ID         0x6C   // register value
#define LSM6DSO_ADDRESS	   0x6A	  // I2C Address

volatile float acceleration_mg[3];
volatile float angular_rate_dps[3];
volatile float lsm6dsoTemperature;
volatile float lps22hhTemperature;

int initI2c(void);
void closeI2c(void);