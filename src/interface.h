#ifndef _MAIN_H
#define _MAIN_H

#define MAX_MOTOR 4

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>

#ifdef ROCKPIS_VERSION
    /* mraa headers */
    #include "mraa/common.hpp"
    #include "mraa/gpio.hpp"
    #include "mraa/i2c.hpp"
    #include "mraa/pwm.hpp"
#elif defined(CM4_VERSION)
    #include <wiringPi.h>
    #include <wiringPiI2C.h>
#endif

#endif