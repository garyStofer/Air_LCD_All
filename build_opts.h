/* This file controls the build-time features */
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  BUILD OPTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
#include "BoardPins.h"
// for serial debug
//#define DEBUG

// comment/uncomment for additional features 

#define V_BUS
// #define WITH_BARO_HYG_TEMP // Note that SD_CARD and WITH_BARO_HYG_TEMP are mutually exclusive because they share a pin
// #define WetBulbTemp  // relies on having a baro/hyg sensor
// #define WITH_RPM   // Note: Wind and RPM are mutually excluisive as they currently make use of the same IO pin and the Pin-Change interrupt. 
// #define WITH_WIND  // Note: Wind and RPM are mutually excluisive as they currently make use of the same IO pin and the Pin-Change interrupt. 
// #define WITH_AOA   // NOTE that AOA, FUELP and NTC are mutually exclusive as both use ADC1 input
// #define WITH_NTC   //  ""
   #define WITH_FUELP // ""
// #define WITH_SERVO
//#define WITH_SD_CARD  // Note that SD_CARD and WITH_BARO_HYG_TEMP are mutually exclusive because they share a pin
#define  ALARMS_A 
// #define MENUWRAP   // allows the menu to wrap around from last to first enntry etc..
