/* This file controls the build-time features */
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  BUILD OPTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// for serial debug
//#define DEBUG

// comment/uncomment for additional features 
// Note: Wind and RPM are mutually excluisive as they currently make use of the same IO pin and the Pin-Change interrupt. 
// #define V_BUS
// #define WITH_BARO_HYG_TEMP 
// #define WetBulbTemp
// #define WITH_RPM 
// #define WITH_WIND 
// #define WITH_AOA
// NOTE that AOA and NTC are mutually exclusive as both use ADC1 input
#define WITH_NTC
#define WITH_SERVO
#define WITH_SD_CARD
// #define  ALARMS_A 
// #define MENUWRAP   // allows the menu to wrap around from last to first enntry etc..
