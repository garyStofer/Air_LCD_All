# Air_LCD_All
This is a collection of applications for the AIR_LCDuino board documented in https://github.com/garyStofer/AIR_LCDuino
The file build_opts.h has a bunch of #defines that can be used to enable/disable individual features to measure and display. 
Also added a rudimentary data logging capability to NTC based temperature measurments that could be expanded for other measure/display items as well.
<code>
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
</code>
