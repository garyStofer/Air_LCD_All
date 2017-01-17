/* 
 * File:   Atmos.h
 * Author: Gary Stofer
 *
 * Created on December 18, 2013, 8:25 PM
 */

 
#include "Arduino.h"


#ifndef Atmosphere_H
#define	Atmosphere_H
#define STD_ALT_SETTING 1013.25
//With a Temp-Dewpoint delta of less that 3degC (~5degF) you can expect fog, so I set the default for the alarm a bit higher than that.
#define TD_DELTA_ALARM  4.0 // in degC when the alarm is to come on
#define VOLT_LOW_ALARM 10.0
#define VOLT_HIGH_ALARM 15.0
#define FREEZE_ALARM 1.0

#ifdef	__cplusplus
extern "C" {
#endif

extern float DewPt( float Tc, float RH );
extern float PressureAlt( float press_hPa);
extern float DensityAlt( float P_hPa, float Temp_C);
extern float Altitude ( float P_hPa, float P_sl);
extern float T_wetbulb_C(float Temp_C,float Press_hPa, float Rh);

#ifdef	__cplusplus
}
#endif

#endif	


