/* 
 * File:   NTC.h
 * Author: Gary Stofer
 *
 * Created on Nov 4, 2016, 9:35 AM
 * 
 * Module to read temperature from and attached NTC device and compute temperatur using the R@25C and BETA methode. 
 */
#include "Arduino.h"
#include "build_opts.h"
#include "math.h"

#ifdef WITH_NTC
 
#ifndef NTC_H
#define	NTC_H


 
// Resistance at 25C and Beta koefficient for the simpler methode	
// NTC3 is westach 1/8 NPT sensor , Westach Temperature Sender 399S7
//  212o at 100C, 1.750K at 37.777C , 30K at -19.62C == R25C=3000, Beta = 3902
// #define NTC3_R25C 3000// Ohms at 25C  WESTACH probe in RV
// #define NTC3_BETA 4000	// K(B25/100)
#define NTC3_R25C 10e3 // Ohms at 25C- MOUSER PN594-NTCLE213E31003FLB
#define NTC3_BETA 3435  // K(B25/100)


// NTC2 is little yellow bead, MOUSER PN594-NTCLE213E31003FLB
#define NTC2_R25C 10e3 // Ohms at 25C
#define NTC2_BETA 3435  // K(B25/100)

// NTC1 same as NTC2
#define NTC1_R25C 10e3 // Ohms at 25C -- MOUSER PN594-NTCLE213E31003FLB
#define NTC1_BETA 3435  // K(B25/100)

 
#define NTC1_ADC    1 // A1_PC1
#define NTC2_ADC    2
#define NTC3_ADC    6

#define V_REF 5.0     // 5.0V from VCC -- Since the measurment is ratiometric to VCC as well the actual Vref precision falls out of the equation
#define R_Ref 1000.0  // series resistior from VCC to NTC used as reference resistor to calculate R_Ntc

#define NTC_BW (V_REF/1024.0)		// ADC bit weight 

#ifdef	__cplusplus
extern "C" {
#endif



extern float ntcRead(float r25c, int Beta, int adcn);

#ifdef	__cplusplus
}
#endif
#endif
#endif	


