/* 
 * File:   AOA.h
 * Author: Gary Stofer
 *
 * Created on Nov 7, 2015, 9:35 AM
 */

#include "Arduino.h"
#include "build_opts.h"
#ifdef WITH_AOA
 
#ifndef AOA_H
#define	AOA_H
 
#define AOA_0 1.8
#define AOA_90 2.94
 
#define AOA_ADC   1 // A1_PC1
#define AOA_ADC_BW (5.0/1024.0)

#ifdef	__cplusplus
extern "C" {
#endif

extern  float V_AOA;

extern void aoaRead(void);

#ifdef	__cplusplus
}
#endif
#endif
#endif	


