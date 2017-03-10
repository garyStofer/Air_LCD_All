/* 
 * File:   HPpos.h
 * Author: Gary Stofer
 *
 * Created on Feb 2 2017, 9:35 AM
*/ 
 
#include "Arduino.h"
#include "build_opts.h" 
 
#ifdef WITH_SERVO
extern unsigned char HPpos ;
extern unsigned char HPpos_adjust( unsigned char position );
#endif
