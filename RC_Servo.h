/* 
 * File:   RC_Servo.h
 * Author: Gary Stofer
 *
 * Created on Nov 16, 2016, 9:35 AM
 * 
 * Module to  set RC servo pulse according to knob setting. 
 */
#include "Arduino.h"
#include "build_opts.h" 
 
#ifdef WITH_SERVO
#ifndef RC_SERVO_H
#define RC_SERVO_H

#include <Servo.h>    // This is from the Servo library 
/* servo stuf */
#define SERVO_MIN 80       // position in degrees -- 90 is neutral 
#define SERVO_MAX 140
#define SERVO_DEFAULT SERVO_MAX    // startup default position 
  

extern unsigned char servo_pos;
extern void ServoSetup(int pin);
extern unsigned char Servo_adjust( unsigned char position );

#endif 
#endif
