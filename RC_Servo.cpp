/* 
 * File:   Servo.c
 * Author: Gary Stofer
 *
 * Created on Nov 16, 2016, 9:35 AM
 * 
 * Module to  set RC servo pulse according to knob setting. 
 */
#include "RC_Servo.h"
 
#ifdef WITH_SERVO
// ------------------------------------------
#include <LiquidCrystal.h>
#include <avr/wdt.h>

// imports from .ino file   -- messy
extern char EncoderCnt;
extern unsigned char ShortPressCnt;
extern unsigned char LongPressCnt;
extern LiquidCrystal lcd;
// ------------------------------------------

static Servo servo1;   // Servo control object
static int ServoPin;
unsigned char servo_pos = SERVO_DEFAULT;

void ServoSetup(int pin)
{
  // initialize servo position -- then remove the signal again so that it doesn't work against a stop
  ServoPin = pin;
  servo1.attach(pin);
  servo1.write(servo_pos);              // tell servo to go to position in variable 'pos'
  delay(800);    
  servo1.detach();
}

unsigned char Servo_adjust( unsigned char position )
{     
  char p;
  unsigned long time;
  
  ShortPressCnt = p = EncoderCnt =0;
  lcd.setCursor ( 0, 0 );
  lcd.print("-adjust-");
  
  time = millis();
  servo1.attach(ServoPin);
  
  while ( !ShortPressCnt ) // exit adjust loop by button press
  {
    wdt_reset();
    if (EncoderCnt !=p)
    {
      time = millis();
      
      if (EncoderCnt > p)
      position+= 5;
      else if (EncoderCnt < p)
      position-= 5;
      
      p = EncoderCnt;
    }

    if (position > SERVO_MAX)
    position = SERVO_MAX;
    else if ( position < SERVO_MIN )
    position = SERVO_MIN;

    servo1.write(position);              // tell servo to go to position in variable 'pos'
    
    lcd.setCursor ( 0, 1 );
    lcd.print( position );
    lcd.print(char(223)); // degree symbol
    lcd.print("    ");

    if ( millis() > time + 3000 )   // exit adjust loop by timeout
    break;
  }
  
  servo1.detach();
  digitalWrite(ServoPin, LOW); // make sure that the PWM pin is off again 
  return position;
}

#endif
