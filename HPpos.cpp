/* 
 * File:   HPpos.c
 * Author: Gary Stofer
 *
 * Created on Feb 2, 2017, 9:35 AM
 * 
 * SET HP position in % for SD card logging according to knob setting. 
 */
#include "HPpos.h"
 
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


unsigned char HPpos = 5;
unsigned char HPpos_adjust( unsigned char position )
{     
  char p;
  unsigned long time;
  
  ShortPressCnt = p = EncoderCnt =0;
  lcd.setCursor ( 0, 0 );
  lcd.print("-adjust-");
  
  time = millis();
 
  
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

    if (position > 100)
    position = 100;
    else if ( position < 5 )
    position = 5;
    
    lcd.setCursor ( 0, 1 );
    lcd.print( position );
    lcd.print("%"); // degree symbol
    lcd.print("    ");

    if ( millis() > time + 3000 )   // exit adjust loop by timeout
    break;
  }

  return position;
}

#endif
