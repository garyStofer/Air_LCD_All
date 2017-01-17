// Sketch to read various sensors including Bosh BMP085/ BMP180 and SI_7021 hygrometer and display readings on 8x2 lcd display 
// using a quadrature encoder knob to switch between the individual displays.

// See file build_opt.h for #defines to enable individual measurments and their display


// Warning : This sketch needs to be compiled and run on a device with Optiboot since the watchdog doesn't work under the standard Bootloader.
// NOTE : Before uploading sketch make sure that the Board type is set to OPTIBOOT on a 32 Pin CPU -- otherwise the programmer runs at the wronng baud rate

// Last compiled and tested with Arduino IDE 1.6.6

#include "build_opts.h"		// Controls build time features

#include <LiquidCrystal.h>
#include <EEPROM.h>
#ifdef WITH_SD_CARD
#include <SD.h>
#endif
#include <avr/wdt.h>
#include "BMP085_baro.h"
#include "SI_7021.h"
#include "TMP100.h"
#include "Atmos.h"
#include "Wind.h"
#include "RPM.h"
#include "NTC.h"
#include "RC_Servo.h"
#include "AOA.h"
#include "Encoder.h"


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  BUILD OPTIONS  see file build_opt.h   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

#define LCD_COLS 8
#define LCD_ROWS 2
#define LINE1 0
#define LINE2 1
/*
// lcd special char
byte specialchar[8] = {
  0B00000,
  0B11111,
  0B11111,
  0B11111,
  0B11111,
  0B11111,
  0B11111,
  0B00000
};
lcd.createChar(7, specialchar);  // 0 thyrough 7
...
  lcd.write(7);
*/



#define VBUS_ADC_BW  (5.0*(14+6.8)/(1024*6.8))    //adc bit weight for voltage divider 14.0K and 6.8k to gnd.

#define UPDATE_PER 500   // in ms second, this is the measure interval and also the LCD update interval
#define KMpMILE 0.62137119

// units for display
#define FAR "F" 
#define CEL "C"
#define KMH "KMH    "
#define MPH "MPH    "
#define Meter " M     "
#define Feet  " ft    "
#define DegSym (char(223))  // degree symbol display character for LCD
#define DeltaSym ( char(7))
#define UpSym ( char(6))
#define DownSym ( char(5))

enum Displays {
  DISPLAY_BEGIN =0,
 #ifdef V_BUS
  V_Bus,
#endif
#ifdef WITH_BARO_HYG_TEMP
  D_Alt,
  Alt,
  Station_P,
  Rel_Hum,
  Temp,
  DewPoint,
#ifdef WetBulbTemp
  T_WetBulb,
#endif
  TD_spread,
#endif
 
#ifdef WITH_WIND
  Wind_DIR,
  Wind_SPD,
  Wind_AVG,
  Wind_GST,
#else
  #ifdef WITH_RPM
    RPM,
  #endif 
#endif
 
#ifdef WITH_AOA
  AOA,
#endif
 
#ifdef WITH_NTC
  NTC,
#endif
#ifdef WITH_SERVO
  SERVO,
#endif
  DISP_END        // this must be the last entry
};

 

static bool MetricDisplay = false;


// Global LCD control class
LiquidCrystal lcd(9, 8, 6, 7, 4, 5); // in 4 bit interface mode

static bool No_Baro = true;
static bool No_Hygro = true;
static bool No_TMP100 = true;

#ifdef WITH_SD_CARD
// SD file for datalogging 
File dataFile;
#endif


/* Note regarding contrast control */
// Contrast control simply using a PWM doesn't work because the PWM needs to be filtered with a R-C, but the LCD itself is pulling the LCD control
// input up, so the R-C from the MCU would have to be relatively low in impendance <300ohms, making for a big capacitor and lots of current
// Would have to feed the filtered voltage into an OP-Amp to drive the LCD input properly.
// For now the hardware has a resistor pair that sets the contrast.



// The Arduino IDE Setup function -- called once upon reset
void setup()
{
  unsigned err;
  // lcd special characters
byte DeltaChar[8] = {

  0B00000,
  0B00000,
  0B00100,
  0B01010,
  0B10001,
  0B11111,
  0B00000,
  0B00000
};
/*
byte DeltaChar2[8] = {
  
  0B10000,
  0B11000,
  0B10100,
  0B10010,
  0B10100,
  0B11000,
  0B10000,
  0B00000
};
*/
byte UpChar[8] = {
  0B00100,
  0B01110,
  0B11111,
  0B00100,
  0B00100,
  0B00100,
  0B00100,
  0B00000
};

byte DownChar[8] = {
  0B00100,
  0B00100,
  0B00100,
  0B00100,
  0B11111,
  0B01110,
  0B00100,
  0B00000
};


  // Setup the Encoder pins to be inputs with pullups
  pinMode(Enc_A_PIN, INPUT);    // Use external 10K pullup and 100nf to gnd for debounce
  pinMode(Enc_B_PIN, INPUT);    // Use external 10K pullup and 100nf to gnd for debounce
  pinMode(Enc_PRESS_PIN, INPUT);// Use external 10K pullup and 100nf to gnd for debounce

  // The two LEDs 
  pinMode(LED2_PIN, OUTPUT);    // BLUE
  digitalWrite( LED2_PIN, LOW); // also shared with servo

  pinMode(LED1_PIN, OUTPUT);    // RED
  digitalWrite( LED1_PIN, LOW);

  attachInterrupt(0, ISR_KnobTurn, FALLING);    // for the rotary encoder knob rotating
  attachInterrupt(1, ISR_ButtonPress, FALLING);    // for the rotary encoder knob push

#ifdef WITH_SERVO
  ServoSetup(LED2_PIN);   // The servo pulse pin is shared with the blue LED, when servo is commanded the LED lights up 
#endif

#ifdef WITH_WIND
  WindSetup() ;
#endif

#ifdef WITH_RPM
  RPM_Setup();
#endif

  wdt_enable(WDTO_2S);
  
#ifdef DEBUG
   Serial.begin(57600);
   Serial.print("AIR-LCDuino Display\n");
#endif   
   
  lcd.begin(LCD_COLS, LCD_ROWS);              // initialize the LCD columns and rows

  lcd.home ();                   // go home
 // lcd.print("AIR-LCD ");
  lcd.print("OIL TEMP ");
  lcd.setCursor ( 0, 1 );
  lcd.print("Display");
  lcd.createChar(DeltaSym, DeltaChar);  
  lcd.createChar(UpSym, UpChar);  
  lcd.createChar(DownSym, DownChar); 
  
#ifdef WITH_BARO_HYG_TEMP
  // the i2c pins -- I2C mode will overwrite this
  pinMode(18, INPUT);   // SDA
  pinMode(19, OUTPUT);  // SCL
  digitalWrite( 18, LOW);
  digitalWrite( 19, LOW);
  if ( (err = BMP085_init()) != 0)
  {
    lcd.setCursor ( 0, 0 );
    lcd.print("No Baro!");
    No_Baro = true;
  }
  else
    No_Baro = false;

  if ( (err = SI7021_init()) != 0)
  {
    lcd.setCursor ( 0, 1 );
    lcd.print("No Hygr!");
    No_Hygro = true;
  }
  else
    No_Hygro = false;

  if (No_Baro && No_Hygro)
  {
    if ( (err = TMP100_init()) != 0)
    {
      lcd.setCursor ( 0, 0 );
      lcd.print("NoTMP100");
      No_TMP100 = true;
    }
    else
    {
      No_TMP100 = false;
    }
  }
#else 
  #ifdef WITH_SD_CARD
    #define SD_SS_PIN 19
    pinMode(SD_SS_PIN, OUTPUT);  // SCL used as SS pin on SD card -- I2c and SD card can not operate at the same time
    digitalWrite( SD_SS_PIN, LOW);
  #endif
#endif

#if defined (WITH_WIND) || defined( WITH_RPM)  || defined(WITH_AOA) || !defined  WITH_BARO_HYG_TEMP
  ;
#else
  if ( No_Baro && No_Hygro && No_TMP100)
    while (1);          // Since there is nothing to measure let the watchdog catch it and reboot, Maybe a sensor gets plugged in soon.
#endif

 wdt_enable(WDTO_8S);  // set watchdog slower

 
#ifdef WITH_SD_CARD
#define SD_RECORD_PER 60
  if (  SD.begin(SD_SS_PIN) ) // the Slave Select pin is shared with SCL from I2C, therefore SD and I2C are mutually exclusive
  {
    dataFile = SD.open("datalog.txt", FILE_WRITE);
   
    if (dataFile) 
    {
      dataFile.print("Logger Start -- recording NTC temps every ");
      dataFile.print(SD_RECORD_PER);
      dataFile.println(" Seconds");
      dataFile.flush();
    }
#ifdef DEBUG    
    else
      Serial.println("No SD FILE open");
  }
  else
      Serial.println("no SD");
#else
  }
#endif
  
  if (!dataFile)
  {
    digitalWrite( SD_SS_PIN, LOW);   // Turn SS low,  LED  off
    lcd.setCursor ( 0, 1 );
    lcd.print("NoSDcard");
    delay(1000);        
  }

#endif 

#ifdef WITH_BARO_HYG_TEMP
  BMP085_startMeasure( );    // initiate initial measure cycle
  SI7021_startMeasure( );    // initiate initial measure cycle
  TMP100_startMeasure( );
#endif


  EEPROM.get( 0, MetricDisplay);

}

// checkes if a Long button press was issued and switches the EEprom and Metric display global.
void CheckToggleMetric( void )  
{
      if (LongPressCnt)   // toggle between imperial and metric display
      {
        if ( MetricDisplay != 0 )
          MetricDisplay = 0;
        else
          MetricDisplay = 1;

        EEPROM.put( 0, MetricDisplay);
        LongPressCnt = 0;
      }
}

void lcdPrintUnits(char ln, char Sym, char * units)
{
        lcd.print("      ");     
        lcd.setCursor ( 6, ln );
        lcd.print(Sym); 
        lcd.print(units);  
}
// The Arduino IDE loop function -- Called contineously
void loop()
{
  short adc_val;
  float Vbus_Volt;
  short rounded;
  float result,result2, result3;
  static char PrevEncCnt = EncoderCnt;  // used to indicate that user turned knob
  static unsigned char PrevShortPressCnt = 0;
  static char p;
  static bool REDledAlarm = false;
  static unsigned long t = millis();
  static unsigned loop_count =0;
  static unsigned timeout =0;
  int axa;
  char * units = "";
 

  
#ifdef WITH_BARO_HYG_TEMP
  float dewptC, TD_deltaC;
  float Temp_C;
#endif


  wdt_reset();

 

#ifdef WITH_WIND
  WindRead();
#endif
#ifdef WITH_RPM
  RPM_Read();
#endif
#ifdef WITH_BARO_HYG_TEMP
  BMP085_Read_Process();
  SI7021_Read_Process();
  TMP100_Read_Process();
#endif

  // anything to display ?
  if ( t + UPDATE_PER > millis() && EncoderCnt == PrevEncCnt && ShortPressCnt == PrevShortPressCnt)
    return;

  // update every n sec
  t = millis();
  loop_count++;


#ifdef WITH_BARO_HYG_TEMP
  // Note: Temperatur can come from the Barometer or Hygrometer temp reading, but hyg has more resolution
  TD_deltaC = 99.0;         // init to huge value in case no Hygrometer present,
  if (No_Hygro == false )
  {
    Temp_C = HygReading.TempC;
    dewptC = DewPt( Temp_C, HygReading.RelHum);
    TD_deltaC = Temp_C - dewptC;
  }
  else if (No_Baro == false )
    Temp_C = BaroReading.TempC ;
  else if (No_TMP100 == false)
    Temp_C = TMP100_TempC;
  else
    Temp_C = -300.0 ;  // Impossible value, will never be displayed
#endif


#ifdef WITH_BARO_HYG_TEMP
  if ( TD_deltaC < TD_DELTA_ALARM)
  {
    if (! REDledAlarm)
      EncoderCnt = TD_spread;     // switch to the Alarming display once

    REDledAlarm = true;
  }

  // This alarm illuminates the blue LED, but doesn't lock the display to it
  /*
  if (Temp_C < FREEZE_ALARM )
    digitalWrite( LED2_PIN, HIGH);
  else
    digitalWrite( LED2_PIN, LOW);
    */
#endif
 

  // Start of the individual readings display
  lcd.home(  );  // Don't use LCD clear because of screen flicker

re_eval:
  // Limit this display to valid choices, wrap around if invalid choice is reached
  
#ifdef MENUWRAP 
  if (EncoderCnt <= DISPLAY_BEGIN )
    EncoderCnt = DISP_END - 1;
    
  if (EncoderCnt >= DISP_END)
    EncoderCnt = DISPLAY_BEGIN+1;
#else  // don't wrap -- lock at end points
  if (EncoderCnt <= DISPLAY_BEGIN )
    EncoderCnt = DISPLAY_BEGIN +1;
    
  if (EncoderCnt >= DISP_END)
    EncoderCnt = DISP_END-1;
#endif



  switch (EncoderCnt)
  {
#ifdef V_BUS
    case V_Bus:
      if (timeout == 0 )
        timeout = loop_count + 10;
        
      adc_val = analogRead(VBUS_ADC);
      Vbus_Volt = adc_val * VBUS_ADC_BW;
      lcd.print("Voltage ");
      lcd.setCursor ( 0, LINE2 );
      lcd.print( Vbus_Volt ) ;
      lcd.print(" V  ");
      if (Vbus_Volt > VOLT_HIGH_ALARM  || Vbus_Volt < VOLT_LOW_ALARM )
      {
        REDledAlarm = true;
      }
      if (loop_count > timeout )
      {
        EncoderCnt++; 
      }
      break;
#endif
#ifdef WITH_BARO_HYG_TEMP
    case D_Alt:
      if (No_Baro)
      {
        EncoderCnt += EncoderDirection;
        goto re_eval;
      }
      lcd.print("Dens Alt");
      lcd.setCursor ( 0, LINE2 );
      if (MetricDisplay)
      {
        rounded = DensityAlt(BaroReading.BaromhPa, BaroReading.TempC) + 0.5;
        lcd.print(rounded ) ;
        lcd.print(Meter);
      }
      else
      {
        rounded = MtoFeet( DensityAlt(BaroReading.BaromhPa, BaroReading.TempC)) + 0.5;
        rounded /= 10;
        lcd.print(rounded * 10) ; // limit to 10ft resolution
        lcd.print(Feet);
      }

      break;

    case Alt:
      if (No_Baro)
      {
        EncoderCnt += EncoderDirection;
        goto re_eval;
      }

      if (LongPressCnt)   // entering setup
      {
        Alt_Setting_adjust( );
        LongPressCnt = 0;
        lcd.home (  );
        EncoderCnt = Alt;    // restore the Alt display item
      }

      lcd.print(" Alt *  ");
      lcd.setCursor ( 0, 1LINE2 );

      if (MetricDisplay)
      {
        rounded = Altitude(BaroReading.BaromhPa, AltimeterSetting) + 0.5;
        lcd.print( rounded);
        lcd.print(Meter);
      }
      else
      {
        rounded = MtoFeet(Altitude(BaroReading.BaromhPa, AltimeterSetting)) + 0.5;
        lcd.print( rounded);
        lcd.print(Feet);
      }
      break;

    case Station_P:
      if (No_Baro)
      {
        EncoderCnt += EncoderDirection;
        goto re_eval;
      }
      lcd.print("Pressure");
      lcd.setCursor ( 0, LINE2 );

      if (MetricDisplay)
      {
        lcd.print( BaroReading.BaromhPa );
        lcd.print("hPa");
      }
      else
      {
        lcd.print( hPaToInch(BaroReading.BaromhPa) );
        lcd.print("\"Hg");
      }
      break;

    case Rel_Hum:
      if (No_Hygro)
      {
        EncoderCnt += EncoderDirection;
        goto re_eval;
      }
      lcd.print("Humidity");
      lcd.setCursor ( 0, LINE2 );
      rounded = HygReading.RelHum  + 0.5;
      lcd.print( rounded  );
      lcd.print(" % RH  ");
      break;

    case Temp:
      if (No_Hygro && No_Baro && No_TMP100)
      {
        EncoderCnt += EncoderDirection;
        goto re_eval;
      }

      CheckToggleMetric();

      lcd.print(" Temp * ");
      lcd.setCursor ( 0, LINE2 );
      if ( MetricDisplay)
      {
        units = CEL;
        result = Temp_C;
      }
      else
      {
        units = FAR;
        result =  CtoF( Temp_C);
      }
      lcd.print(result );
      lcdPrintUnits(LINE2,DegSym,units);
      break;


    case DewPoint:
      if (No_Hygro)
      {
        EncoderCnt += EncoderDirection;
        goto re_eval;
      }
      lcd.print("DewPoint");
      

      if (MetricDisplay)
      {
        rounded = dewptC + 0.5;
        units = CEL;
       }
      else
      {
        rounded = CtoF(dewptC) + 0.5;
        units = CEL;
      }

      lcd.setCursor ( 0, LINE2 );
      lcd.print( rounded );
      lcdPrintUnits(LINE2,DegSym,units);
      break;

#ifdef WetBulbTemp

    case  T_WetBulb:

      if (No_Hygro)
      {
        EncoderCnt += EncoderDirection;
        goto re_eval;
      }

      lcd.print("Wet Bulb");
       
      if ( No_Baro)
        result = T_wetbulb_C(Temp_C, 942.0, HygReading.RelHum); // in the absence of a Barometer reading I take the pressure at ~2000ft in standard Atmos.
      else
        result = T_wetbulb_C(Temp_C, BaroReading.BaromhPa, HygReading.RelHum);
      
      if (MetricDisplay)
      {
        units = CEL;
      }
      else
      {
        units = FAR;
        result =  CtoF(result);
      }
      
      lcd.setCursor ( 0, LINE2 );
      lcd.print( result );
      lcdPrintUnits(LINE2,DegSym,units);
      break;
#endif

    case TD_spread:
      if (No_Hygro)
      {
        EncoderCnt += EncoderDirection;
        goto re_eval;
      }
      lcd.print("TDspread");

      if (MetricDisplay)
      {
        rounded = (Temp_C - dewptC) + 0.5; // for deg C
        units = CEL;
      }
      else
      {
        rounded = (CtoF(Temp_C) - CtoF(dewptC)) + 0.5; // for deg F
        units = FAR;
      }
      
      lcd.setCursor ( 0, LINE2 );
      lcd.print( rounded );
      lcdPrintUnits(LINE2,DegSym,units);
      
      if ( TD_deltaC > TD_DELTA_ALARM)
        REDledAlarm = false;            // turn alarm off
      break;
#endif
#ifdef WITH_WIND
    case Wind_SPD:
      lcd.print("Wind SPD");
      lcd.setCursor ( 0, LINE2 );
      if (MetricDisplay)
      {
        lcd.print( WindSpdMPH * KMpMILE);
        lcd.print(KMH);
      }
      else
      {
        lcd.print( WindSpdMPH);
        lcd.print(MPH);
      }
      t -= UPDATE_PER;  // fastest readout
      break;

    case Wind_GST:
      lcd.print("Wind GST");
      lcd.setCursor ( 0, LINE2 );
      if (MetricDisplay)
      {
        lcd.print( WindGustMPH * KMpMILE);
        lcd.print(KMH);
      }
      else
      {
        lcd.print( WindGustMPH);
        lcd.print(MPH);
      }
      break;

    case Wind_AVG:
      lcd.print("Wind AVG");
      lcd.setCursor ( 0, LINE2 );
      if ( MetricDisplay)
      {
        lcd.print( WindAvgMPH * KMpMILE);
        lcd.print(KMH);
      }
      else
      {
        lcd.print( WindAvgMPH);
        lcd.print(MPH);
      }
      break;

    case Wind_DIR:
      if  ( LongPressCnt)
      {
        WindDirCal(  );   // blocking until completed
        lcd.home (  );
        LongPressCnt = 0;
        EncoderCnt = Wind_DIR;    // restore the current display item
      }

      lcd.print("Wnd DIR*");
      lcd.setCursor ( 0, LINE2 );
      lcd.print( WindDir);
      lcd.print(" ");
      lcd.print(DegSym); // degree symbol
      lcd.print("     ");
      t -= UPDATE_PER;  // fastest readout

      break;

#endif
#ifdef WITH_RPM
    case RPM:
      lcd.print("  RPM   ");
      lcd.setCursor ( 0, LINE2 );
      lcd.print( RPM_);
      lcd.print("      ");
      break;
#endif

#ifdef WITH_AOA
    case AOA:   // This is basically a potentiometer position readout

      aoaRead();

      axa = V_AOA + 0.5;
      lcd.print("  AOA  ");
      lcd.setCursor ( 0, LINE2 );
      
      if (axa > 0)
        lcd.print("+");
        
      lcd.print( axa);
      lcd.print(DegSym); // degree symbol
      lcd.print("     ");

      if ( axa > 14 )
        digitalWrite( LED1_PIN, HIGH);
      else
        digitalWrite( LED1_PIN, LOW);

      if ( axa == 14 )
        digitalWrite( LED2_PIN, HIGH);
      else
        digitalWrite( LED2_PIN, LOW);

      break;
#endif

#ifdef WITH_NTC
#define OIL_TEMP_ALARM 230    // when the red light comes on, in deg F
#define TrendTime 80          //  trend window in 1/2 seconds intervals , i.e. 100 == 50 seconds
#define TrendHysteresis 10    // in 1/10 deg C makes a hysteresis for the temp trend check, applies + and - , i.e 10 is +- 1 degree 
    case NTC:
    {
      static bool disp_Delta = true;
      static int prev_reading ;  // two vars for trend display 
      static int curr_reading ;
      char trend = ' ';
      
      CheckToggleMetric();

      if (ShortPressCnt != PrevShortPressCnt)   // entering setup
      {
        disp_Delta = !disp_Delta;
      }
      
      result = ntcRead(NTC1_R25C, NTC1_BETA, NTC1_ADC);
      result2 = ntcRead(NTC2_R25C, NTC2_BETA, NTC2_ADC);
      result3 = ntcRead(NTC3_R25C, NTC3_BETA, NTC3_ADC);
      
      if (loop_count % TrendTime == 0)
      {
        prev_reading = curr_reading;
        curr_reading = (int) result3 *10 ;      // Trend must be taken from the 3rd probe that's immersed in oil for fast response
      }
      
      if (curr_reading > prev_reading+TrendHysteresis) 
          trend =UpSym;
      else if (curr_reading < prev_reading-TrendHysteresis)
        trend = DownSym;
        
      if (OIL_TEMP_ALARM > CtoF( result3))
        REDledAlarm = false;
      else
        REDledAlarm = true;  
        
      if (MetricDisplay)
      {
         units = CEL;
      }
      else
      {
        units = FAR;
        result = CtoF( result);
        result2 = CtoF( result2);
        result3 = CtoF( result3);
      }
     
      if (disp_Delta == false )
      {
        lcd.print(result,1);
        lcdPrintUnits(LINE1,DegSym,units);
        
        lcd.setCursor ( 0, LINE2 );
        lcd.print(result2,1);
        lcdPrintUnits(LINE2,DegSym,units);
      } 
      else
      {  
        lcd.print(result3,1);
        lcd.print(trend);
        lcdPrintUnits(0,DegSym,units); 
          
        lcd.setCursor ( 0, LINE2 );
        lcd.print(result - result2,1);
        lcdPrintUnits(LINE2,DeltaSym,units);
 
      }
      
     
#ifdef WITH_SD_CARD  
      // store readings every SD_RECORD_PER seconds in SD card
      if (loop_count % (1000/UPDATE_PER * SD_RECORD_PER )   == 0  && dataFile  )
      {
        dataFile.print( (loop_count / (1000/UPDATE_PER * 60 )));
        dataFile.print(',');
        dataFile.print(result);
        dataFile.print(',');
        dataFile.print (result2);
        dataFile.print(',');
        dataFile.print (result3);
        dataFile.print(',');
        dataFile.println (units);
        dataFile.flush();
      }
#endif      
      break;
#endif
    }

#ifdef WITH_SERVO
    case SERVO:
      lcd.print("Servo  *");
      lcd.setCursor ( 0, LINE2 );
      lcd.print( servo_pos);
      lcd.print(DegSym); // degree symbol
      lcd.print("     ");
      
      if (timeout == 0 )
      timeout = loop_count + 10;
        
      
      // only operate the servo when setting a new position -- consumes less power and prevents the servo to run up against a stop for long periodes
      if (ShortPressCnt != PrevShortPressCnt)   // entering setup
      {
          servo_pos = Servo_adjust( servo_pos );  
          EncoderCnt = NTC;    // go directly to teperature readout again. 

       
#ifdef WITH_SD_CARD
        if (dataFile)
        {
          dataFile.print(",,,,,,Servo: ");
          dataFile.println(servo_pos);
          dataFile.flush();
        }
#endif  

      }
      
      if (loop_count > timeout ) // prevent the parking of the display on the Servo position
      {

          EncoderCnt= NTC;
      }
      break;
#endif

    default:
      // go in the same direction as last knob input from user and re-evalute again.
      // will wrap around in re_eval;
      EncoderCnt += EncoderDirection;
      goto re_eval;
      break;
  }

#ifdef WITH_BARO_HYG_TEMP
  BMP085_startMeasure(  );    // initiate an other measure cycle on the Barometer
  SI7021_startMeasure(  );    // initiate an other measure cycle on the Hygrometer
  TMP100_startMeasure(  );    // initiate an other measure cycle on the Stand alone Thermometer
#endif

  if (PrevEncCnt != EncoderCnt )
    timeout = 0;
    
  PrevEncCnt = EncoderCnt;
  PrevShortPressCnt = ShortPressCnt;

  
#ifdef ALARMS_A
  // Blink the RED alarm led
  if (REDledAlarm)
  {
    if (digitalRead(LED1_PIN) )
      digitalWrite( LED1_PIN, LOW);
    else
      digitalWrite( LED1_PIN, HIGH);

  }
  else
    digitalWrite( LED1_PIN, LOW);
#endif
}

