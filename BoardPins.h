// The pin definitions are as per obfuscated Arduino pin defines -- see aka for ATMEL pin names as found on the MEGA328P spec sheet
#define Enc_A_PIN 2     // This generates the interrupt, providing the click  aka PD2 (Int0)
#define Enc_B_PIN 14    // This is providing the direction aka PC0,A0
#define Enc_PRESS_PIN 3 // aka PD3 ((Int1)
#define LED1_PIN 10     // aka PB2,SS Wired to RED LED, Also used as slave select pin for the SD card 
#define LED2_PIN 17     // aka PC3,A3, Wired to Blue LED, also used for the Servo pulse pin
#define VBUS_ADC 7      // ADC7
// LiquidCrystal lcd(9, 8, 6, 7, 4, 5); // in 4 bit interface mode

