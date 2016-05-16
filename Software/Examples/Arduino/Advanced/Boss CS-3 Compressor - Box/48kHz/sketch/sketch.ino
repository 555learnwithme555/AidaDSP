/*
 AIDA Boss CS-3 Compressor Sketch
 	
 This sketch is a "Boss CS-3 Compressor" as described in...

 This sketch was written for the Arduino, and will not work on other boards.
 	
 The circuit:
 
 Audio:
 * Input range 2.0Vrms (5.66Vpp, 8.23dBu)
 * Output range 0.9Vrms (2.5Vpp, 1.30dBu) 
 
 PC:
 * Please connect with PuTTY on Stellaris USB Serial with a PC for a minimal user interface
 
 NOTE:
 - Despite I prefer to visualize Hz, dB etc. many commercial pedals show only 0-100% settings. This
 is the reason why you'll find paramN_fake
 
 created February 2016
 by Massimo Pennazio
 
 This example code is in the public domain.
 
 */

#include <Arduino.h>
#include <pins_arduino.h>
#include <Wire.h>
#include "AidaFW.h"
#include "AidaDSP.h"
#include "LiquidCrystal.h"

#define EVER (;;)

// DEFINES USER INTERFACE
#define PREGAIN_MAX 12.0f // dB
#define PREGAIN_MIN 0.0f // dB
#define THR_MAX 6.0f // dB
#define THR_MIN -96.0f // dB
#define RATIO_MAX 100.0f
#define RATIO_MIN 1.0f
#define ATTACK_MAX 500.0f // ms
#define ATTACK_MIN 1.0f // ms
#define HOLD_MAX ATTACK_MAX
#define HOLD_MIN 1.0f // ms
#define DECAY_MAX 2000.0f // ms
#define DECAY_MIN 1.0f // ms
#define BRIGHT_MAX 18.0f // dB
#define BRIGHT_MIN -18.0f  // dB
#define POSTGAIN_MAX 24.0f // dB
#define POSTGAIN_MIN -30.0f // dB

// Master Volume
#define MASTER_VOLUME_MAX 0.00
#define MASTER_VOLUME_MIN -80.00

#define POT_THR  4 // Threshold for filtering noise on pots (adcs)

#define ON 1
#define OFF 0

#define POT1 A0
#define POT2 A1
#define POT3 A2
#define POT4 A3

#define PIN_LED  13
#define LED_1    23
#define LED_2    25
#define PUSH_1   18
#define PUSH_2   19

#define STOMPBOX // Comment to use on Aida DSP "La Prima"

// FUNCTION PROTOTYPES
void spettacolino();
void clearAndHome(void);
void setBypass(uint8_t);
void setBrightPrePost(uint8_t);

void print_menu_putty(void);
void print_menu_lcd(void);

// GLOBAL VARIABLES
// ENCODER
int32_t OldPulses = 0;

// UI
uint8_t func_counter = 0;
uint8_t old_func_counter = 0;
uint8_t bypass = OFF;

uint32_t timec=0, prevtimec=0;

// Values in param pulses are startup values for
// DSP Blocks
int32_t param1_pulses = 0; // Pre Gain
int32_t param2_pulses = 0; // Threshold
int32_t param3_pulses = 0; // Ratio
int32_t param4_pulses = 0; // Attack
int32_t param5_pulses = 0; // Hold
int32_t param6_pulses = 0; // Decay
int32_t param7_pulses = 0; // Bright
int32_t param8_pulses = 0; // PrePost Bright
int32_t param9_pulses = 0; // Master Volume
int32_t param10_pulses = 0; // Post Gain

uint8_t restore = 1;  // If 1 startup values are written to DSP

float param1_value = 0.00; 
float param2_value = 0.00; 
float param3_value = 0.00; 
float param4_value = 0.00; 
float param5_value = 0.00; 
float param6_value = 0.00;
float param7_value = 0.00;
uint8_t param8_value = 0;
float param9_value = 0.00;
float param10_value = 0.00;

equalizer_t bright_eq;
compressor_t compressor1;

char inByte = 0x00;

uint16_t pot1 = 0;
uint16_t oldpot1 = 0;
uint16_t pot2 = 0;
uint16_t oldpot2 = 0;
uint16_t pot3 = 0;
uint16_t oldpot3 = 0;
uint16_t pot4 = 0;
uint16_t oldpot4 = 0;

// Pot Filter 1
uint16_t adcvalue1 = 0;
uint32_t sum1 = 0;
uint32_t out1 = 0;
// Pot Filter 2
uint16_t adcvalue2 = 0;
uint32_t sum2 = 0;
uint32_t out2 = 0;
// Pop Filter 3
uint16_t adcvalue3 = 0;
uint32_t sum3 = 0;
uint32_t out3 = 0;
// Pop Filter 4
uint16_t adcvalue4 = 0;
uint32_t sum4 = 0;
uint32_t out4 = 0;

// Push Encoder
uint8_t push_e_count = 0;
uint8_t push_e_function = 0;

// Push 1
uint8_t push_1_lock = 0;

// Push 2
uint8_t push_2_lock = 0;

uint8_t reinitdisplaycounter = 0;

// Configure pins for LCD display
LiquidCrystal lcd(17, 16, 15, 14, 6, 7); // RS, EN, D4, D5, D6, D7

void setup()
{
  // put your setup code here, to run once:
  // I/O
  pinMode(PIN_LED, OUTPUT);
  #ifdef STOMPBOX
  pinMode(LED_1, OUTPUT);
  digitalWrite(LED_1, HIGH);
  pinMode(LED_2, OUTPUT);
  digitalWrite(LED_2, HIGH);
  pinMode(PUSH_1, INPUT_PULLUP);
  //attachInterrupt(5, push1_isr, FALLING); 
  pinMode(PUSH_2, INPUT_PULLUP);
  //attachInterrupt(4, push2_isr, FALLING); 
  #endif

  // open the USBSerial port
  Serial.begin(115200);
  clearAndHome();
  Serial.println(F("Aida DSP control with ARDUINO")); // Welcome message
  Serial.print(F("0x"));
  Serial.println((DEVICE_ADDR_7bit<<1)&~0x01, HEX);
  
  // LCD Display
  #ifdef STOMPBOX
  lcd.begin(16, 2); // set up the LCD's number of columns and rows
  lcd.setCursor(0, 0);
  lcd.print(F("Aida DSP Box")); // Print a message to the LCD.
  lcd.setCursor(0, 1);
  lcd.print(F("Tubescr. V0.1"));
  #endif

  // DSP board
  InitAida();	// Initialize DSP board
  digitalWrite(RESET, HIGH); // Wake up DSP
  delay(100);  // Start-up delay for DSP
  program_download();    // Here we load program, parameters and hardware configuration to DSP
  delay(20);
  check_program(); // !!!Debug!!!
  delay(5);
  check_param(); // !!!Debug!!!
  delay(5);
  check_config(); // !!!Debug!!!
  delay(2);
  spettacolino();
   
  // -- Initialization 
  
  // Param Values
  param1_value = processencoder(PREGAIN_MIN, PREGAIN_MAX, param1_pulses); // Pre Gain
  param2_value = processencoder(THR_MIN, THR_MAX, param2_pulses); // Threshold
  param3_value = processencoder(RATIO_MIN, RATIO_MAX, param3_pulses); // Ratio
  param4_value = processencoder(ATTACK_MIN, ATTACK_MAX, param4_pulses); // Attack
  param5_value = processencoder(HOLD_MIN, HOLD_MAX, param5_pulses); // Hold
  param6_value = processencoder(DECAY_MIN, DECAY_MAX, param6_pulses); // Decay
  param7_value = processencoder(BRIGHT_MIN, BRIGHT_MAX, param7_pulses); // Bright
  param8_value = selectorwithencoder(param8_pulses, 1); // Bright Pre Post
  param9_value = processencoder(MASTER_VOLUME_MIN, MASTER_VOLUME_MAX, param9_pulses); // Master Volume
  param10_value = processencoder(POSTGAIN_MIN, POSTGAIN_MAX, param10_pulses); // Post Gain
  
  // Pre Gain
  gainCell(DEVICE_ADDR_7bit, PreGainAddr, param1_value);
  delayMicroseconds(100);
  
  // Bright Filter Pre & Post
  bright_eq.S = 1.0;
  bright_eq.gain = 0.0; 
  bright_eq.f0 = 2000.0;
  bright_eq.boost = param7_value;
  bright_eq.type = HighShelf;
  bright_eq.phase = false;
  SetBrightPrePost(param8_value);
  delayMicroseconds(100);
  
  compressor1.threshold = param2_value;
  compressor1.ratio = param3_value;
  compressor1.attack = param4_value;
  compressor1.hold = param5_value;
  compressor1.decay = param6_value;
  compressor1.postgain = param10_value;
  CompressorRMS(DEVICE_ADDR_7bit, Compressor1Addr, &compressor1);
  delayMicroseconds(100);
  
  // Master Volume
  MasterVolumeMono(DEVICE_ADDR_7bit, MasterVolumeAddr, pow(10, param9_value/20.0)); // Set Master Volume 
  delayMicroseconds(100); 
   
  MuteOff();  // Mute DAC Off
  
  // Bypass status init = disable
  bypass = 0;
  muxnoiseless(DEVICE_ADDR_7bit, BypassAddr, 1); // FX
  digitalWrite(LED_2, LOW); // Led 2 On
}

void loop()
{
  // put your main code here, to run repeatedly:
  
  if(Serial.available()>0) // Serial parser to speed up operations
  {
    inByte = Serial.read();
    switch(inByte)
    {
      case '1':
        func_counter = 0; 
        break;
      case '2':
        func_counter = 1; 
        break;
      case '3':
        func_counter = 2; 
        break;
      case '4':
        func_counter = 3; 
        break;
      case '5':
        func_counter = 4; 
        break;
      case '6':
        func_counter = 5; 
        break;
      case '7':
        func_counter = 6; 
        break;
      case '8':
        func_counter = 7; 
        break;
      case '9':
        func_counter = 8; 
        break;
      case '0': // 10
        func_counter = 9; 
        break;
      
      default:
        // Do nothing
        break;
    }
  }
  
  
  #ifdef STOMBOX
  adcvalue1 = analogRead(POT1);
  sum1 = ((((64)-1) * sum1)+((uint32_t)adcvalue1*(64)))/(64);
  out1 = sum1/64;
  pot1 = out1;
  if(!isinrange(pot1, oldpot1, POT_THR))
  {
    //func_counter=0;
    
    oldpot1 = pot1;
  }
  
  adcvalue2 = analogRead(POT2);
  sum2 = ((((64)-1) * sum2)+((uint32_t)adcvalue2*(64)))/(64);
  out2 = sum2/64;
  pot2 = out2;
  if(!isinrange(pot2, oldpot2, POT_THR))
  {
    //func_counter=1;
    
    oldpot2 = pot2;
  }
  
  adcvalue3 = analogRead(POT3);
  sum3 = ((((64)-1) * sum3)+((uint32_t)adcvalue3*(64)))/(64);
  out3 = sum3/64;
  pot3 = out3;
  if(!isinrange(pot3, oldpot3, POT_THR))
  {
    //func_counter=2;
    
    oldpot3 = pot3;
  }
  
  adcvalue4 = analogRead(POT4);
  sum4 = ((((64)-1) * sum4)+((uint32_t)adcvalue4*(64)))/(64);
  out4 = sum4/64;
  pot4 = out4;
  if(!isinrange(pot4, oldpot4, POT_THR))
  {
    //func_counter=3;
    
    oldpot4 = pot4;
  }
  #endif

  if(digitalRead(ENC_PUSH)==LOW)  
  {
    digitalWrite(PIN_LED, HIGH);
    delay(50);  // debounce
    if(digitalRead(ENC_PUSH)==LOW)
    {
      push_e_count++;
    }   
  }
  else
  {
    if(push_e_count>0 && push_e_count<10)
      push_e_function = 1;
    else if(push_e_count>10 && push_e_count<30)
      push_e_function = 2;
    else
      push_e_function = 0;  // No function triggered on switch
    push_e_count = 0;
    digitalWrite(PIN_LED, LOW);
  }
  
  if(push_e_function==1)
  {
    func_counter++;
    if(func_counter==5)
      func_counter=0;
  }
  else if(push_e_function==2)
  {
    #ifndef STOMPBOX
    bypass ^= 1; // Use 2nd function as bypass switch in normal mode
    #endif
  }
  
  if(digitalRead(PUSH_1)==LOW)
  {
    delay(50);  // debounce
    if(digitalRead(PUSH_1)==LOW)
    {
      if(push_1_lock != 1)
      {
        push_1_lock = 1;
        // PUSH_1 PRESSED
      }
    }
  }
  else
  {
    push_1_lock = 0;
  }
  
  if(digitalRead(PUSH_2)==LOW)
  {
    delay(50);  // debounce
    if(digitalRead(PUSH_2)==LOW)
    {
      if(push_2_lock != 1)
      {
        push_2_lock = 1;
        // PUSH_2 PRESSED
        bypass ^= 1;
      }
    }
  }
  else
  {
    push_2_lock = 0;
  }
  
  timec = millis();
  if(timec-prevtimec >= 250)  // Here we manage control interface every 250ms
  { 
    clearAndHome();    // !!!Warning use with real terminal emulation program
    Serial.println(F("********************************"));
    Serial.println(F("*    User control interface    *"));
    Serial.println(F("*    AIDA Boss CS-3 Sketch     *"));
    Serial.println(F("********************************"));
    Serial.write('\n');
    Serial.print(F("Encoder pulses: "));
    Serial.println(getPulses(), DEC);
    Serial.write('\n');
    
    #ifdef STOMPBOX
    reinitdisplaycounter++;
    if(reinitdisplaycounter==4) // Sometimes display takes noise and corrupts its RAM...
    {
      lcd.begin(16, 2); // set up the LCD's number of columns and rows
      reinitdisplaycounter = 0;
    }
    #endif
    
    // Using PUSH_1 and LED_1
    setBypass(bypass); // Using PUSH_2 and LED_2
    
    if(old_func_counter != func_counter)
    {
      restore = 1;
      old_func_counter = func_counter;
    }
    switch(func_counter)
    {
    case 0: // Pre Gain
      if(restore)
      {
        restore = 0;
        setPulses(param1_pulses);
      }
      param1_pulses = getPulses();
      param1_value = processencoder(PREGAIN_MIN, PREGAIN_MAX, param1_pulses); // Pre Gain
      gainCell(DEVICE_ADDR_7bit, PreGainAddr, param1_value);
      break;
    case 1: // Threshold
      if(restore)
      {
        restore = 0;
        setPulses(param2_pulses);
      }
      param2_pulses = getPulses();
      param2_value = processencoder(THR_MIN, THR_MAX, param2_pulses); // Threshold
      compressor1.threshold = param2_value;
      CompressorRMS(DEVICE_ADDR_7bit, Compressor1Addr, &compressor1);
      break;
    case 2: // Ratio
      if(restore)
      {
        restore = 0;
        setPulses(param3_pulses);
      }
      param3_pulses = getPulses();
      param3_value = processencoder(RATIO_MIN, RATIO_MAX, param3_pulses); // Ratio
      compressor1.ratio = param3_value;
      CompressorRMS(DEVICE_ADDR_7bit, Compressor1Addr, &compressor1);
      break;
    case 3: // Attack
      if(restore)
      {
        restore = 0;
        setPulses(param4_pulses);
      }
      param4_pulses = getPulses();
      param4_value = processencoder(ATTACK_MIN, ATTACK_MAX, param4_pulses); // Attack
      compressor1.attack = param4_value;
      CompressorRMS(DEVICE_ADDR_7bit, Compressor1Addr, &compressor1);
      break;
    case 4: // Hold
      if(restore)
      {
        restore = 0;
        setPulses(param5_pulses);
      }
      param5_pulses = getPulses();
      param5_value = processencoder(HOLD_MIN, HOLD_MAX, param5_pulses); // Hold
      compressor1.hold = param5_value;
      CompressorRMS(DEVICE_ADDR_7bit, Compressor1Addr, &compressor1);
      break;
    case 5: // Decay
      if(restore)
      {
        restore = 0;
        setPulses(param6_pulses);
      }
      param6_pulses = getPulses();
      param6_value = processencoder(DECAY_MIN, DECAY_MAX, param6_pulses); // Decay
      compressor1.decay = param6_value;
      CompressorRMS(DEVICE_ADDR_7bit, Compressor1Addr, &compressor1);
      break;  
    case 6: // Bright
      if(restore)
      {
        restore = 0;
        setPulses(param7_pulses);
      }
      param7_pulses = getPulses();
      param7_value = processencoder(BRIGHT_MIN, BRIGHT_MAX, param7_pulses); // Bright
      bright_eq.boost = param7_value;
      SetBrightPrePost(param8_value);
      break;
    case 7: // Bright Pre / Post
      if(restore)
      {
        restore = 0;
        setPulses(param8_pulses);
      }
      param8_pulses = getPulses();
      param8_value = selectorwithencoder(param8_pulses, 1); // Bright Pre Post
      SetBrightPrePost(param8_value);
      break;
    case 8: // Master Volume
      if(restore)
      {
        restore = 0;
        setPulses(param8_pulses);
      }
      param8_pulses = getPulses();
      param9_value = processencoder(MASTER_VOLUME_MIN, MASTER_VOLUME_MAX, param9_pulses); // Master Volume
      MasterVolumeMono(DEVICE_ADDR_7bit, MasterVolumeAddr, pow(10, param9_value/20.0)); // Set Master Volume
      break;
    case 9: // Post Gain
      if(restore)
      {
        restore = 0;
        setPulses(param10_pulses);
      }
      param10_pulses = getPulses();
      param10_value = processencoder(POSTGAIN_MIN, POSTGAIN_MAX, param10_pulses); // Post Gain
      compressor1.postgain = param10_value;
      CompressorRMS(DEVICE_ADDR_7bit, Compressor1Addr, &compressor1);
      break;
    } // End switch func_counter

    // Display information for user
    #ifndef STOMPBOX
    print_menu_putty();
    #else
    print_menu_lcd();
    #endif

    prevtimec = timec;
  } // End if 1000ms tick
} // End void loop

void spettacolino()
{
  byte i;
  byte statusc = 0x00;

  for(i=0;i<6;i++)
  {
    statusc ^= 1;
    digitalWrite(PIN_LED, statusc);
    digitalWrite(LED_1, statusc);
    digitalWrite(LED_2, statusc);
    delay(250);
  }
  digitalWrite(PIN_LED, HIGH);
  digitalWrite(LED_1, HIGH);
  digitalWrite(LED_2, HIGH);
}

void clearAndHome(void)
{
  Serial.write(0x1b); // ESC
  Serial.print(F("[2J")); // clear screen
  Serial.write(0x1b); // ESC
  Serial.print(F("[H")); // cursor to home
}

void print_menu_putty(void)
{
  // Print menu
  Serial.print(F("Effect status: "));
  if(bypass)
    Serial.println(F("bypass"));
  else
    Serial.println(F("on"));
  Serial.write('\n');  
  if(func_counter==0)
    Serial.print(F("    "));
  Serial.print(F("PreGain: "));
  Serial.print(param1_value, 1);
  Serial.println(F(" dB"));
  if(func_counter==1)
    Serial.print(F("    "));
  Serial.print(F("Thr: "));
  Serial.print(param2_value, 1);
  Serial.println(F(" dB"));
  if(func_counter==2)
    Serial.print(F("    "));
  Serial.print(F("Ratio: "));
  Serial.println(param3_value, 1);
  if(func_counter==3)
    Serial.print(F("    "));
  Serial.print(F("Attack: "));
  Serial.print(param4_value, 1);
  Serial.println(F(" ms"));
  if(func_counter==4)
    Serial.print(F("    "));
  Serial.print(F("Hold: "));
  Serial.print(param5_value, 1);
  Serial.println(F(" ms"));
  if(func_counter==5)
    Serial.print(F("    "));
  Serial.print(F("Decay: "));
  Serial.print(param6_value, 1);
  Serial.println(F(" ms"));
  if(func_counter==6)
    Serial.print(F("    "));
  Serial.print(F("Bright: "));
  Serial.print(param7_value, 1);
  Serial.println(F(" dB"));
  if(func_counter==7)
    Serial.print(F("    "));
  Serial.print(F("Pos: "));
  if(param8_value==1)
    Serial.println(F(" Pre"));
  else if(param8_value==2)
    Serial.println(F(" Post"));
  if(func_counter==8)
    Serial.print(F("    "));
  Serial.print(F("Vol: "));
  Serial.print(param9_value, 1);
  Serial.println(F(" dB"));
  if(func_counter==9)
    Serial.print(F("    "));
  Serial.print(F("Makeup: "));
  Serial.print(param10_value, 1);
  Serial.println(F(" dB"));
  
  Serial.write('\n');
  Serial.print(F("Active item: "));
  Serial.println(func_counter, DEC);
}

void print_menu_lcd(void)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  if(bypass)
    lcd.print(F("BYPASS"));
  else
  {
    lcd.print(F("Boss CS-3")); 
    lcd.setCursor(0, 1);
    switch(func_counter)
    {
      case 0:
        lcd.print(F("PreGain:"));
        lcd.print(param1_value, 1);
        lcd.print(F("dB"));
        break;
      case 1:
        lcd.print(F("Thr:"));
        lcd.print(param2_value, 1);
        lcd.print(F("dB"));
        break;
      case 2:
        lcd.print(F("Ratio:"));
        lcd.print(param3_value, 1);
        break;
      case 3:
        lcd.print(F("Attack:"));
        lcd.print(param4_value, 1);
        lcd.print(F("ms"));
        break;
      case 4:
        lcd.print(F("Hold:"));
        lcd.print(param5_value, 1);
        lcd.print(F("ms"));
        break;
      case 5:
        lcd.print(F("Decay:"));
        lcd.print(param6_value, 1);
        lcd.print(F("ms"));
        break;
      case 6:
        lcd.print(F("Bright:"));
        lcd.print(param7_value, 1);
        lcd.print(F("dB"));
        break;
      case 7:
        lcd.print(F("Pos:"));
        if(param8_value==1)
          lcd.print(F("Pre"));
        else if(param8_value==2)
          lcd.print(F("Post"));
        break;
      case 8:
        lcd.print(F("Vol:"));
        lcd.print(param9_value, 1);
        lcd.print(F("dB"));
        break;
      case 9:
        lcd.print(F("Makeup:"));
        lcd.print(param10_value, 1);
        lcd.print(F("dB"));
        break;
    }
  }
}

void setBypass(uint8_t status)
{
  static uint8_t oldstatus = OFF;
  
  if(oldstatus != status)
  {
    if(status == ON)
    {
      muxnoiseless(DEVICE_ADDR_7bit, BypassAddr, 2); // Bypass
      digitalWrite(LED_2, HIGH);
    }
    else
    {
      muxnoiseless(DEVICE_ADDR_7bit, BypassAddr, 1); // FX
      digitalWrite(LED_2, LOW);
    }
    oldstatus = status;
  }
}

void SetBrightPrePost(uint8_t value)
{
  if(value==1) // Pre
  {
    bright_eq.onoff = ON;
    EQ2ndOrd(DEVICE_ADDR_7bit, PreBrightAddr, &bright_eq);
    delayMicroseconds(100);
    bright_eq.onoff = OFF;
    EQ2ndOrd(DEVICE_ADDR_7bit, PostBrightAddr, &bright_eq);
    delayMicroseconds(100);
  }
  else if(value==2) // Post
  {
    bright_eq.onoff = OFF;
    EQ2ndOrd(DEVICE_ADDR_7bit, PreBrightAddr, &bright_eq);
    delayMicroseconds(100);
    bright_eq.onoff = ON;
    EQ2ndOrd(DEVICE_ADDR_7bit, PostBrightAddr, &bright_eq);
    //delayMicroseconds(100);
  }
}

void MuteOff(void)
{
  AIDA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_7bit, CoreRegisterR4Addr, CoreRegisterR4Size, CoreRegisterR4Data);    // Mute DAC Off
}

void MuteOn(void)
{
  AIDA_WRITE_REGISTER_BLOCK( DEVICE_ADDR_7bit, CoreRegisterR0Addr, CoreRegisterR0Size, CoreRegisterR0Data ); // Mute DAC On
}

/*void push1_isr(void)
{
  
}*/

/*void push2_isr(void)
{

}*/

void check_program(void) 
{
  uint8_t value_wr = 0;
  uint8_t buff_r[5];
  uint8_t value_r;
  uint16_t addr = ProgramDataAddr;
  uint16_t i, j, errors;
  
  Serial.println(F("Program checking..."));
  
  errors = 0;
  for(i=0;i<ProgramDataSize;i+=5) // Program address 1024 to 2047
  {
    memset(buff_r, 0, 5);
    AIDA_READ_REGISTER(DEVICE_ADDR_7bit, addr, 5, buff_r); 
    for(j=0;j<5;j++)
    {
      #ifdef __AVR__
      //value_wr = pgm_read_byte_far(&ProgramDataData[i+j]);
      value_wr = pgm_read_byte_near(&ProgramDataData[i+j]);
      #else
      value_wr = ProgramDataData[i+j];
      #endif
      value_r = buff_r[j];
      if(value_wr != value_r)
      {
        errors++;
        break;
      }
    }
    if(errors)
      break;
    addr++;
    delayMicroseconds(100);
  }

  if(errors)
  {
    //Serial.print(F("i: "));
    //Serial.println(i, DEC);
    //Serial.print(F("j: "));
    //Serial.println(j, DEC);
    Serial.print(errors, DEC);
    Serial.println(F(" errors during Program download")); 
    Serial.print(F("Address: "));
    Serial.println(addr, DEC);
    Serial.print(F("Written = "));
    Serial.print(F("0x"));
    Serial.println(value_wr, HEX);
    Serial.print(F("Readed = "));
    Serial.print(F("0x"));
    Serial.println(value_r, HEX);
    while(1);
  }
  else
  {
    Serial.println(F("Program OK"));
  }
}

void check_param(void)
{
  uint8_t value_wr = 0;
  uint8_t buff_r[4];
  uint8_t value_r;
  uint16_t addr = regParamAddr;
  uint16_t i, j, errors;
  
  Serial.println(F("Parameter checking..."));
  
  errors = 0;
  for(i=0;i<regParamSize;i+=4) // 0 to 1023
  {
    memset(buff_r, 0, 4);
    AIDA_READ_REGISTER(DEVICE_ADDR_7bit, addr, 4, buff_r); 
    for(j=0;j<4;j++)
    {
      #ifdef __AVR__
      //value_wr = pgm_read_byte_far(&regParamData[i+j]);
      value_wr = pgm_read_byte_near(&regParamData[i+j]);
      #else
      value_wr = regParamData[i+j];
      #endif
      value_r = buff_r[j];
      if(j==0)
        value_wr&=0x0F;
      if(value_wr != value_r)
      {
        errors++;
        break;
      }
    }
    if(errors)
      break;
    addr++;
    delayMicroseconds(100);
  }
  
  if(errors)
  {
    Serial.print(errors, DEC);
    Serial.println(F(" errors during Reg Param download")); 
    Serial.print(F("Address: "));
    Serial.println(addr, DEC);
    Serial.print(F("Written = "));
    Serial.print(F("0x"));
    Serial.println(value_wr, HEX);
    Serial.print(F("Readed = "));
    Serial.print(F("0x"));
    Serial.println(value_r, HEX);
    while(1);
  }
  else
  {
    Serial.println(F("Reg Param OK"));
  }
}

void check_config(void)
{
  uint8_t value_wr = 0;
  uint8_t buff_r[HWConFigurationSize];
  uint8_t value_r;
  uint16_t addr = HWConFigurationAddr;
  uint16_t i, errors;
  
  Serial.println(F("HW Config checking..."));
  
  errors = 0;
  memset(buff_r, 0, HWConFigurationSize);
  AIDA_READ_REGISTER(DEVICE_ADDR_7bit, addr, HWConFigurationSize, buff_r); // Read all parameters in one block   
  
  for(i=0;i<HWConFigurationSize;i++) //  2076 to 2087 
  {
    #ifdef __AVR__
    //value_wr = pgm_read_byte_far(&HWConFigurationData[i]);
    value_wr = pgm_read_byte_near(&HWConFigurationData[i]);
    #else
    value_wr = HWConFigurationData[i];
    #endif
    value_r = buff_r[i];
    if(value_wr != value_r)
    {
      errors++;
      break;
    }
  }
  
  if(errors)
  {
    Serial.print(errors, DEC);
    Serial.println(F(" errors during HW config download")); 
    Serial.print(F("Address: "));
    Serial.println(addr, DEC);
    Serial.print(F("Written = "));
    Serial.print(F("0x"));
    Serial.println(value_wr, HEX);
    Serial.print(F("Readed = "));
    Serial.print(F("0x"));
    Serial.println(value_r, HEX);
    while(1);
  }
  else
  {
    Serial.println(F("HW Config OK"));
  }
}

