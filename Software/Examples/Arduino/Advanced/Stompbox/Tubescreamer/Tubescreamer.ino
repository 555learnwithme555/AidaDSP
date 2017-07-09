/*
 AIDA Tubescreamer Sketch
 
 created February 2016
 by Massimo Pennazio
 
 This example code is in the public domain.
 
 */

#include <Arduino.h>
#include <pins_arduino.h>
#include <Wire.h>
#include "AidaFW.h"
#include "AidaDSPStompbox.h"

#define EVER (;;)

// DEFINES USER INTERFACE
#define DRIVE_MAX 100.0f
#define DRIVE_MIN 0.0f
#define TONE_MAX 4180.0f // Hz
#define TONE_MIN 792.0f  // Hz
#define MIX_MAX 100.0
#define MIX_MIN 0.0

// Master Volume
#define MASTER_VOLUME_MAX 0.00
#define MASTER_VOLUME_MIN -80.00

#define POT_THR 4 // Threshold for filtering noise on pots (adcs)

#define ON 1
#define OFF 0
#define DEBOUNCETIME  50u

#define TONE POT1
#define DRIVE POT2
#define VOLUME POT3
#define BLEND POT4

//#define READBACK // Comment to do not perform readback of input guitar signal level inside DSP

// FUNCTION PROTOTYPES
void spettacolino();
void clearAndHome(void);
void debounceSwitch1(void);
void debounceSwitch2(void);
void setBypass(uint8_t);
void setDrive(float);
void setMix(float);
void toggleTech(uint8_t);
void setVolume(float);

void print_menu_putty(void);

// GLOBAL VARIABLES
// ENCODER
int32_t OldPulses = 0;

// UI
uint8_t func_counter = 0;
uint8_t old_func_counter = 0;
uint8_t bypass;
uint8_t toggletech;

uint32_t timec=0, prevtimec=0;

float param1_value = 0.00; // Drive
float param2_value = 0.00; // Tone
float param2_fake = 0.00;
float param3_value = 0.00; 
float param3_fake = 0.00;
float param4_value = 0;

equalizer_t tone_eq;
equalizer_t opamp_eq;
equalizer_t antialias_eq; 
equalizer_t postdist_eq;

float readback1 = 0.00;
float maxreadback1 = 0.00;

// Pot variables
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

void setup()
{
  // put your setup code here, to run once:
  // I/O

  // open the USBSerial port
  Serial.begin(115200);
  //while(!SerialUSB);
  //clearAndHome();
  Serial.println(F("Aida DSP Stompbox")); // Welcome message
  Serial.print(F("0x"));
  Serial.println((DEVICE_ADDR_7bit<<1)&~0x01, HEX);

  // DSP board
  InitAida();	// Initialize DSP board
  spettacolino();
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
  //spettacolino();
   
  // -- Initialization 
  
  // Param Values
  adcvalue1 = analogRead(DRIVE);
  adcvalue2 = analogRead(TONE);
  //adcvalue3 = analogRead(VOLUME);
  //adcvalue4 = analogRead(BLEND);
  param1_value = processpot(DRIVE_MIN, DRIVE_MAX, adcvalue1); // Drive.
  param2_value = processpot(TONE_MIN, TONE_MAX, adcvalue2); // Tone
  //param3_value = processpot(MASTER_VOLUME_MIN, MASTER_VOLUME_MAX, adcvalue3); // Master Volume
  //param4_value = processpot(MIX_MIN, MIX_MAX, adcvalue4); // Technology Mix
  
  // Pre Gain
  gainCell(DEVICE_ADDR_7bit, PreGainAddr, 2.83);
  delayMicroseconds(100);
  
  // Opamp Highpass Filter
  opamp_eq.gain = 1.0; 
  opamp_eq.f0 = 720.0; // Hz
  opamp_eq.type = Highpass;
  opamp_eq.phase = false;
  opamp_eq.onoff = ON;
  EQ1stOrd(DEVICE_ADDR_7bit, OpampAddr, &opamp_eq);
  delayMicroseconds(100);
  
  hard_clip(DEVICE_ADDR_7bit, PreGainLimitAddr, 5.0, -5.0);
  delayMicroseconds(100);
  
  // Drive
  setDrive(param1_value);
  delayMicroseconds(100);
  
  // Anti-aliasing filter (lowpass before distortion)
  antialias_eq.gain = 0.0; 
  antialias_eq.f0 = 6000; // 8x oversampling @ 96k
  //antialias_eq.f0 = 12000.0; // 4x oversampling @ 96k
  antialias_eq.type = Lowpass;
  antialias_eq.phase = false;
  antialias_eq.onoff = ON;
  EQ1stOrd(DEVICE_ADDR_7bit, AntiAliasingFAddr, &antialias_eq);
  delayMicroseconds(100);
  
  hard_clip(DEVICE_ADDR_7bit, PostGainLimitAddr, 5.0, -5.0);
  delayMicroseconds(100);
  
  //setMix(param4_value);
  //delayMicroseconds(100);
  
  // Post-distortion lowpass filter
  postdist_eq.gain = 0.0; 
  postdist_eq.f0 = 24000.0;
  postdist_eq.type = Lowpass;
  postdist_eq.phase = false;
  postdist_eq.onoff = OFF;
  EQ1stOrd(DEVICE_ADDR_7bit, PostFAddr, &postdist_eq);
  delayMicroseconds(100);
  
  // Tone
  tone_eq.gain = 0.0; 
  tone_eq.f0 = param2_value;
  tone_eq.type = Lowpass;
  tone_eq.phase = false;
  tone_eq.onoff = ON;
  EQ1stOrd(DEVICE_ADDR_7bit, ToneAddr, &tone_eq);
  delayMicroseconds(100);
  
  // Master Volume
  //setVolume(param3_value);
  //delayMicroseconds(100);

  // Initial technology status read from RANDOMSW
  if(digitalRead(RANDOMSW) == LOW)
    toggletech = ON;
  else
    toggletech = OFF;
  toggleTech(toggletech);

  // Initial bypass status read from FOOTSW
  if(digitalRead(FOOTSW) == LOW)
    bypass = ON;
  else
    bypass = OFF;
  setBypass(bypass);
   
  MuteOff();  // Mute DAC Off
}

void loop()
{
  // put your main code here, to run repeatedly:
  adcvalue1 = analogRead(DRIVE);
  sum1 = ((((64)-1) * sum1)+((uint32_t)adcvalue1*(64)))/(64);
  out1 = sum1/64;
  pot1 = out1;
  if(!isinrange(pot1, oldpot1, POT_THR))
  {
    func_counter=0;
    param1_value = processpot(DRIVE_MIN, DRIVE_MAX, pot1); // Drive
    setDrive(param1_value);
    oldpot1 = pot1;
  }
  
  adcvalue2 = analogRead(TONE);
  sum2 = ((((64)-1) * sum2)+((uint32_t)adcvalue2*(64)))/(64);
  out2 = sum2/64;
  pot2 = out2;
  if(!isinrange(pot2, oldpot2, POT_THR))
  {
    func_counter=1;
    param2_value = processpot(TONE_MIN, TONE_MAX, pot2); // Tone
    param2_fake = processpot(0.0, 100.0, pot2); // Only for vizualization
    tone_eq.f0 = param2_value;
    EQ1stOrd(DEVICE_ADDR_7bit, ToneAddr, &tone_eq);
    oldpot2 = pot2;
  }
  
  // adcvalue3 = analogRead(VOLUME);
  // sum3 = ((((64)-1) * sum3)+((uint32_t)adcvalue3*(64)))/(64);
  // out3 = sum3/64;
  // pot3 = out3;
  // if(!isinrange(pot3, oldpot3, POT_THR))
  // {
    // func_counter=2;
    // param3_value = processpot(MASTER_VOLUME_MIN, MASTER_VOLUME_MAX, pot3); // Master Volume
    // param3_fake = processpot(0.0, 100.0, pot3); // Only for visualization
    // MasterVolumeMono(DEVICE_ADDR_7bit, MasterVolumeAddr, pow(10, param3_value/20.0)); // Set Master Volume
    // oldpot3 = pot3;
  // }
  
  // adcvalue4 = analogRead(BLEND);
  // sum4 = ((((64)-1) * sum4)+((uint32_t)adcvalue4*(64)))/(64);
  // out4 = sum4/64;
  // pot4 = out4;
  // if(!isinrange(pot4, oldpot4, POT_THR))
  // {
    // func_counter=3;
    // param4_value = processpot(MIX_MIN, MIX_MAX, pot4); // Technology Mix
    // setMix(param4_value);
    
    // oldpot4 = pot4;
  // }

  debounceSwitch1();
  debounceSwitch2();
  
  timec = millis();
  if(timec-prevtimec >= 250)  // Here we manage control interface every 250ms
  { 
    #ifdef READBACK
    readBack(DEVICE_ADDR_7bit, ReadBackAlg1Addr, 0x011E, &readback1); // raw abs value
    if(readback1 > maxreadback1)
      maxreadback1 = readback1;
    #endif
    
    // Display information for user
    print_menu_putty();

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
    digitalWrite(STATUS_LED, statusc);
    delay(250);
  }
  digitalWrite(STATUS_LED, LOW);
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
  clearAndHome();    // !!!Warning use with real terminal emulation program
  Serial.println(F("********************************"));
  Serial.println(F("*   User control interface     *"));
  Serial.println(F("*   AIDA Tubescreamer 96k      *"));
  Serial.println(F("********************************"));
  Serial.write('\n');
  Serial.print(F("Encoder pulses: "));
  Serial.println(getPulses(), DEC);
  Serial.write('\n');
  
  #ifdef READBACK
  Serial.print(F(" Raw abs : ")); // Print linear values (max +/-1.00) for readback values
  Serial.println(maxreadback1, 3);
  #endif
  
  Serial.print(F("Effect status: "));
  if(bypass)
    Serial.println(F("bypass"));
  else
    Serial.println(F("on"));
  Serial.write('\n');  
  if(func_counter==0)
    Serial.print(F("    "));
  Serial.print(F("Drive: "));
  Serial.print(param1_value, 1);
  Serial.println(F(" %"));
  if(func_counter==1)
    Serial.print(F("    "));
  Serial.print(F("Tone: "));
  Serial.print(param2_fake, 1);
  Serial.println(F(" %"));
  //Serial.print(param2_value, 2);
  //Serial.println(F(" Hz"));
  if(func_counter==2)
    Serial.print(F("    "));
  Serial.print(F("Vol: "));
  //Serial.print(param3_fake, 1);
  //Serial.println(F(" %"));
  Serial.print(param3_value, 1);
  Serial.println(F(" dB"));
  if(func_counter==3)
    Serial.print(F("    "));
  Serial.print(F("Blend: "));
  Serial.print(param4_value, 1);
  Serial.println(F(" %"));
  if(param4_value<5.0)
    Serial.println(F("Si"));
  else if(param4_value>95.0)
    Serial.println(F("Ge"));
  
  Serial.write('\n');
  Serial.print(F("Active item: "));
  Serial.println(func_counter, DEC);
}

void setBypass(uint8_t value)
{
  static uint8_t oldvalue = ON;
  
  if(oldvalue != value)
  {
    if(value == ON)
    {
      muxnoiseless(DEVICE_ADDR_7bit, BypassAddr, 2); // Bypass
      digitalWrite(STATUS_LED, LOW);
    }
    else
    {
      muxnoiseless(DEVICE_ADDR_7bit, BypassAddr, 1); // FX
      digitalWrite(STATUS_LED, HIGH);
    }
    oldvalue = value;
  }
}

void setVolume(float volume)
{
  MasterVolumeMono(DEVICE_ADDR_7bit, MasterVolumeAddr, pow(10, volume/20.0)); // Set Master Volume 
}

void setMix(float percent)
{
  float value = 0.00;

  if(percent>100)
    percent = 100;
  value = percent/100.00;

  // MIX
  AIDA_SAFELOAD_WRITE_VALUE(DEVICE_ADDR_7bit, TechMixAddr, false, 1.00-value);  // Si	 
  AIDA_SAFELOAD_WRITE_VALUE(DEVICE_ADDR_7bit, TechMixAddr+1, true, value);   // Ge
}

// This function set the linear gain. We cannot have
// gain more than 15.999 on a single cell, so we use two cells to achieve 
// the desired gain
void setDrive(float value)
{
  float drive,a1,a2;
  float coefficients[3];
  uint16_t address = 0x00;
 
  address = OpampAddr;

  if(value == 0.00)
    drive = 1.00;
  else
    drive = ((value * 13.0)/100) + 1.0;
  
  gainCell(DEVICE_ADDR_7bit, Drive1Addr, 10.0);
  delayMicroseconds(100);
  gainCell(DEVICE_ADDR_7bit, Drive2Addr, drive);
  delayMicroseconds(100);
}

/* Deprecated version 
// This function calculates the opamp stage filter + gain
// unfortunately we can't set such enormous gain in a single 2nd order
// cell due to fixed point coefficient saturation
// so this function is implemented in another way 
// see my post here https://ez.analog.com/thread/82012
void setDrive(float value)
{
  static float oldvalue = 0.00;
  float drive,a1,a2;
  float coefficients[3];
  uint16_t address = 0x00;
  
  address = OpampAddr;
  
  if(oldvalue != value)
  {
    if(value == 0.00)
      drive = 1.00;
    else
      drive = value / 100.0; // Scaling to 0-1.0 range...

    a1 = 251.3184 + (drive * 2256.0);
    a2 = 21.2064;
    
    coefficients[0] = (1 + a1) / (1 + a2); // B0
    coefficients[1] = (1 - a1) / (1 + a2); // B1
    coefficients[2] = (1 - a2) / (1 + a2); // A1
  
    // Write coefficients to Sigma DSP
    #ifdef ADAU144x
      AIDA_SW_SAFELOAD_WRITE_VALUE(DEVICE_ADDR_7bit, address++, false, coefficients[0]);
      AIDA_SW_SAFELOAD_WRITE_VALUE(DEVICE_ADDR_7bit, address++, false, coefficients[1]);
      AIDA_SW_SAFELOAD_WRITE_VALUE(DEVICE_ADDR_7bit, address, true, coefficients[2]);
    #else
      AIDA_SAFELOAD_WRITE_VALUE(DEVICE_ADDR_7bit, address++, false, coefficients[0]);
      AIDA_SAFELOAD_WRITE_VALUE(DEVICE_ADDR_7bit, address++, false, coefficients[1]);
      AIDA_SAFELOAD_WRITE_VALUE(DEVICE_ADDR_7bit, address, true, coefficients[2]);
    #endif
    
    oldvalue = value;
  }
}
*/

void toggleTech(uint8_t toggle)
{
  func_counter = 3; // Display blend parameter
  
  if(toggle==ON)
  {
    param4_value = 0.00;
    setMix(param4_value); // Si
    param3_value = -3.0; // Compensate for difference in volumes between technologies
    setVolume(param3_value); // Master Volume
  }
  else
  {
    param4_value = 100.0;
    setMix(param4_value); // Ge
    param3_value = -2.0; // Compensate for difference in volumes between technologies
    setVolume(param3_value); // Master Volume
  }
}

void debounceSwitch1(void)
{
  static int reading1 = HIGH;
  static int buttonState1; // the current reading from the input pin
  static int lastButtonState1 = LOW; // the previous reading from the input pin
  static unsigned long lastDebounceTime1 = 0; // the last time the output pin was toggled
  
  reading1 = digitalRead(FOOTSW);

  // If the switch changed, due to noise or pressing:
  if (reading1!= lastButtonState1) {
    // reset the debouncing timer
    lastDebounceTime1 = millis();
    lastButtonState1 = reading1;
  }
  
  if ((millis() - lastDebounceTime1) > DEBOUNCETIME) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading1 != buttonState1) {
      buttonState1 = reading1;

      if (buttonState1 == LOW) // Switch
        bypass = ON;
      else
        bypass = OFF;
      setBypass(bypass); // Using FOOTSW and STATUS_LED
    }
  }
}

void debounceSwitch2(void)
{
  static int reading2 = HIGH;
  static int buttonState2; // the current reading from the input pin
  static int lastButtonState2 = LOW; // the previous reading from the input pin
  static unsigned long lastDebounceTime2 = 0; // the last time the output pin was toggled
  
  reading2 = digitalRead(RANDOMSW);

  // If the switch changed, due to noise or pressing:
  if (reading2!= lastButtonState2) {
    // reset the debouncing timer
    lastDebounceTime2 = millis();
  }
  
  if ((millis() - lastDebounceTime2) > DEBOUNCETIME) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading2 != buttonState2) {
      buttonState2 = reading2;

      if (buttonState2 == LOW) // Switch
        toggletech = ON;
      else
        toggletech = OFF;
      toggleTech(toggletech);
    }
  }
  lastButtonState2 = reading2;
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

